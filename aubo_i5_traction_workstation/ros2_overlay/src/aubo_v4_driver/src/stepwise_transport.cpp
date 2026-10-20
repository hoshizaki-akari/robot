#include "aubo_v4_driver/stepwise_transport.hpp"

#include <aubo_driver/serviceinterface.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>
#include "aubo_v4_driver/state_validation.hpp"

namespace aubo_v4_driver
{
namespace vendor = aubo_robot_namespace;
double steady_seconds()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

struct StepwiseTransport::Impl
{
  struct Job {StepwiseCommand command; std::array<double, 6> seed;};
  std::unique_ptr<ServiceInterface> send, read, stop;
  bool send_connected{false}, read_connected{false}, stop_connected{false};
  TransportOptions options;
  mutable std::mutex mutex;
  std::condition_variable changed;
  std::thread sender, reader, stopper;
  std::atomic<bool> shutdown{false}, allowed{false};
  std::atomic<std::uint64_t> allowed_epoch{0};
  bool busy{false}, stop_in_progress{false}, pose_valid{false};
  std::atomic<bool> ever_dispatched{false};
  std::optional<Job> job;
  std::optional<std::uint64_t> stop_job;
  std::uint64_t last_submitted_id{0};
  std::deque<TransportResult> results;
  TransportSample latest;
  double previous_dispatch_at{0.0};
  vendor::JointVelcAccParam cached_joint_velocity{}, cached_joint_acceleration{};
  bool joint_profile_cached{false};
  std::atomic<bool> invalidate_follow_profile{true};

  // These getters report this SDK session's profile, not measured physical speed.
  bool apply_profile(double speed)
  {
    const double requested_acceleration = speed <= 0.002 ?
      options.acceleration_mps2 : options.fast_acceleration_mps2;
    if (send->robotServiceSetGlobalMoveEndMaxLineAcc(requested_acceleration) != vendor::ErrnoSucc ||
      send->robotServiceSetGlobalMoveEndMaxLineVelc(speed) != vendor::ErrnoSucc)
    {
      return false;
    }
    double velocity = NAN, acceleration = NAN;
    send->robotServiceGetGlobalMoveEndMaxLineVelc(velocity);
    send->robotServiceGetGlobalMoveEndMaxLineAcc(acceleration);
    if (!std::isfinite(velocity) || !std::isfinite(acceleration) ||
      std::abs(velocity - speed) > 1e-9 ||
      std::abs(acceleration - requested_acceleration) > 1e-9) {return false;}
    std::lock_guard<std::mutex> lock(mutex);
    latest.sdk_speed_mps = velocity;
    latest.sdk_acceleration_mps2 = acceleration;
    return true;
  }

  bool apply_follow_profile(const Job & current, const vendor::wayPoint_S & target)
  {
    if (!current.command.follow_joint_mode) {
      joint_profile_cached = false;
      return send->robotServiceSetNoArrivalAhead() == 0;
    }
    if (!options.allow_follow_mode) {return false;}
    if (invalidate_follow_profile.exchange(false)) {joint_profile_cached = false;}
    if (current.command.hold_position) {return joint_profile_cached;}
    double squared = 0.0;
    for (unsigned i = 0; i < 3; ++i) {
      const double delta = current.command.target[i] - current.command.start[i];
      squared += delta * delta;
    }
    const double length = std::sqrt(squared);
    if (!options.fixed_follow_profile && (!std::isfinite(length) || length <= 0.0)) {return false;}
    vendor::JointVelcAccParam velocity{}, acceleration{}, got_velocity{}, got_acceleration{};
    bool unchanged = joint_profile_cached;
    for (unsigned i = 0; i < 6; ++i) {
      const double ratio = options.fixed_follow_profile ? 0.0 :
        std::abs(target.jointpos[i] - current.seed[i]) / length;
      // Keep low-speed force regulation proportional. Cache only small (5%)
      // IK/profile changes, rather than rounding tiny velocities up to a large
      // bucket that would unexpectedly accelerate near-target regulation.
      velocity.jointPara[i] =
        options.fixed_follow_profile ? options.follow_joint_speed_radps : std::clamp(
        ratio * current.command.speed_mps, 0.0001, options.follow_joint_speed_radps);
      acceleration.jointPara[i] =
        options.fixed_follow_profile ? options.follow_joint_acceleration_radps2 : std::clamp(
        ratio * options.fast_acceleration_mps2, 0.001, options.follow_joint_acceleration_radps2);
      unchanged = unchanged &&
        std::abs(velocity.jointPara[i] - cached_joint_velocity.jointPara[i]) <=
        0.05 * velocity.jointPara[i] &&
        std::abs(acceleration.jointPara[i] - cached_joint_acceleration.jointPara[i]) <=
        0.05 * acceleration.jointPara[i];
    }
    if (unchanged) {return true;}
    if (send->robotServiceSetGlobalMoveJointMaxVelc(velocity) != 0 ||
      send->robotServiceSetGlobalMoveJointMaxAcc(acceleration) != 0 ||
      send->robotServiceSetArrivalAheadTimeMode(0.05) != 0) {return false;}
    send->robotServiceGetGlobalMoveJointMaxVelc(got_velocity);
    send->robotServiceGetGlobalMoveJointMaxAcc(got_acceleration);
    for (unsigned i = 0; i < 6; ++i) {
      if (!std::isfinite(got_velocity.jointPara[i]) ||
        !std::isfinite(got_acceleration.jointPara[i]) ||
        std::abs(got_velocity.jointPara[i] - velocity.jointPara[i]) > 1e-9 ||
        std::abs(got_acceleration.jointPara[i] - acceleration.jointPara[i]) > 1e-9) {return false;}
    }
    cached_joint_velocity = velocity;
    cached_joint_acceleration = acceleration;
    joint_profile_cached = true;
    std::lock_guard<std::mutex> lock(mutex);
    ++latest.follow_profile_updates;
    latest.follow_joint_speed_max_radps = *std::max_element(
      velocity.jointPara, velocity.jointPara + 6);
    latest.follow_joint_acceleration_max_radps2 = *std::max_element(
      acceleration.jointPara, acceleration.jointPara + 6);
    return true;
  }

  ~Impl()
  {
    // No detached SDK work may outlive the owner. Vendor calls have their
    // own request timeout; shutdown cannot promise to interrupt those calls.
    allowed = false;
    {
      std::lock_guard<std::mutex> lock(mutex);
      job.reset();
      if (stop_connected && ever_dispatched) {stop_job = ++allowed_epoch;}
    }
    changed.notify_all();
    // Give the urgent-stop owner a chance to interrupt an outstanding call.
    if (stop_connected && ever_dispatched && stopper.joinable()) {
      std::unique_lock<std::mutex> lock(mutex);
      changed.wait_for(
        lock, std::chrono::seconds(3),
        [this]() {return !stop_job && !stop_in_progress && !busy;});
    }
    // Pair the predicate update with the waiters' mutex. An atomic alone
    // permits notify to land between their predicate check and sleep.
    {
      std::lock_guard<std::mutex> lock(mutex);
      shutdown = true;
    }
    changed.notify_all();
    if (sender.joinable()) {sender.join();}
    if (stopper.joinable()) {stopper.join();}
    if (reader.joinable()) {reader.join();}
    // All sender work has ended: this final stop cannot be overtaken by an
    // older motion dispatch. It is not a promise of bounded vendor latency.
    if (stop_connected && ever_dispatched) {
      try {send->robotMoveFastStop();} catch (...) {}
    }
    if (read_connected) {
      read->robotServiceRegisterRealTimeRoadPointCallback(nullptr, nullptr);
      read->robotServiceLogout();
    }
    if (send_connected) {send->robotServiceLogout();}
    if (stop_connected) {stop->robotServiceLogout();}
  }

  static void waypoint(const vendor::wayPoint_S * point, void * context)
  {
    if (point == nullptr || context == nullptr) {return;}
    auto & self = *static_cast<Impl *>(context);
    PoseSample pose;
    pose.position = {point->cartPos.position.x, point->cartPos.position.y,
      point->cartPos.position.z};
    pose.orientation = {point->orientation.x, point->orientation.y,
      point->orientation.z, point->orientation.w};
    std::copy_n(point->jointpos, 6, pose.joints.begin());
    std::string invalid;
    const auto time = steady_seconds();
    std::lock_guard<std::mutex> lock(self.mutex);
    if (!valid_pose_sample(pose, &invalid)) {
      self.pose_valid = false;
      self.latest.error = invalid;
      self.latest.last_telemetry_failure = "invalid waypoint: " + invalid;
      self.latest.feedback.healthy = false;
      return;
    }
    self.pose_valid = true;
    double speed = 0.0;
    const double dt = time - self.latest.feedback.received_at;
    self.latest.waypoint_interval_s = self.latest.feedback.sequence > 0 ? dt : 0.0;
    if (self.latest.feedback.sequence > 0 && dt > 0.0 && dt <= 1.0) {
      double squared = 0.0;
      for (std::size_t i = 0; i < 3; ++i) {
        const double delta = pose.position[i] - self.latest.feedback.position[i];
        squared += delta * delta;
      }
      speed = std::sqrt(squared) / dt;
    }
    self.latest.feedback.position = pose.position;
    self.latest.feedback.orientation = pose.orientation;
    self.latest.feedback.received_at = time;
    self.latest.feedback.speed_mps = speed;
    ++self.latest.feedback.sequence;
    self.latest.joints = pose.joints;
  }

  void result(TransportResult value, bool sender_finished = false)
  {
    std::lock_guard<std::mutex> lock(mutex);
    // Publishing an ACK also releases the only sender slot atomically. A
    // tracking replacement may arrive immediately after the consumer sees it.
    if (sender_finished) {busy = false;}
    // At most one motion and one stop can be in flight. Bound even unexpected
    // repeated stop results; overflow is itself a diagnostic fault.
    if (results.size() >= 8) {
      results.clear();
      latest.error = "stepwise result mailbox overflow";
      value.success = false;
      value.detail = latest.error;
    }
    results.push_back(std::move(value));
    changed.notify_all();
  }

  void send_loop()
  {
    while (!shutdown) {
      Job current;
      {
        std::unique_lock<std::mutex> lock(mutex);
        changed.wait(lock, [this]() {return shutdown || job.has_value();});
        if (shutdown) {break;}
        current = *job;
        job.reset();
        busy = true;
      }
      TransportResult reply;
      reply.id = current.command.id;
      reply.epoch = current.command.epoch;
      try {
        vendor::Pos position{};
        position.x = current.command.target[0];
        position.y = current.command.target[1];
        position.z = current.command.target[2];
        vendor::Ori orientation{};
        orientation.x = current.command.orientation[0];
        orientation.y = current.command.orientation[1];
        orientation.z = current.command.orientation[2];
        orientation.w = current.command.orientation[3];
        vendor::wayPoint_S target{};
        const double ik_started = steady_seconds();
        int ik = vendor::ErrnoSucc;
        if (current.command.hold_position) {
          // Hold exactly the sampled joints. Re-solving a Cartesian hold pose
          // can introduce a branch/roundoff correction on an already moving arm.
          std::copy(current.seed.begin(), current.seed.end(), target.jointpos);
        } else {
          ik = send->robotServiceRobotIk(current.seed.data(), position, orientation, target);
        }
        const double ik_duration = steady_seconds() - ik_started;
        double maximum_joint_delta = 0.0;
        std::size_t maximum_joint_index = 0;
        std::string ik_failure;
        if (ik != vendor::ErrnoSucc) {
          ik_failure = "robotServiceRobotIk SDK result " + std::to_string(ik);
        } else {
          for (std::size_t i = 0; i < 6; ++i) {
            if (!std::isfinite(target.jointpos[i])) {
              ik_failure = "robotServiceRobotIk returned nonfinite joint " + std::to_string(i + 1);
              break;
            }
            const double delta = std::abs(target.jointpos[i] - current.seed[i]);
            if (delta > maximum_joint_delta) {
              maximum_joint_delta = delta;
              maximum_joint_index = i;
            }
          }
          const double proposed_delta = maximum_joint_delta;
          if (ik_failure.empty() && current.command.follow_joint_mode &&
            options.subdivide_follow_joint_steps &&
            maximum_joint_delta > options.maximum_joint_step_rad &&
            maximum_joint_delta <= 2.0 * options.maximum_joint_step_rad)
          {
            // Execute a shorter joint-space segment, not the oversized target.
            // A different IK branch remains an error, never a queued trajectory.
            const double scale = 0.9 * options.maximum_joint_step_rad / maximum_joint_delta;
            for (unsigned i = 0; i < 6; ++i) {
              target.jointpos[i] = current.seed[i] + scale * (target.jointpos[i] - current.seed[i]);
            }
            maximum_joint_delta *= scale;
            std::lock_guard<std::mutex> lock(mutex);
            ++latest.follow_joint_subdivisions;
          }
          {
            std::lock_guard<std::mutex> lock(mutex);
            latest.proposed_ik_max_joint_delta_rad = proposed_delta;
          }
          if (ik_failure.empty() && maximum_joint_delta > options.maximum_joint_step_rad) {
            ik_failure = "IK joint " + std::to_string(maximum_joint_index + 1) +
              " delta " + std::to_string(maximum_joint_delta) + " rad exceeds " +
              std::to_string(options.maximum_joint_step_rad) + " rad continuity bound";
          }
        }
        {
          std::lock_guard<std::mutex> lock(mutex);
          latest.last_ik_result = ik;
          latest.ik_duration_s = ik_duration;
          latest.maximum_ik_duration_s = std::max(latest.maximum_ik_duration_s, ik_duration);
          latest.last_ik_max_joint_delta_rad = maximum_joint_delta;
          latest.last_ik_joint_index = ik == vendor::ErrnoSucc ? maximum_joint_index + 1 : 0;
          latest.last_ik_failure = ik_failure;
        }
        if (!ik_failure.empty()) {
          reply.detail = ik_failure;
        } else if (!allowed || current.command.epoch != allowed_epoch || shutdown) {
          reply.detail = "stepwise command cancelled before dispatch";
        } else if (!current.command.follow_joint_mode &&
          !apply_profile(current.command.speed_mps))
        {
          reply.detail = "stepwise SDK speed profile readback failed";
        } else if (!apply_follow_profile(current, target)) {
          reply.detail = "motion mode/joint profile rejected";
        } else if (!allowed || current.command.epoch != allowed_epoch || shutdown) {
          reply.detail = "stepwise command cancelled during profile update";
        } else {
          // Absolute bounded endpoint: no speculative queued trajectory, no
          // automatic resend. 'false' does NOT promise zero request latency.
          {
            std::lock_guard<std::mutex> lock(mutex);
            ever_dispatched = true;
          }
          const double dispatch_at = steady_seconds();
          const int code = current.command.follow_joint_mode ?
            send->robotServiceFollowModeJointMove(target.jointpos) :
            send->robotServiceLineMove(target, false);
          const double motion_duration = steady_seconds() - dispatch_at;
          reply.success = code == vendor::ErrnoSucc;
          {
            std::lock_guard<std::mutex> lock(mutex);
            ++latest.motion_ack_count;
            latest.motion_call_duration_s = motion_duration;
            latest.maximum_motion_call_duration_s = std::max(
              latest.maximum_motion_call_duration_s, motion_duration);
            latest.dispatch_interval_s = previous_dispatch_at > 0.0 ?
              dispatch_at - previous_dispatch_at : 0.0;
            latest.maximum_dispatch_interval_s = std::max(
              latest.maximum_dispatch_interval_s, latest.dispatch_interval_s);
            previous_dispatch_at = current.command.hold_position ? 0.0 : dispatch_at;
            latest.last_command_id = current.command.id;
            latest.requested_speed_mps = current.command.speed_mps;
            latest.follow_joint_mode = current.command.follow_joint_mode;
          }
          reply.detail = std::string(
            current.command.follow_joint_mode ?
            "FollowModeJointMove SDK result " : "ordinary LineMove SDK result ") + std::to_string(
            code);
        }
      } catch (const std::exception & exception) {
        reply.detail = std::string("motion SDK exception: ") + exception.what();
      } catch (...) {
        reply.detail = "unknown motion SDK exception";
      }
      result(std::move(reply), true);
      changed.notify_all();
    }
  }

  void stop_loop()
  {
    while (!shutdown) {
      std::uint64_t epoch;
      bool sender_quiescent;
      {
        std::unique_lock<std::mutex> lock(mutex);
        changed.wait(lock, [this]() {return shutdown || stop_job.has_value();});
        if (shutdown) {break;}
        epoch = *stop_job;
        stop_job.reset();
        stop_in_progress = true;
        sender_quiescent = !busy && !job.has_value();
      }
      TransportResult reply;
      reply.epoch = epoch;
      reply.stop = true;
      try {
        if (options.single_stop_when_quiescent && sender_quiescent) {
          // Intent was revoked before this lock snapshot. No older send can
          // race this stop, so a second stop on another session is redundant.
          const int code = send->robotMoveFastStop();
          reply.success = code == vendor::ErrnoSucc;
          reply.detail = "quiescent stop SDK result " + std::to_string(code);
        } else {
          const int first = stop->robotMoveFastStop();
          // A dispatch can pass its cancellation check just before the first
          // stop. Wait for that request to finish, then issue a final stop.
          // Never acknowledge stop while an older send could still start late.
          bool quiescent;
          {
            std::unique_lock<std::mutex> lock(mutex);
            quiescent = changed.wait_for(
              lock, std::chrono::seconds(2),
              [this]() {return !busy || shutdown;});
          }
          if (!quiescent || shutdown) {
            reply.detail = "stop unconfirmed: earlier SDK send has not quiesced";
          } else {
            // Quiescent sender session gives the final stop the same request
            // ordering as the earlier LineMove. Firmware queue-clearing semantics
            // still need the explicit hardware checkpoint; ACK is not physical stop.
            const int last = send->robotMoveFastStop();
            reply.success = first == vendor::ErrnoSucc && last == vendor::ErrnoSucc;
            reply.detail = "stop SDK results " + std::to_string(first) + "," + std::to_string(last);
          }
        }
      } catch (const std::exception & exception) {
        reply.detail = std::string("stop SDK exception: ") + exception.what();
      } catch (...) {
        reply.detail = "unknown stop SDK exception";
      }
      result(std::move(reply));
      {
        std::lock_guard<std::mutex> lock(mutex);
        stop_in_progress = false;
      }
      changed.notify_all();
    }
  }

  void read_loop()
  {
    while (!shutdown) {
      vendor::RobotDiagnosis raw{};
      const double query_started_at = steady_seconds();
      vendor::RobotState state = vendor::RobotStopped;
      int diagnosis_rc = -1, state_rc = -1;
      double diagnosis_time = 0.0, state_time = 0.0;
      try {
        diagnosis_rc = read->robotServiceGetRobotDiagnosisInfo(raw);
        diagnosis_time = steady_seconds();
      } catch (...) {
        // Mark unavailable, never substitute a healthy/stopped sample.
      }
      DiagnosisSample d;
      d.arm_power_status = raw.armPowerStatus;
      d.arm_canbus_status = raw.armCanbusStatus;
      d.soft_emergency = raw.softEmergency;
      d.remote_emergency = raw.remoteEmergency;
      d.robot_collision = raw.robotCollision;
      d.joint_collision_detect = raw.jointCollisionDetect;
      d.encoder_error_status = raw.encoderErrorStatus;
      d.encoder_lines_error = raw.encoderLinesError;
      d.joint_error_status = raw.jointErrorStatus;
      d.singularity_overspeed_alarm = raw.singularityOverSpeedAlarm;
      d.robot_current_alarm = raw.robotCurrentAlarm;
      d.tool_io_error = raw.toolIoError;
      std::string reason;
      const bool good = diagnosis_rc == vendor::ErrnoSucc && diagnosis_is_healthy(d, &reason);
      {
        std::lock_guard<std::mutex> lock(mutex);
        latest.diagnosis_ok = good;
        latest.diagnosis_result = diagnosis_rc;
        if (diagnosis_rc == vendor::ErrnoSucc) {latest.diagnosis_at = diagnosis_time;}
        latest.error = good ? std::string{} : "telemetry unavailable or unhealthy: " + reason;
        if (!good) {
          latest.last_telemetry_failure = latest.error + " diagnosis_rc=" +
            std::to_string(diagnosis_rc);
        }
      }
      // Publish diagnosis immediately. A slow subsequent state getter must
      // not hide an already completed, fresh diagnosis from the control tick.
      try {
        state_rc = read->robotServiceGetRobotCurrentState(state);
        state_time = steady_seconds();
      } catch (...) {}
      {
        std::lock_guard<std::mutex> lock(mutex);
        latest.state_result = state_rc;
        latest.query_duration_s = steady_seconds() - query_started_at;
        latest.feedback.stopped = state_rc == vendor::ErrnoSucc && state == vendor::RobotStopped;
        if (state_rc == vendor::ErrnoSucc) {latest.robot_state_at = state_time;}
        if (state_rc != vendor::ErrnoSucc) {
          latest.error = "robot state query failed: " + std::to_string(state_rc);
          latest.last_telemetry_failure = latest.error;
        }
      }
      std::unique_lock<std::mutex> lock(mutex);
      changed.wait_for(
        lock, std::chrono::duration<double>(options.state_poll_interval_s),
        [this]() {return shutdown.load();});
    }
  }
};

StepwiseTransport::StepwiseTransport()
: impl_(std::make_unique<Impl>()) {}
StepwiseTransport::~StepwiseTransport() = default;

bool StepwiseTransport::connect(const TransportOptions & options, std::string & error)
{
  if (impl_->send || options.host.empty() || options.username.empty() || options.password.empty() ||
    options.port < 1 || options.port > 65535 || !std::isfinite(options.speed_mps) ||
    options.speed_mps <= 0.0 || options.speed_mps > 0.200 ||
    !std::isfinite(options.acceleration_mps2) || options.acceleration_mps2 <= 0.0 ||
    options.acceleration_mps2 > 0.02 || !std::isfinite(options.maximum_joint_step_rad) ||
    !std::isfinite(options.fast_acceleration_mps2) || options.fast_acceleration_mps2 <= 0.0 ||
    options.fast_acceleration_mps2 > 0.5 ||
    !std::isfinite(options.follow_joint_speed_radps) || options.follow_joint_speed_radps <= 0.0 ||
    options.follow_joint_speed_radps > 1.0 ||
    !std::isfinite(options.follow_joint_acceleration_radps2) ||
    options.follow_joint_acceleration_radps2 <= 0.0 ||
    options.follow_joint_acceleration_radps2 > 5.0 ||
    options.maximum_joint_step_rad <= 0.0 || options.maximum_joint_step_rad > 0.1 ||
    !std::isfinite(options.maximum_step_m) || options.maximum_step_m <= 0.0 ||
    options.maximum_step_m > 0.005 || !std::isfinite(options.state_poll_interval_s) ||
    options.state_poll_interval_s < 0.02 || options.state_poll_interval_s > 1.0)
  {
    error = "invalid stepwise transport configuration or repeated connect";
    return false;
  }
  auto & self = *impl_;
  self.options = options;
  try {
    self.send = std::make_unique<ServiceInterface>();
    self.read = std::make_unique<ServiceInterface>();
    self.stop = std::make_unique<ServiceInterface>();
    const auto login = [&options](ServiceInterface & service) {
        return service.robotServiceLogin(
          options.host.c_str(), options.port,
          options.username.c_str(), options.password.c_str()) == vendor::ErrnoSucc;
      };
    if (!(self.send_connected = login(*self.send)) ||
      !(self.read_connected = login(*self.read)) || !(self.stop_connected = login(*self.stop)))
    {
      error = "stepwise SDK login failed; no automatic retry";
      return false;
    }
    bool real = false;
    vendor::RobotWorkMode mode = vendor::RobotModeSimulator;
    if (self.read->robotServiceGetIsRealRobotExist(real) != vendor::ErrnoSucc || !real ||
      self.read->robotServiceGetRobotWorkMode(mode) != vendor::ErrnoSucc ||
      mode != vendor::RobotModeReal)
    {
      error = "controller did not verify a real robot in real mode";
      return false;
    }
    // Initialize this SDK session's local move profile then set both limits.
    // No power, brake, TCP, safety IO, work-mode or firmware changes.
    if (self.send->robotServiceInitGlobalMoveProfile() != vendor::ErrnoSucc ||
      self.send->robotServiceSetGlobalMoveEndMaxLineAcc(options.acceleration_mps2) !=
      vendor::ErrnoSucc ||
      !self.apply_profile(options.speed_mps))
    {
      error = "stepwise move profile/limits rejected";
      return false;
    }
    if (self.read->robotServiceRegisterRealTimeRoadPointCallback(&Impl::waypoint, &self) !=
      vendor::ErrnoSucc ||
      self.read->robotServiceSetRealTimeRoadPointPush(true) != vendor::ErrnoSucc)
    {
      error = "waypoint push registration rejected";
      return false;
    }
    if (options.allow_follow_mode && options.fixed_follow_profile) {
      Impl::Job initial;
      initial.command.follow_joint_mode = true;
      vendor::wayPoint_S unused{};
      if (!self.apply_follow_profile(initial, unused)) {
        error = "fixed follow profile readback failed";
        return false;
      }
    }
    self.sender = std::thread([&self]() {self.send_loop();});
    self.stopper = std::thread([&self]() {self.stop_loop();});
    self.reader = std::thread([&self]() {self.read_loop();});
    return true;
  } catch (...) {
    error = "stepwise SDK connection/setup exception";
    return false;
  }
}

TransportSample StepwiseTransport::sample() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  auto value = impl_->latest;
  value.feedback.healthy = value.diagnosis_ok && value.state_result == vendor::ErrnoSucc &&
    impl_->pose_valid;
  return value;
}
bool StepwiseTransport::submit(const StepwiseCommand & command, const std::array<double, 6> & seed)
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (impl_->shutdown || impl_->busy || impl_->job || impl_->stop_job ||
    (command.follow_joint_mode && !impl_->options.allow_follow_mode) ||
    (command.hold_position && !command.follow_joint_mode) ||
    !std::isfinite(command.speed_mps) || command.speed_mps <= 0.0 ||
    command.speed_mps > impl_->options.speed_mps ||
    impl_->stop_in_progress || !impl_->sender.joinable() ||
    command.id <= impl_->last_submitted_id || command.epoch < impl_->allowed_epoch)
  {
    return false;
  }
  double squared_step = 0.0, quaternion_norm = 0.0;
  for (std::size_t index = 0; index < 3; ++index) {
    if (!std::isfinite(command.start[index]) || !std::isfinite(command.target[index])) {
      return false;
    }
    const double delta = command.target[index] - command.start[index];
    squared_step += delta * delta;
  }
  for (double value : command.orientation) {
    if (!std::isfinite(value)) {return false;}
    quaternion_norm += value * value;
  }
  for (double value : seed) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  if (!std::isfinite(squared_step) ||
    (squared_step <= 0.0 && !command.hold_position) ||
    (command.hold_position && squared_step > 1e-20) ||
    squared_step > impl_->options.maximum_step_m * impl_->options.maximum_step_m * (1.0 + 1e-9) ||
    !std::isfinite(quaternion_norm) || std::abs(quaternion_norm - 1.0) > 1e-3) {return false;}
  impl_->last_submitted_id = command.id;
  impl_->allowed_epoch = command.epoch;
  impl_->allowed = true;
  impl_->job = Impl::Job{command, seed};
  impl_->changed.notify_all();
  return true;
}
void StepwiseTransport::cancel_and_stop(std::uint64_t epoch)
{
  impl_->allowed = false;
  impl_->invalidate_follow_profile = true;
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->allowed = false;
  impl_->allowed_epoch = std::max(epoch, impl_->allowed_epoch.load());
  impl_->previous_dispatch_at = 0.0;
  impl_->job.reset();
  impl_->stop_job = impl_->allowed_epoch.load();
  impl_->changed.notify_all();
}
std::optional<TransportResult> StepwiseTransport::take_result()
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (impl_->results.empty()) {return std::nullopt;}
  auto value = std::move(impl_->results.front());
  impl_->results.pop_front();
  return value;
}
}  // namespace aubo_v4_driver
