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

  // These getters report this SDK session's profile, not measured physical speed.
  bool apply_profile(double speed)
  {
    if (send->robotServiceSetGlobalMoveEndMaxLineVelc(speed) != vendor::ErrnoSucc) {
      return false;
    }
    double velocity = NAN, acceleration = NAN;
    send->robotServiceGetGlobalMoveEndMaxLineVelc(velocity);
    send->robotServiceGetGlobalMoveEndMaxLineAcc(acceleration);
    if (!std::isfinite(velocity) || !std::isfinite(acceleration) ||
      std::abs(velocity - speed) > 1e-9 ||
      std::abs(acceleration - options.acceleration_mps2) > 1e-9) {return false;}
    std::lock_guard<std::mutex> lock(mutex);
    latest.sdk_speed_mps = velocity;
    latest.sdk_acceleration_mps2 = acceleration;
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

  void result(TransportResult value)
  {
    std::lock_guard<std::mutex> lock(mutex);
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
        const int ik = send->robotServiceRobotIk(
          current.seed.data(), position, orientation, target);
        bool valid = ik == vendor::ErrnoSucc;
        for (std::size_t i = 0; i < 6; ++i) {
          valid = valid && std::isfinite(target.jointpos[i]) &&
            std::abs(target.jointpos[i] - current.seed[i]) <= options.maximum_joint_step_rad;
        }
        if (!valid) {
          reply.detail = "stepwise IK failed or exceeded joint-step bound";
        } else if (!allowed || current.command.epoch != allowed_epoch || shutdown) {
          reply.detail = "stepwise command cancelled before dispatch";
        } else if (!apply_profile(current.command.speed_mps)) {
          reply.detail = "stepwise SDK speed profile readback failed";
        } else if (!allowed || current.command.epoch != allowed_epoch || shutdown) {
          reply.detail = "stepwise command cancelled during profile update";
        } else {
          // Absolute bounded endpoint: no speculative queued trajectory, no
          // automatic resend. 'false' does NOT promise zero request latency.
          {
            std::lock_guard<std::mutex> lock(mutex);
            ever_dispatched = true;
          }
          const int code = send->robotServiceLineMove(target, false);
          reply.success = code == vendor::ErrnoSucc;
          {
            std::lock_guard<std::mutex> lock(mutex);
            ++latest.motion_ack_count;
            latest.last_command_id = current.command.id;
            latest.requested_speed_mps = current.command.speed_mps;
          }
          reply.detail = "ordinary LineMove SDK result " + std::to_string(code);
        }
      } catch (const std::exception & exception) {
        reply.detail = std::string("motion SDK exception: ") + exception.what();
      } catch (...) {
        reply.detail = "unknown motion SDK exception";
      }
      result(std::move(reply));
      {
        std::lock_guard<std::mutex> lock(mutex);
        busy = false;
      }
      changed.notify_all();
    }
  }

  void stop_loop()
  {
    while (!shutdown) {
      std::uint64_t epoch;
      {
        std::unique_lock<std::mutex> lock(mutex);
        changed.wait(lock, [this]() {return shutdown || stop_job.has_value();});
        if (shutdown) {break;}
        epoch = *stop_job;
        stop_job.reset();
        stop_in_progress = true;
      }
      TransportResult reply;
      reply.epoch = epoch;
      reply.stop = true;
      try {
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
      vendor::RobotState state = vendor::RobotStopped;
      int diagnosis_rc = -1, state_rc = -1;
      double diagnosis_time = 0.0, state_time = 0.0;
      try {
        diagnosis_rc = read->robotServiceGetRobotDiagnosisInfo(raw);
        diagnosis_time = steady_seconds();
        state_rc = read->robotServiceGetRobotCurrentState(state);
        state_time = steady_seconds();
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
      const bool good = diagnosis_rc == vendor::ErrnoSucc &&
        state_rc == vendor::ErrnoSucc && diagnosis_is_healthy(d, &reason);
      {
        std::lock_guard<std::mutex> lock(mutex);
        latest.diagnosis_ok = good;
        latest.feedback.healthy = good;
        latest.feedback.stopped = state_rc == vendor::ErrnoSucc && state == vendor::RobotStopped;
        if (diagnosis_rc == vendor::ErrnoSucc) {latest.diagnosis_at = diagnosis_time;}
        if (state_rc == vendor::ErrnoSucc) {latest.robot_state_at = state_time;}
        latest.error = good ? std::string{} : "telemetry unavailable or unhealthy: " + reason;
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
    options.speed_mps <= 0.0 || options.speed_mps > 0.005 ||
    !std::isfinite(options.acceleration_mps2) || options.acceleration_mps2 <= 0.0 ||
    options.acceleration_mps2 > 0.02 || !std::isfinite(options.maximum_joint_step_rad) ||
    options.maximum_joint_step_rad <= 0.0 || options.maximum_joint_step_rad > 0.1 ||
    !std::isfinite(options.maximum_step_m) || options.maximum_step_m <= 0.0 ||
    options.maximum_step_m > 0.001 || !std::isfinite(options.state_poll_interval_s) ||
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
  value.feedback.healthy = value.diagnosis_ok && impl_->pose_valid;
  return value;
}
bool StepwiseTransport::submit(const StepwiseCommand & command, const std::array<double, 6> & seed)
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (impl_->shutdown || impl_->busy || impl_->job || impl_->stop_job ||
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
  if (!std::isfinite(squared_step) || squared_step <= 0.0 ||
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
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->allowed = false;
  impl_->allowed_epoch = std::max(epoch, impl_->allowed_epoch.load());
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
