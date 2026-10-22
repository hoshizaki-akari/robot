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
#include "aubo_v4_driver/aubo_drag_reference.hpp"
#include "aubo_v4_driver/tcp_can_drag.hpp"

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
  // Legacy follow/line RPCs serialize normal queries. Experimental TCP2CAN
  // keeps telemetry on the separate read session, away from its buffer feeder.
  // Real-time callbacks and urgent stop remain separate in both modes.
  std::mutex request_mutex;
  std::atomic<bool> sender_waiting{false};
  std::condition_variable changed;
  std::thread sender, reader, stopper;
  std::atomic<bool> shutdown{false}, allowed{false};
  std::atomic<std::uint64_t> allowed_epoch{0};
  bool busy{false}, stop_in_progress{false}, pose_valid{false};
  bool continuation_busy{false};
  bool state_valid{false};
  std::atomic<bool> ever_dispatched{false};
  std::optional<Job> job;
  std::optional<Job> last_stream_job;
  std::optional<std::uint64_t> stop_job;
  std::uint64_t last_submitted_id{0};
  std::deque<TransportResult> results;
  TransportSample latest;
  double previous_dispatch_at{0.0};
  std::array<double, 6> filtered_joint_velocity{};
  CartesianVelocityWindow drag_velocity_window;
  struct DragIntent
  {
    std::array<double, 3> velocity;
    std::uint64_t epoch, generation;
    double received_at;
  };
  std::optional<DragIntent> drag_intent;
  std::uint64_t drag_intent_generation{0};
  vendor::JointVelcAccParam cached_joint_velocity{}, cached_joint_acceleration{};
  bool joint_profile_cached{false};
  std::atomic<bool> invalidate_follow_profile{true};
  std::unique_ptr<TcpCanDragSession> tcp_can;
  TcpCanPacing tcp_can_pacing;  // Accessed only by the serialized sender/stop path.
  bool tcp_can_entry_buffer_checked{false};

  TcpCanIo tcp_can_io(bool confirmed_buffer = false)
  {
    TcpCanIo io;
    io.enter = [this]() {return send->robotServiceEnterTcp2CanbusMode();};
    io.leave = [this]() {return send->robotServiceLeaveTcp2CanbusMode();};
    io.queued_points = [this, confirmed_buffer](unsigned & points) {
        if (!confirmed_buffer && tcp_can_entry_buffer_checked) {
          points = tcp_can_pacing.queued(steady_seconds());
          if (points == 0) {tcp_can_pacing.reset();}
          return true;
        }
        vendor::RobotDiagnosis diagnosis{};
        const double started = steady_seconds();
        const int code = send->robotServiceGetRobotDiagnosisInfo(diagnosis);
        {
          std::lock_guard<std::mutex> lock(mutex);
          latest.tcp_can_buffer_query_duration_s = steady_seconds() - started;
        }
        if (code != 0 ||
          diagnosis.macTargetPosDataSize % 6 != 0) {return false;}
        points = static_cast<unsigned>(diagnosis.macTargetPosDataSize / 6);
        if (!confirmed_buffer && points == 0) {tcp_can_entry_buffer_checked = true;}
        return true;
      };
    io.forward = [this](const std::array<double, 6> & joints, StreamPose & pose) {
        vendor::wayPoint_S point{};
        if (send->robotServiceRobotFk(joints.data(), 6, point) != 0) {return false;}
        pose.joints = joints;
        pose.position =
        {point.cartPos.position.x, point.cartPos.position.y, point.cartPos.position.z};
        pose.orientation =
        {point.orientation.x, point.orientation.y, point.orientation.z, point.orientation.w};
        return true;
      };
    io.inverse = [this, forward = io.forward](const StreamPose & request, StreamPose & solved) {
        vendor::Pos position{};
        position.x = request.position[0]; position.y = request.position[1];
        position.z = request.position[2];
        vendor::Ori orientation{};
        orientation.x = request.orientation[0]; orientation.y = request.orientation[1];
        orientation.z = request.orientation[2]; orientation.w = request.orientation[3];
        vendor::wayPoint_S target{};
        if (send->robotServiceRobotIk(request.joints.data(), position, orientation, target) != 0) {
          return false;
        }
        std::array<double, 6> joints{};
        std::copy(target.jointpos, target.jointpos + 6, joints.begin());
        return forward(joints, solved);
      };
    io.send = [this](const std::vector<std::array<double, 6>> & points) {
        std::vector<vendor::wayPoint_S> batch(points.size());
        for (std::size_t i = 0; i < points.size(); ++i) {
          std::copy(points[i].begin(), points[i].end(), batch[i].jointpos);
        }
        const double started = steady_seconds();
        if (tcp_can_pacing.initialized() && tcp_can_pacing.queued(started) == 0) {
          return -7101;  // Planning missed the deadline: never send a late catch-up batch.
        }
        const int code = send->robotServiceSetRobotPosData2Canbus(batch);
        const double finished = steady_seconds();
        const bool timely = code != 0 || tcp_can_pacing.accept(
          static_cast<unsigned>(points.size()), started, finished);
        std::lock_guard<std::mutex> lock(mutex);
        latest.tcp_can_send_duration_s = finished - started;
        latest.motion_call_duration_s = latest.tcp_can_send_duration_s;
        latest.maximum_motion_call_duration_s = std::max(
          latest.maximum_motion_call_duration_s,
          latest.motion_call_duration_s);
        latest.dispatch_interval_s = previous_dispatch_at >
          0 ? started - previous_dispatch_at : 0.0;
        latest.maximum_dispatch_interval_s = std::max(
          latest.maximum_dispatch_interval_s,
          latest.dispatch_interval_s);
        previous_dispatch_at = started;
        // An accepted batch with an expired deadline is still ambiguous motion,
        // not a retryable failure. The existing stop/leave path owns recovery.
        return timely ? code : -7102;
      };
    return io;
  }

  TransportResult send_tcp_can_drag(const Job & current)
  {
    TransportResult reply;
    reply.id = current.command.id; reply.epoch = current.command.epoch;
    StreamPose measured;
    std::array<double, 3> velocity{};
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (latest.tcp_can_fault) {
        reply.detail = latest.tcp_can_status;
        return reply;
      }
      measured.joints = latest.joints;
      measured.position = latest.feedback.position;
      measured.orientation = latest.feedback.orientation;
      if (!tcp_can->active() && (!latest.feedback.stopped || latest.feedback.speed_mps > 0.0002)) {
        reply.detail = "TCP2CAN entry requires measured standstill"; return reply;
      }
      if (drag_intent) {velocity = drag_intent->velocity;}
    }
    const auto authorized = [this, epoch = current.command.epoch]() {
        std::lock_guard<std::mutex> lock(mutex);
        const double time = steady_seconds();
        return allowed && !shutdown && allowed_epoch == epoch && drag_intent &&
               drag_intent->epoch == epoch && time >= drag_intent->received_at &&
               time - drag_intent->received_at <= options.drag_intent_timeout_s &&
               time - latest.feedback.received_at <= options.feedback_timeout_s &&
               pose_valid && latest.diagnosis_ok && state_valid && !latest.tcp_can_fault &&
               (!options.adaptive_follow_ik || latest.kinematics_verified);
      };
    const double started = steady_seconds();
    // Mode entry already changes controller ownership; even a rejected first
    // batch must run the existing stop/leave path during teardown.
    ever_dispatched = true;
    const auto accepted = tcp_can->update(
      measured, velocity, current.command.orientation, current.command.epoch,
      started, authorized, tcp_can_io());
    reply.success = accepted.success;
    reply.detail = accepted.detail;
    {
      std::lock_guard<std::mutex> lock(mutex);
      latest.tcp_can_active = tcp_can->active();
      if (!latest.tcp_can_fault) {latest.tcp_can_status = accepted.detail;}
      latest.tcp_can_fault = latest.tcp_can_fault || tcp_can->faulted();
      latest.tcp_can_batches = tcp_can->batches();
      latest.tcp_can_points = tcp_can->points();
      latest.tcp_can_queued_points = accepted.queued_points;
      latest.last_command_id = current.command.id;
      latest.requested_speed_mps = std::hypot(velocity[0], velocity[1], velocity[2]);
      latest.tcp_can_cycle_duration_s = steady_seconds() - started;
      if (accepted.points_sent > 0) {++latest.motion_ack_count;}
    }
    return reply;
  }

  bool leave_tcp_can_after_stop()
  {
    if (!tcp_can) {return true;}
    const bool left = tcp_can->leave_after_stop(tcp_can_io(true));
    if (left) {
      tcp_can_pacing.reset();
      tcp_can_entry_buffer_checked = false;
    }
    std::lock_guard<std::mutex> lock(mutex);
    latest.tcp_can_active = tcp_can->active();
    latest.tcp_can_fault = tcp_can->faulted();
    latest.tcp_can_status = left ? "TCP2CAN left after stop" : "TCP2CAN leave rejected";
    return left;
  }

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
    if (current.command.hold_position) {
      if (current.command.assisted_drag && options.drag_release_prediction_s > 0.0) {
        // Arrival-ahead blending is useful in motion, not for the final hold.
        // Reinstall the moving profile only when deliberate input resumes.
        joint_profile_cached = false;
        return send->robotServiceSetNoArrivalAhead() == 0;
      }
      return joint_profile_cached;
    }
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

  bool prepare_cartesian_drag(
    Job & current, vendor::Pos & position, vendor::Ori & orientation,
    std::uint64_t & generation, std::uint64_t & pose_sequence)
  {
    std::lock_guard<std::mutex> lock(mutex);
    const double time = steady_seconds();
    if (!pose_valid || latest.feedback.sequence == 0 ||
      time - latest.feedback.received_at > options.feedback_timeout_s) {return false;}
    std::array<double, 3> velocity{};
    generation = drag_intent ? drag_intent->generation : 0;
    pose_sequence = latest.feedback.sequence;
    latest.drag_intent_age_s = drag_intent ? time - drag_intent->received_at : -1.0;
    if (drag_intent && drag_intent->epoch == current.command.epoch &&
      latest.drag_intent_age_s >= 0.0 && latest.drag_intent_age_s <= options.drag_intent_timeout_s)
    {
      velocity = drag_intent->velocity;
    }
    const double requested_speed = std::hypot(velocity[0], velocity[1], velocity[2]);
    current.command.hold_position = current.command.hold_position || requested_speed <= 1e-12;
    double horizon = current.command.drag_reference_horizon_s;
    if (current.command.hold_position) {
      // One Cartesian endpoint along window-measured motion. Six independent
      // joint predictions need not preserve a translational release path.
      velocity = drag_velocity_window.velocity();
      if (time - latest.feedback.received_at > 0.1) {velocity = {};}
      const double speed = std::hypot(velocity[0], velocity[1], velocity[2]);
      horizon = speed <= 0.0002 ? 0.0 : std::min(
        options.drag_release_prediction_s, speed / (2.0 * options.fast_acceleration_mps2));
      orientation.x = latest.feedback.orientation[0];
      orientation.y = latest.feedback.orientation[1];
      orientation.z = latest.feedback.orientation[2];
      orientation.w = latest.feedback.orientation[3];
    } else if (requested_speed > options.speed_mps) {
      for (double & component : velocity) {component *= options.speed_mps / requested_speed;}
    }
    const auto offset = aubo_drag_offset(velocity, horizon, options.maximum_drag_step_m);
    current.seed = latest.joints;
    current.command.start = latest.feedback.position;
    current.command.speed_mps = current.command.hold_position ? options.speed_mps :
      std::min(requested_speed, options.speed_mps);
    position.x = current.command.start[0] + offset[0];
    position.y = current.command.start[1] + offset[1];
    position.z = current.command.start[2] + offset[2];
    current.command.target = {position.x, position.y, position.z};
    latest.drag_release_predicted_joint_step_rad = 0.0;
    ++latest.drag_reference_reanchors;
    return true;
  }

  bool drag_was_superseded(
    std::uint64_t generation, bool holding, const std::array<double, 3> & target)
  {
    std::lock_guard<std::mutex> lock(mutex);
    const bool released = drag_intent &&
      std::hypot(
      drag_intent->velocity[0], drag_intent->velocity[1],
      drag_intent->velocity[2]) <= 1e-12;
    bool changed = drag_intent && !(holding && released) &&
      (drag_intent->generation != generation ||
      (!holding && steady_seconds() - drag_intent->received_at > options.drag_intent_timeout_s));
    if (!holding && drag_intent && !released) {
      double ahead = 0.0;
      for (unsigned i = 0; i < 3; ++i) {
        ahead += (target[i] - latest.feedback.position[i]) * drag_intent->velocity[i];
      }
      // Even an unchanged intent can be obsolete after a blocked profile call:
      // never ask a moving handle to chase an endpoint it has already passed.
      changed = changed || ahead <= 0.0;
    }
    if (changed) {++latest.drag_superseded_commands;}
    return changed;
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
      try {
        if (send->robotMoveFastStop() == 0) {leave_tcp_can_after_stop();}
      } catch (...) {}
    }
    if (read_connected) {
      read->robotServiceRegisterRobotEventInfoCallback(nullptr, nullptr);
      read->robotServiceRegisterRealTimeRoadPointCallback(nullptr, nullptr);
      read->robotServiceLogout();
    }
    if (send_connected) {
      send->robotServiceRegisterRobotEventInfoCallback(nullptr, nullptr);
      send->robotServiceLogout();
    }
    if (stop_connected) {
      stop->robotServiceRegisterRobotEventInfoCallback(nullptr, nullptr);
      stop->robotServiceLogout();
    }
  }

  static void robot_event(const vendor::RobotEventInfo * event, void * context)
  {
    if (!event || !context) {return;}
    auto & self = *static_cast<Impl *>(context);
    std::lock_guard<std::mutex> lock(self.mutex);
    ++self.latest.controller_event_sequence;
    self.latest.controller_event_type = static_cast<int>(event->eventType);
    self.latest.controller_event_code = event->eventCode;
    self.latest.controller_event_content = event->eventContent.substr(0, 512);
    self.latest.controller_event_at = steady_seconds();
    const int type = self.latest.controller_event_type;
    if (type > 1000 && type < 1500 && type != 1005 && type != 1300) {
      ++self.latest.controller_error_sequence;
      self.latest.controller_error_type = type;
      self.latest.controller_error_content = self.latest.controller_event_content;
    }
    self.changed.notify_all();
  }

  // A missing stop response is not evidence that the arm is still moving.
  // Verify a new healthy diagnosis/state plus three distinct quiet pose frames,
  // all AFTER the last SDK stop call, while no sender can submit another move.
  bool measured_stop_after(double fence)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(700);
    std::unique_lock<std::mutex> lock(mutex);
    std::uint64_t sequence = latest.feedback.sequence;
    unsigned frames = 0;
    while (!shutdown && std::chrono::steady_clock::now() < deadline) {
      if (latest.feedback.sequence != sequence) {
        sequence = latest.feedback.sequence;
        const double time = steady_seconds();
        const bool stopped = diagnosis_is_healthy_sample(time, fence);
        frames = stopped ? frames + 1 : 0;
        if (frames >= 3) {return true;}
      }
      changed.wait_for(lock, std::chrono::milliseconds(5));
    }
    return false;
  }

  bool diagnosis_is_healthy_sample(double time, double fence) const
  {
    return !busy && !job && !allowed && diagnosis_ok_for_stop(time, fence) &&
           latest.feedback.stopped && latest.feedback.speed_mps <= 0.0002 &&
           latest.joint_speed_norm_radps <= 0.02;
  }

  bool diagnosis_ok_for_stop(double time, double fence) const
  {
    return latest.diagnosis_ok && state_valid && pose_valid &&
           latest.diagnosis_at > fence && latest.robot_state_at > fence &&
           latest.feedback.received_at > fence && time - latest.diagnosis_at <= 0.5 &&
           time - latest.robot_state_at <= 0.5 && time - latest.feedback.received_at <= 0.5;
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
    self.drag_velocity_window.push(pose.position, time);
    double speed = 0.0;
    double joint_speed = 0.0;
    const double dt = time - self.latest.feedback.received_at;
    self.latest.waypoint_interval_s = self.latest.feedback.sequence > 0 ? dt : 0.0;
    if (self.latest.feedback.sequence > 0 && dt > 0.0 && dt <= 1.0) {
      double squared = 0.0;
      for (std::size_t i = 0; i < 3; ++i) {
        const double delta = pose.position[i] - self.latest.feedback.position[i];
        squared += delta * delta;
      }
      speed = std::sqrt(squared) / dt;
      for (std::size_t i = 0; i < 6; ++i) {
        const double measured = (pose.joints[i] - self.latest.joints[i]) / dt;
        joint_speed = std::hypot(joint_speed, measured);
        const double bounded = std::clamp(
          measured, -self.options.follow_joint_speed_radps, self.options.follow_joint_speed_radps);
        const double alpha = dt / (0.04 + dt);
        self.filtered_joint_velocity[i] += alpha * (bounded - self.filtered_joint_velocity[i]);
      }
    } else {
      self.filtered_joint_velocity = {};
    }
    self.latest.feedback.position = pose.position;
    self.latest.feedback.orientation = pose.orientation;
    self.latest.feedback.received_at = time;
    self.latest.feedback.speed_mps = speed;
    ++self.latest.feedback.sequence;
    self.latest.joints = pose.joints;
    self.latest.joint_speed_norm_radps = joint_speed;
    self.changed.notify_all();
  }

  void result(TransportResult value, bool sender_finished = false)
  {
    std::lock_guard<std::mutex> lock(mutex);
    // Publishing an ACK also releases the only sender slot atomically. A
    // tracking replacement may arrive immediately after the consumer sees it.
    if (sender_finished) {busy = false; continuation_busy = false;}
    // Likewise, consuming a stop ACK must never leave a stale stopping flag
    // that rejects the next otherwise valid session command.
    if (value.stop) {stop_in_progress = false;}
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
      bool continuation = false;
      {
        std::unique_lock<std::mutex> lock(mutex);
        if (tcp_can) {
          // Keep producing bounded timed deceleration points after the single
          // scheduler hold command; no unacknowledged fake scheduler jobs.
          changed.wait_for(
            lock, std::chrono::milliseconds(10), [this]() {
              return shutdown || job.has_value();
            });
        } else {
          changed.wait(lock, [this]() {return shutdown || job.has_value();});
        }
        if (shutdown) {break;}
        if (job) {
          current = *job;
          job.reset();
          if (tcp_can && current.command.assisted_drag) {last_stream_job = current;}
        } else if (tcp_can && latest.tcp_can_active && !latest.tcp_can_fault && allowed &&
          last_stream_job)
        {
          current = *last_stream_job;
          continuation = true;
        } else {continue;}
        busy = true;
        continuation_busy = continuation;
      }
      TransportResult reply;
      reply.id = current.command.id;
      reply.epoch = current.command.epoch;
      try {
        sender_waiting = true;
        {
          std::unique_lock<std::mutex> lock(mutex);
          // A saturated motion stream must not starve diagnosis/state RPCs.
          // Yield briefly when they are due, without overlapping SDK calls.
          const auto current_telemetry = [this]() {
              const double time = steady_seconds();
              return time - latest.diagnosis_at < 0.25 && time - latest.robot_state_at < 0.25;
            };
          if (!current_telemetry() && !(tcp_can && latest.tcp_can_active)) {
            changed.wait_for(lock, std::chrono::milliseconds(100), current_telemetry);
          }
        }
        std::unique_lock<std::mutex> request_lock(request_mutex);
        sender_waiting = false;
        if (tcp_can && current.command.assisted_drag) {
          auto streamed = send_tcp_can_drag(current);
          if (continuation) {
            std::lock_guard<std::mutex> lock(mutex);
            busy = false;
            continuation_busy = false;
            if (!streamed.success) {
              latest.tcp_can_fault = true;
              latest.error = streamed.detail;
            }
          } else {result(std::move(streamed), true);}
          changed.notify_all();
          continue;
        }
        if (tcp_can && tcp_can->active()) {
          reply.detail = "stop the TCP2CAN drag session before changing motion modes";
          result(std::move(reply), true);
          continue;
        }
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
        std::uint64_t drag_generation = 0;
        std::uint64_t drag_pose_sequence = 0;
        if (current.command.cartesian_drag_reference) {
          ik = prepare_cartesian_drag(
            current, position, orientation, drag_generation, drag_pose_sequence) ?
            send->robotServiceRobotIk(
            current.seed.data(), position, orientation, target) : -1;
        } else if (current.command.hold_position) {
          // Hold exactly the sampled joints. Re-solving a Cartesian hold pose
          // can introduce a branch/roundoff correction on an already moving arm.
          std::lock_guard<std::mutex> lock(mutex);
          std::copy(latest.joints.begin(), latest.joints.end(), target.jointpos);
          latest.drag_release_predicted_joint_step_rad = 0.0;
          if (current.command.assisted_drag && options.drag_release_prediction_s > 0.0 &&
            latest.joint_speed_norm_radps > 0.02 &&
            steady_seconds() - latest.feedback.received_at <= 0.1)
          {
            // A moving handle must decelerate ahead, not chase its sampled
            // position backwards after the network/servo delay. This is a
            // single bounded terminal target, not an integrating drift term.
            for (unsigned i = 0; i < 6; ++i) {
              const double velocity = filtered_joint_velocity[i];
              const double time = std::min(
                options.drag_release_prediction_s,
                std::abs(velocity) / (2.0 * options.follow_joint_acceleration_radps2));
              const double room = std::max(
                0.0, options.maximum_joint_step_rad -
                std::abs(latest.joints[i] - current.seed[i]));
              const double bound = std::min(room, 0.2 * options.maximum_joint_step_rad);
              const double delta = std::clamp(velocity * time, -bound, bound);
              target.jointpos[i] += delta;
              latest.drag_release_predicted_joint_step_rad = std::max(
                latest.drag_release_predicted_joint_step_rad, std::abs(delta));
            }
          }
        } else {
          ik = send->robotServiceRobotIk(current.seed.data(), position, orientation, target);
        }
        if (current.command.cartesian_drag_reference && ik == vendor::ErrnoSucc) {
          bool changed_intent = false;
          {
            std::lock_guard<std::mutex> lock(mutex);
            changed_intent = latest.feedback.sequence != drag_pose_sequence ||
              (drag_intent && (drag_intent->generation != drag_generation ||
              steady_seconds() - drag_intent->received_at > options.drag_intent_timeout_s));
            if (changed_intent) {++latest.drag_intent_refreshes;}
          }
          // A blocked IK/RPC must not release an obsolete direction later.
          // Refresh once from the newest intent/pose; never queue catch-up moves.
          if (changed_intent) {
            ik = prepare_cartesian_drag(
              current, position, orientation, drag_generation, drag_pose_sequence) ?
              send->robotServiceRobotIk(current.seed.data(), position, orientation, target) : -1;
          }
        }
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
          double accepted_scale = 1.0;
          int attempts = 0;
          int fk_result = -1;
          if (ik_failure.empty() && current.command.follow_joint_mode &&
            options.adaptive_follow_ik && !current.command.hold_position &&
            maximum_joint_delta > options.maximum_joint_step_rad)
          {
            vendor::wayPoint_S start_pose{};
            fk_result = send->robotServiceRobotFk(current.seed.data(), 6, start_pose);
            PoseSample anchor;
            anchor.position = {start_pose.cartPos.position.x, start_pose.cartPos.position.y,
              start_pose.cartPos.position.z};
            anchor.orientation = {start_pose.orientation.x, start_pose.orientation.y,
              start_pose.orientation.z, start_pose.orientation.w};
            anchor.joints = current.seed;
            if (fk_result != vendor::ErrnoSucc || !valid_pose_sample(anchor, nullptr)) {
              ik_failure = "adaptive IK anchor FK rejected";
            } else {
              double dot = anchor.orientation[0] * orientation.x + anchor.orientation[1] *
                orientation.y +
                anchor.orientation[2] * orientation.z + anchor.orientation[3] * orientation.w;
              const double sign = dot < 0.0 ? -1.0 : 1.0;
              const std::array<double, 4> goal{orientation.x * sign, orientation.y * sign,
                orientation.z * sign, orientation.w * sign};
              for (unsigned retry = 0;
                retry < 8 && maximum_joint_delta > options.maximum_joint_step_rad; ++retry)
              {
                if (!allowed || current.command.epoch != allowed_epoch || shutdown) {break;}
                accepted_scale *= 0.5;
                ++attempts;
                vendor::Pos shorter{};
                shorter.x = anchor.position[0] + accepted_scale * (position.x - anchor.position[0]);
                shorter.y = anchor.position[1] + accepted_scale * (position.y - anchor.position[1]);
                shorter.z = anchor.position[2] + accepted_scale * (position.z - anchor.position[2]);
                std::array<double, 4> blended{};
                double squared = 0.0;
                for (unsigned i = 0; i < 4; ++i) {
                  blended[i] = anchor.orientation[i] + accepted_scale *
                    (goal[i] - anchor.orientation[i]);
                  squared += blended[i] * blended[i];
                }
                vendor::Ori shorter_orientation{};
                const double length = std::sqrt(squared);
                shorter_orientation.x = blended[0] / length;
                shorter_orientation.y = blended[1] / length;
                shorter_orientation.z = blended[2] / length;
                shorter_orientation.w = blended[3] / length;
                ik = send->robotServiceRobotIk(
                  current.seed.data(), shorter, shorter_orientation, target);
                if (ik != vendor::ErrnoSucc) {ik_failure = "adaptive IK solve rejected"; break;}
                maximum_joint_delta = 0.0;
                for (unsigned i = 0; i < 6; ++i) {
                  if (!std::isfinite(target.jointpos[i])) {
                    ik_failure = "adaptive IK returned nonfinite joints"; break;
                  }
                  const double delta = std::abs(target.jointpos[i] - current.seed[i]);
                  if (delta > maximum_joint_delta) {
                    maximum_joint_delta = delta; maximum_joint_index = i;
                  }
                }
                if (!ik_failure.empty()) {break;}
              }
            }
          }
          if (ik_failure.empty() && current.command.follow_joint_mode &&
            options.subdivide_follow_joint_steps && !options.adaptive_follow_ik &&
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
            latest.accepted_ik_scale = accepted_scale;
            latest.adaptive_ik_attempts = attempts;
            latest.fk_result = fk_result;
            double squared_step = 0.0;
            for (unsigned i = 0; i < 3; ++i) {
              const double delta = current.command.target[i] - current.command.start[i];
              squared_step += delta * delta;
            }
            latest.accepted_cartesian_step_m = std::sqrt(squared_step) * accepted_scale;
            if (attempts > 0) {++latest.follow_joint_subdivisions;}
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
          latest.ik_duration_s = steady_seconds() - ik_started;
          latest.maximum_ik_duration_s =
            std::max(latest.maximum_ik_duration_s, latest.ik_duration_s);
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
        } else if (!(current.command.hold_position && options.brake_on_hold) &&
          !apply_follow_profile(current, target))
        {
          reply.detail = "motion mode/joint profile rejected";
        } else if (!allowed || current.command.epoch != allowed_epoch || shutdown) {
          reply.detail = "stepwise command cancelled during profile update";
        } else if (current.command.cartesian_drag_reference &&
          drag_was_superseded(
            drag_generation, current.command.hold_position, current.command.target))
        {
          reply.superseded = true;
          reply.detail = "obsolete drag intent discarded before SDK dispatch";
        } else {
          // Absolute bounded endpoint: no speculative queued trajectory, no
          // automatic resend. 'false' does NOT promise zero request latency.
          {
            std::lock_guard<std::mutex> lock(mutex);
            ever_dispatched = true;
          }
          const double dispatch_at = steady_seconds();
          const bool braking_hold = current.command.hold_position && options.brake_on_hold;
          int code =
            braking_hold ? send->robotMoveFastStop() : current.command.follow_joint_mode ?
            send->robotServiceFollowModeJointMove(target.jointpos) :
            send->robotServiceLineMove(target, false);
          const int first_code = code;
          // RequestTimeout does not say whether the controller applied the stop.
          // Repeating this idempotent brake once is safe; never resend a move,
          // or turn an unconfirmed stop into a successful acknowledgement.
          const bool retry_hold = braking_hold && code == vendor::ErrCode_RequestTimeout &&
            allowed && current.command.epoch == allowed_epoch && !shutdown;
          if (retry_hold) {code = send->robotMoveFastStop();}
          if (braking_hold) {invalidate_follow_profile = true;}
          const double motion_duration = steady_seconds() - dispatch_at;
          reply.success = code == vendor::ErrnoSucc;
          reply.recoverable = !braking_hold && current.command.follow_joint_mode &&
            (code == vendor::ErrCode_RequestTimeout ||
            code == vendor::ErrCode_ResponseReturnError ||
            code == vendor::ErrCodeWaitRobotStopped);
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
            latest.requested_speed_mps = braking_hold ? 0.0 : current.command.speed_mps;
            latest.follow_joint_mode = current.command.follow_joint_mode;
          }
          reply.detail = std::string(
            braking_hold ? "hold brake SDK result " :
            current.command.follow_joint_mode ?
            "FollowModeJointMove SDK result " : "ordinary LineMove SDK result ") + std::to_string(
            code);
          if (retry_hold) {
            reply.detail += " after initial timeout " + std::to_string(first_code);
          }
        }
      } catch (const std::exception & exception) {
        reply.detail = std::string("motion SDK exception: ") + exception.what();
      } catch (...) {
        reply.detail = "unknown motion SDK exception";
      }
      if (continuation) {
        std::lock_guard<std::mutex> lock(mutex);
        busy = false;
        continuation_busy = false;
        latest.tcp_can_fault = true;
        latest.error = reply.detail;
        latest.tcp_can_status = reply.detail;
        changed.notify_all();
        continue;
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
          int code;
          {
            std::lock_guard<std::mutex> request_lock(request_mutex);
            code = send->robotMoveFastStop();
          }
          const double stop_finished = steady_seconds();
          reply.success = code == vendor::ErrnoSucc;
          reply.detail = "quiescent stop SDK result " + std::to_string(code);
          if (code == vendor::ErrCode_RequestTimeout && measured_stop_after(stop_finished)) {
            reply.success = true;
            reply.detail += "; stop verified by fresh post-request telemetry";
          }
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
            int last;
            {
              std::lock_guard<std::mutex> request_lock(request_mutex);
              last = send->robotMoveFastStop();
            }
            const double stop_finished = steady_seconds();
            // The first request can time out without telling us whether it was
            // applied. After every earlier send has quiesced, the final ACK is
            // authoritative. Physical idle is still confirmed by the scheduler.
            reply.success = last == vendor::ErrnoSucc &&
              (first == vendor::ErrnoSucc || first == vendor::ErrCode_RequestTimeout);
            reply.detail = "stop SDK results " + std::to_string(first) + "," + std::to_string(last);
            if (last == vendor::ErrCode_RequestTimeout &&
              (first == vendor::ErrnoSucc || first == vendor::ErrCode_RequestTimeout) &&
              measured_stop_after(stop_finished))
            {
              reply.success = true;
              reply.detail += "; stop verified by fresh post-request telemetry";
            }
          }
        }
        if (reply.success) {
          std::lock_guard<std::mutex> request_lock(request_mutex);
          reply.success = leave_tcp_can_after_stop();
          if (!reply.success) {reply.detail += "; TCP2CAN mode/buffer release unconfirmed";}
        }
      } catch (const std::exception & exception) {
        reply.detail = std::string("stop SDK exception: ") + exception.what();
      } catch (...) {
        reply.detail = "unknown stop SDK exception";
      }
      result(std::move(reply));
      changed.notify_all();
    }
  }

  void read_loop()
  {
    while (!shutdown) {
      // Yield normal queries to a queued motion; never block the ROS executor.
      bool streaming;
      std::uint64_t streaming_epoch;
      {
        std::lock_guard<std::mutex> lock(mutex);
        streaming = latest.tcp_can_active;
        streaming_epoch = allowed_epoch;
      }
      std::unique_lock<std::mutex> request_lock(request_mutex, std::defer_lock);
      if (!streaming) {request_lock.try_lock();}
      bool telemetry_due;
      {
        std::lock_guard<std::mutex> lock(mutex);
        const double time = steady_seconds();
        telemetry_due = time - latest.diagnosis_at >= 0.25 ||
          time - latest.robot_state_at >= 0.25;
      }
      if (!streaming && (!request_lock.owns_lock() || (sender_waiting && !telemetry_due))) {
        if (request_lock.owns_lock()) {request_lock.unlock();}
        std::unique_lock<std::mutex> lock(mutex);
        changed.wait_for(lock, std::chrono::milliseconds(5));
        continue;
      }
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
        // A failed read is not a new diagnosis. Retain only the last successful
        // sample on a request timeout; its original age still expires in node.
        // Explicit alarm/error replies invalidate health immediately.
        if (diagnosis_rc != vendor::ErrCode_RequestTimeout) {latest.diagnosis_ok = good;}
        latest.diagnosis_result = diagnosis_rc;
        if (diagnosis_rc == vendor::ErrCode_RequestTimeout) {++latest.diagnosis_read_timeouts;}
        if (diagnosis_rc == vendor::ErrnoSucc) {latest.diagnosis_at = diagnosis_time;}
        if (streaming && latest.tcp_can_active && allowed_epoch == streaming_epoch &&
          diagnosis_rc == vendor::ErrnoSucc)
        {
          latest.tcp_can_observed_buffer_values = raw.macTargetPosDataSize;
          // Real controller occupancy remains independently supervised. This
          // value is not replaced by the sender's time-based pacing estimate.
          if (raw.macTargetPosDataSize > 72 || raw.macTargetPosDataSize % 6 != 0) {
            latest.tcp_can_fault = true;
            latest.tcp_can_status = "TCP2CAN observed buffer inconsistent with bounded feeder";
          }
        }
        latest.error = good ? std::string{} : "telemetry unavailable or unhealthy: " + reason;
        if (!good) {
          latest.last_telemetry_failure = latest.error + " diagnosis_rc=" +
            std::to_string(diagnosis_rc);
        }
      }
      if (request_lock.owns_lock()) {request_lock.unlock();}
      // Publish diagnosis immediately. A slow subsequent state getter must
      // not hide an already completed, fresh diagnosis from the control tick.
      {
        std::unique_lock<std::mutex> state_lock(request_mutex, std::defer_lock);
        if (!streaming) {state_lock.lock();}
        try {
          state_rc = read->robotServiceGetRobotCurrentState(state);
          state_time = steady_seconds();
        } catch (...) {}
      }
      {
        std::lock_guard<std::mutex> lock(mutex);
        latest.state_result = state_rc;
        if (state_rc == vendor::ErrCode_RequestTimeout) {++latest.state_read_timeouts;}
        latest.query_duration_s = steady_seconds() - query_started_at;
        latest.feedback.stopped = state_rc == vendor::ErrnoSucc && state == vendor::RobotStopped;
        if (state_rc != vendor::ErrCode_RequestTimeout) {
          state_valid = state_rc == vendor::ErrnoSucc;
        }
        if (state_rc == vendor::ErrnoSucc) {latest.robot_state_at = state_time;}
        if (state_rc != vendor::ErrnoSucc) {
          latest.error = "robot state query failed: " + std::to_string(state_rc);
          latest.last_telemetry_failure = latest.error;
        }
        changed.notify_all();
      }
      std::unique_lock<std::mutex> lock(mutex);
      if (options.adaptive_follow_ik && !latest.kinematics_verified &&
        latest.feedback.sequence > 0)
      {
        const auto check = latest;
        lock.unlock();
        vendor::wayPoint_S checked{};
        int rc = -1;
        {
          std::lock_guard<std::mutex> fk_lock(request_mutex);
          try {rc = read->robotServiceRobotFk(check.joints.data(), 6, checked);} catch (...) {}
        }
        PoseSample pose;
        pose.position =
        {checked.cartPos.position.x, checked.cartPos.position.y, checked.cartPos.position.z};
        pose.orientation =
        {checked.orientation.x, checked.orientation.y, checked.orientation.z,
          checked.orientation.w};
        pose.joints = check.joints;
        double squared = 0.0;
        for (unsigned i = 0; i < 3; ++i) {
          const double delta = pose.position[i] - check.feedback.position[i];
          squared += delta * delta;
        }
        const double error = std::sqrt(squared);
        const bool verified = rc == 0 && valid_pose_sample(pose, nullptr) && error <= 0.001 &&
          quaternion_distance_rad(pose.orientation, check.feedback.orientation) <= 0.01;
        lock.lock();
        latest.kinematics_verified = verified;
        latest.fk_position_error_m = error;
        if (!verified) {latest.error = "FK verification did not match measured pose";}
      }
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
    !std::isfinite(options.maximum_drag_step_m) || options.maximum_drag_step_m <= 0.0 ||
    options.maximum_drag_step_m > 0.020 ||
    !std::isfinite(options.drag_intent_timeout_s) || options.drag_intent_timeout_s <= 0.0 ||
    options.drag_intent_timeout_s > 0.5 ||
    !std::isfinite(options.feedback_timeout_s) || options.feedback_timeout_s <= 0.0 ||
    options.feedback_timeout_s > 1.0 ||
    !std::isfinite(options.drag_release_prediction_s) ||
    options.drag_release_prediction_s < 0.0 || options.drag_release_prediction_s > 0.1 ||
    options.state_poll_interval_s < 0.02 || options.state_poll_interval_s > 1.0)
  {
    error = "invalid stepwise transport configuration or repeated connect";
    return false;
  }
  auto & self = *impl_;
  self.options = options;
  if (options.experimental_tcp_can_drag) {
    if (!options.allow_follow_mode || !options.fixed_follow_profile) {
      error = "TCP2CAN candidate requires the independent drag configuration";
      return false;
    }
    TcpCanConfig config;
    config.maximum_speed_mps = options.speed_mps;
    config.maximum_lead_m = options.maximum_drag_step_m;
    config.maximum_ik_step_rad = options.maximum_joint_step_rad;
    config.joint_speed_radps = options.follow_joint_speed_radps;
    config.joint_acceleration_radps2 = options.follow_joint_acceleration_radps2;
    self.tcp_can = std::make_unique<TcpCanDragSession>(config);
  }
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
    if (self.read->robotServiceRegisterRobotEventInfoCallback(&Impl::robot_event, &self) !=
      vendor::ErrnoSucc ||
      self.send->robotServiceRegisterRobotEventInfoCallback(&Impl::robot_event, &self) !=
      vendor::ErrnoSucc ||
      self.stop->robotServiceRegisterRobotEventInfoCallback(&Impl::robot_event, &self) !=
      vendor::ErrnoSucc)
    {
      error = "controller event subscription rejected";
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
  if (value.tcp_can_fault) {value.error = value.tcp_can_status;}
  value.brake_on_hold = impl_->options.brake_on_hold;
  value.feedback.healthy = value.diagnosis_ok && impl_->state_valid &&
    impl_->pose_valid && !value.tcp_can_fault &&
    (!impl_->options.adaptive_follow_ik || value.kinematics_verified);
  return value;
}
bool StepwiseTransport::submit(const StepwiseCommand & command, const std::array<double, 6> & seed)
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  const bool admissible_during_feed = impl_->continuation_busy && command.assisted_drag &&
    command.epoch == impl_->allowed_epoch;
  if (impl_->shutdown || (impl_->busy && !admissible_during_feed) || impl_->job ||
    impl_->stop_job ||
    (command.follow_joint_mode && !impl_->options.allow_follow_mode) ||
    (command.hold_position && !command.follow_joint_mode) ||
    (command.assisted_drag && !command.follow_joint_mode) ||
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
    (command.cartesian_drag_reference && (!command.assisted_drag || !command.follow_joint_mode)) ||
    (command.cartesian_drag_reference && (!std::isfinite(command.drag_reference_horizon_s) ||
    command.drag_reference_horizon_s <= 0.0 || command.drag_reference_horizon_s > 0.5)) ||
    (squared_step <= 0.0 && !command.hold_position) ||
    (command.hold_position && squared_step > 1e-20) ||
    squared_step > std::pow(
      command.assisted_drag ? impl_->options.maximum_drag_step_m :
      impl_->options.maximum_step_m, 2) * (1.0 + 1e-9) ||
    !std::isfinite(quaternion_norm) || std::abs(quaternion_norm - 1.0) > 1e-3) {return false;}
  impl_->last_submitted_id = command.id;
  impl_->allowed_epoch = command.epoch;
  impl_->allowed = true;
  impl_->job = Impl::Job{command, seed};
  impl_->changed.notify_all();
  return true;
}
bool StepwiseTransport::update_drag_intent(
  const std::array<double, 3> & velocity, std::uint64_t epoch, double received_at)
{
  if (!std::isfinite(received_at)) {return false;}
  for (double value : velocity) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (impl_->drag_intent && received_at < impl_->drag_intent->received_at) {return false;}
  if (!impl_->drag_intent || impl_->drag_intent->epoch != epoch ||
    impl_->drag_intent->velocity != velocity)
  {
    ++impl_->drag_intent_generation;
  }
  impl_->drag_intent =
    Impl::DragIntent{velocity, epoch, impl_->drag_intent_generation, received_at};
  return true;
}

bool StepwiseTransport::quiescent() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return !impl_->allowed && !impl_->busy && !impl_->job && !impl_->stop_job &&
         !impl_->stop_in_progress && impl_->results.empty();
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
  impl_->last_stream_job.reset();
  impl_->drag_intent.reset();
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
