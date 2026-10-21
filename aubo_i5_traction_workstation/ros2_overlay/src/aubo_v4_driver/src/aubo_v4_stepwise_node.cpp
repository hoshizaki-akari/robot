#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <fr_traction/msg/end_effector_state.hpp>
#include <fr_traction/msg/traction_command.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <future>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include "aubo_v4_driver/stepwise_scheduler.hpp"
#include "aubo_v4_driver/stepwise_transport.hpp"
#include "aubo_v4_driver/stop_handoff.hpp"

namespace av = aubo_v4_driver;
namespace
{
std::string environment(const char * key)
{
  const char * value = std::getenv(key);
  return value == nullptr ? std::string{} : std::string(value);
}
std::string state_name(av::StepwiseState state)
{
  switch (state) {
    case av::StepwiseState::DISABLED: return "DISABLED";
    case av::StepwiseState::READY: return "READY";
    case av::StepwiseState::SENDING: return "SENDING";
    case av::StepwiseState::MOVING: return "MOVING";
    case av::StepwiseState::SETTLING: return "SETTLING";
    case av::StepwiseState::STOPPING: return "STOPPING";
    case av::StepwiseState::FAULT: return "FAULT";
  }
  return "INVALID";
}
}

class AuboV4StepwiseNode final : public rclcpp::Node
{
public:
  AuboV4StepwiseNode()
  : Node("aubo_v4_state_node")
  {
    // Same ROS identity/service names as the legacy backend, but never launch
    // both. Two independent opt-ins protect against accidental hardware use.
    const bool connect_hardware = declare_parameter<bool>("connect_hardware", false);
    enable_motion_ = declare_parameter<bool>("enable_motion_interface", false);
    enable_follow_drag_ = declare_parameter<bool>("enable_follow_drag", false);
    enable_follow_traction_ = declare_parameter<bool>("enable_follow_traction", false);
    telemetry_timeout_s_ = declare_parameter("telemetry_timeout_s", 0.5);
    const double follow_response_scale = declare_parameter("follow_response_scale", 1.0);
    if (!std::isfinite(telemetry_timeout_s_) || telemetry_timeout_s_ <= 0.0 ||
      !std::isfinite(follow_response_scale) || follow_response_scale < 1.0 ||
      follow_response_scale > 5.0)
    {
      throw std::invalid_argument("invalid telemetry timeout or follow response scale");
    }
    av::StepwiseConfig config;
    config.maximum_step_m = declare_parameter("maximum_step_m", 0.0005);
    config.maximum_speed_mps = declare_parameter("maximum_speed_mps", 0.050);
    config.horizon_s = declare_parameter("planning_horizon_s", 0.1);
    config.feedback_timeout_s = declare_parameter("feedback_timeout_s", 0.5);
    config.command_timeout_s = declare_parameter("command_timeout_s", 0.25);
    config.ack_timeout_s = declare_parameter("ack_timeout_s", 2.0);
    config.stop_timeout_s = declare_parameter("stop_timeout_s", 5.0);
    config.completion_timeout_s = declare_parameter("completion_timeout_s", 5.0);
    config.settling_time_s = declare_parameter("settling_time_s", 0.1);
    config.arrival_tolerance_m = declare_parameter("arrival_tolerance_m", 0.00005);
    config.quiet_speed_mps = declare_parameter("quiet_speed_mps", 0.0002);
    config.follow_update_interval_s = declare_parameter("follow_update_interval_s", 0.1);
    config.follow_horizon_s = declare_parameter("follow_horizon_s", 0.15);
    config.follow_hold_enabled = declare_parameter("follow_hold_enabled", false);
    config.follow_reference_enabled = declare_parameter("follow_reference_enabled", false);
    config.drag_maximum_step_m = declare_parameter("drag_maximum_step_m", 0.0);
    config.drag_follow_horizon_s = declare_parameter("drag_follow_horizon_s", 0.0);
    const bool fixed_follow_profile = declare_parameter("fixed_follow_profile", false);
    if (fixed_follow_profile && !config.follow_reference_enabled) {
      throw std::invalid_argument("fixed follow profiles require a time-paced reference");
    }
    if (config.maximum_step_m > 0.005 || config.maximum_speed_mps > 0.200 ||
      config.horizon_s > 0.5 || config.feedback_timeout_s > 1.0)
    {
      throw std::invalid_argument("stepwise prototype step/speed/timing limits exceeded");
    }
    config_ = config;
    scheduler_ = std::make_unique<av::StepwiseScheduler>(config_);
    pose_frame_ = declare_parameter<std::string>("pose_frame_id", "base_link");
    maximum_session_travel_ = declare_parameter("maximum_session_travel_m", 0.0);
    if (!std::isfinite(maximum_session_travel_) || maximum_session_travel_ < 0.0) {
      throw std::invalid_argument(
              "maximum_session_travel_m must be nonnegative; zero disables the session bubble");
    }
    session_orientation_tolerance_rad_ = declare_parameter(
      "session_orientation_tolerance_rad", 0.03);
    if (!std::isfinite(session_orientation_tolerance_rad_) ||
      session_orientation_tolerance_rad_ <= 0.0 || session_orientation_tolerance_rad_ > 0.15)
    {
      throw std::invalid_argument("session_orientation_tolerance_rad must be in (0,0.15]");
    }
    names_ = declare_parameter<std::vector<std::string>>(
      "joint_names",
      {"shoulder_joint", "upperArm_joint", "foreArm_joint", "wrist1_joint", "wrist2_joint",
        "wrist3_joint"});
    if (pose_frame_.empty() || names_.size() != 6) {
      throw std::invalid_argument("six joint names and nonempty pose frame required");
    }
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    pose_pub_ = create_publisher<fr_traction::msg::EndEffectorState>(
      "/controller_manager/ee_state",
      10);
    health_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/controller_manager/healthy", rclcpp::QoS(
        1).transient_local());
    idle_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/controller_manager/motion_idle", rclcpp::QoS(
        1).transient_local());
    identity_pub_ =
      create_publisher<std_msgs::msg::String>(
      "/aubo_i5/driver_identity", rclcpp::QoS(
        1).transient_local());
    phase_pub_ = create_publisher<std_msgs::msg::String>(
      "/aubo_i5/stepwise_phase", rclcpp::QoS(
        1).transient_local());
    diagnostic_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::SensorDataQoS());
    session_limit_client_ = create_client<std_srvs::srv::Trigger>("/traction/session_limit_stop");
    mode_sub_ = create_subscription<fr_traction::msg::TractionCommand>(
      "/traction/command", rclcpp::QoS(1).reliable(),
      [this](fr_traction::msg::TractionCommand::ConstSharedPtr message) {
        last_command_mode_ = message->mode;
        scheduler_->set_drag_mode(message->mode == fr_traction::msg::TractionCommand::DRAG);
        mode_at_ = av::steady_seconds();
      });
    velocity_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/controller_manager/command_cart_vel", rclcpp::QoS(1).reliable(),
      [this](geometry_msgs::msg::Twist::ConstSharedPtr message) {
        const auto time = av::steady_seconds();
        ++intent_sequence_;
        intent_at_ = time;
        if (!std::isfinite(message->angular.x) || !std::isfinite(message->angular.y) ||
        !std::isfinite(message->angular.z) || std::abs(message->angular.x) > 1e-12 ||
        std::abs(message->angular.y) > 1e-12 || std::abs(message->angular.z) > 1e-12)
        {
          session_enabled_ = false;
          scheduler_->request_stop(
            time,
            "angular command unsupported in fixed-pose stepwise backend", true);
          return;
        }
        if (session_enabled_) {
          if ((enable_follow_drag_ || enable_follow_traction_) &&
          scheduler_->state() == av::StepwiseState::READY)
          {
            if (time - mode_at_ > config_.command_timeout_s) {return;}
            scheduler_->set_follow_mode(
              (enable_follow_drag_ &&
              last_command_mode_ == fr_traction::msg::TractionCommand::DRAG) ||
              (enable_follow_traction_ &&
              last_command_mode_ >= fr_traction::msg::TractionCommand::PRETENSION &&
              last_command_mode_ <= fr_traction::msg::TractionCommand::POSITIONING &&
              last_command_mode_ != fr_traction::msg::TractionCommand::DRAG), time);
          }
          scheduler_->set_velocity({message->linear.x, message->linear.y, message->linear.z}, time);
        }
      });
    switch_service_ = create_service<controller_manager_msgs::srv::SwitchController>(
      "/controller_manager/switch_controller",
      [this](const std::shared_ptr<controller_manager_msgs::srv::SwitchController::Request> request,
      std::shared_ptr<controller_manager_msgs::srv::SwitchController::Response> response) {
        const auto valid = [](const std::vector<std::string> & names) {
          return std::all_of(
            names.begin(), names.end(), [](const std::string & name) {
              return name == "cartesian_velocity_controller";
            });
        };
        if (!valid(request->start_controllers) || !valid(request->stop_controllers) ||
        (!request->start_controllers.empty() && !request->stop_controllers.empty()))
        {
          response->ok = false;
          return;
        }
        if (!request->stop_controllers.empty()) {
          session_enabled_ = false;
          scheduler_->set_enabled(false, av::steady_seconds());
          // Accepted stop request, NOT a claim of already measured idle.
          // The manager's completion gates separately require motion_idle.
          response->ok = !scheduler_->fault_pending();
          return;
        }
        if (request->start_controllers.empty()) {response->ok = true; return;}
        if (session_limit_handoff_pending_) {response->ok = false; return;}
        if (session_enabled_) {
          response->ok = !scheduler_->fault_pending();
          return;
        }
        response->ok = connected_ && enable_motion_ &&
        scheduler_->set_enabled(true, av::steady_seconds());
        session_enabled_ = response->ok;
        if (response->ok) {
          session_limit_reached_ = false;
          const auto sample = transport_->sample();
          session_origin_ = sample.feedback.position;
          session_orientation_ = sample.feedback.orientation;
        }
      });
    stop_service_ = create_service<std_srvs::srv::Trigger>(
      "~/hardware_stop",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        session_enabled_ = false;
        if (!connected_) {
          response->success = false;
          response->message = "No hardware connection; no stop request sent.";
          return;
        }
        scheduler_->request_stop(av::steady_seconds(), "operator hardware-stop request", true);
        const auto stop_epoch = scheduler_->take_stop_request();
        transport_->cancel_and_stop(
          stop_epoch.value_or(
            scheduler_->epoch()));
        response->success = true;
        response->message = "Stop requested; physical stop is not yet confirmed.";
      });
    recover_service_ = create_service<std_srvs::srv::Trigger>(
      "~/recover_fault",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        const double time = av::steady_seconds();
        if (!connected_) {
          response->message = "Driver recovery rejected: hardware disconnected.";
          return;
        }
        const auto sample = transport_->sample();
        if (!sample.feedback.healthy || !sample.diagnosis_ok ||
        time - sample.feedback.received_at > config_.feedback_timeout_s ||
        time - sample.diagnosis_at > telemetry_timeout_s_ ||
        time - sample.robot_state_at > telemetry_timeout_s_)
        {
          response->message = "Driver recovery rejected: live telemetry unhealthy or stale.";
          return;
        }
        if (session_enabled_ || (scheduler_->state() != av::StepwiseState::FAULT &&
        scheduler_->state() != av::StepwiseState::STOPPING &&
        scheduler_->state() != av::StepwiseState::DISABLED))
        {
          response->message = "Driver recovery rejected: a motion session is active.";
          return;
        }
        if (sample.feedback.stopped && sample.feedback.speed_mps <= config_.quiet_speed_mps &&
        transport_->quiescent() && (scheduler_->reset_fault(time) ||
        (scheduler_->state() == av::StepwiseState::DISABLED && scheduler_->motion_idle(time))))
        {
          recovery_in_progress_ = false;
          resume_after_intent_ = ++intent_sequence_;
          intent_at_ = 0.0;
          mode_at_ = 0.0;
          last_command_mode_ = 0;
          session_limit_reached_ = false;
          last_motion_failure_.clear();
          publish_status(time);
          response->success = true;
          response->message = "Driver fault reset; motion disabled; initial calibration required.";
          return;
        }
        if (!recovery_in_progress_) {
          recovery_in_progress_ = true;
          recovery_started_at_ = time;
          scheduler_->request_stop(time, "operator fault recovery stop", true);
        } else if (time - recovery_started_at_ > 5.0 ||
        (scheduler_->state() == av::StepwiseState::FAULT && transport_->quiescent()))
        {
          recovery_in_progress_ = false;
          response->message = "Driver recovery failed: measured stop was not confirmed.";
          return;
        }
        response->message = "RECOVERY_PENDING: waiting for fresh measured stop confirmation.";
      });
    if (connect_hardware) {
      av::TransportOptions options;
      options.host = environment("AUBO_V4_HOST");
      options.username = environment("AUBO_V4_USERNAME");
      options.password = environment("AUBO_V4_PASSWORD");
      const auto text_port = environment("AUBO_V4_PORT");
      if (!text_port.empty()) {
        std::size_t consumed = 0;
        options.port = std::stoi(text_port, &consumed);
        if (consumed != text_port.size()) {throw std::invalid_argument("invalid AUBO_V4_PORT");}
      }
      options.speed_mps = config_.maximum_speed_mps;
      options.maximum_step_m = config_.maximum_step_m;
      options.acceleration_mps2 = declare_parameter("line_acceleration_mps2", 0.01);
      options.fast_acceleration_mps2 = declare_parameter("fast_line_acceleration_mps2", 0.01);
      options.state_poll_interval_s = declare_parameter("state_poll_interval_s", 0.1);
      options.allow_follow_mode = enable_follow_drag_ || enable_follow_traction_;
      options.fixed_follow_profile = fixed_follow_profile;
      options.single_stop_when_quiescent = declare_parameter("single_stop_when_quiescent", false);
      options.brake_on_hold = declare_parameter("brake_on_hold", false);
      options.drag_release_prediction_s = declare_parameter("drag_release_prediction_s", 0.0);
      options.adaptive_follow_ik = declare_parameter("adaptive_follow_ik", false);
      options.maximum_drag_step_m = std::max(config_.maximum_step_m, config_.drag_maximum_step_m);
      options.subdivide_follow_joint_steps =
        declare_parameter("subdivide_follow_joint_steps", false);
      options.follow_joint_speed_radps = 0.2 * follow_response_scale;
      options.follow_joint_acceleration_radps2 = 0.5 * follow_response_scale;
      transport_ = std::make_unique<av::StepwiseTransport>();
      connected_ = transport_->connect(options, connection_error_);
    } else {
      connection_error_ =
        "OFFLINE: connect_hardware=false; no SDK objects or controller connections";
    }
    timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]() {tick();});
    publish_status(av::steady_seconds());
  }

private:
  void tick()
  {
    const auto tick_time = av::steady_seconds();
    if (last_tick_at_ > 0.0) {
      const double gap = tick_time - last_tick_at_;
      maximum_tick_gap_s_ = std::max(maximum_tick_gap_s_, gap);
      if (session_enabled_ && gap > config_.feedback_timeout_s) {
        scheduler_->request_stop(
          tick_time, "driver execution gap " + std::to_string(gap) + " s", true);
      }
    }
    last_tick_at_ = tick_time;
    if (session_limit_handoff_pending_) {
      bool ready = session_limit_future_.wait_for(std::chrono::seconds(0)) ==
        std::future_status::ready;
      bool accepted = false;
      if (ready) {
        try {accepted = session_limit_future_.get()->success;} catch (...) {accepted = false;}
      }
      const auto state = av::stop_handoff_status(
        ready, accepted, tick_time, session_limit_deadline_);
      if (state != av::StopHandoff::PENDING) {
        session_limit_handoff_pending_ = false;
        if (state != av::StopHandoff::ACCEPTED) {
          session_limit_handoff_error_ = state == av::StopHandoff::TIMED_OUT ?
            "session-limit stop handoff timeout" : "session-limit stop rejected by manager";
          if (state == av::StopHandoff::TIMED_OUT) {
            session_limit_client_->remove_pending_request(session_limit_request_id_);
          }
          session_enabled_ = false;
          scheduler_->request_stop(tick_time, session_limit_handoff_error_, true);
        }
      }
    }
    if (!connected_) {publish_status(av::steady_seconds()); return;}
    if (count_publishers("/aubo_i5/driver_identity") > 1 ||
      count_publishers("/controller_manager/command_cart_vel") > 1)
    {
      session_enabled_ = false;
      scheduler_->request_stop(
        av::steady_seconds(), "multiple robot drivers or motion-command publishers", true);
    }
    const auto sample = transport_->sample();
    if (sample.controller_error_sequence > handled_controller_error_) {
      handled_controller_error_ = sample.controller_error_sequence;
      session_enabled_ = false;
      scheduler_->request_stop(
        av::steady_seconds(),
        "controller event " + std::to_string(sample.controller_error_type) + ": " +
        sample.controller_error_content, true);
    }
    while (const auto result = transport_->take_result()) {
      last_sdk_result_ = result->detail;
      if (!result->success) {
        last_motion_failure_ = result->detail;
      }
      const auto time = av::steady_seconds();
      state_fence_at_ = time;
      if (result->stop) {
        scheduler_->acknowledge_stop(result->epoch, result->success, time);
      } else {
        scheduler_->acknowledge(
          result->id, result->epoch, result->success, time, result->detail,
          result->recoverable && sample.feedback.healthy &&
          time - sample.diagnosis_at <= telemetry_timeout_s_ &&
          time - sample.robot_state_at <= telemetry_timeout_s_);
      }
    }
    const auto time = av::steady_seconds();
    if (sample.feedback.sequence > 0) {
      auto feedback = sample.feedback;
      feedback.healthy = feedback.healthy &&
        time - sample.diagnosis_at <= telemetry_timeout_s_ &&
        time - sample.robot_state_at <= telemetry_timeout_s_;
      feedback.stopped = feedback.stopped && sample.robot_state_at >= state_fence_at_;
      if (session_enabled_ && !feedback.healthy) {
        scheduler_->request_stop(time, "unhealthy or stale SDK diagnosis/state", true);
      }
      if (session_enabled_) {
        const double angle = av::quaternion_distance_rad(
          feedback.orientation, session_orientation_);
        if (!std::isfinite(angle) || angle > session_orientation_tolerance_rad_) {
          scheduler_->request_stop(
            time,
            "fixed-pose session orientation changed; re-calibration required",
            true);
        }
      }
      scheduler_->set_feedback(feedback, time);
      if (feedback.sequence != published_sequence_ &&
        time - feedback.received_at <= config_.feedback_timeout_s)
      {
        publish_measured(sample);
        published_sequence_ = feedback.sequence;
      }
    }
    scheduler_->poll(av::steady_seconds());
    if (scheduler_->state() == av::StepwiseState::STOPPING &&
      previous_phase_ != av::StepwiseState::STOPPING)
    {
      resume_after_intent_ = intent_sequence_;
    }
    if (const auto stop_epoch = scheduler_->take_stop_request()) {
      transport_->cancel_and_stop(*stop_epoch);
    }
    // Only internal zero/stale-intent pauses can resume on a NEW command.
    // Explicit user stop, mode switch and faults revoke session_enabled_.
    if (session_enabled_ && scheduler_->state() == av::StepwiseState::DISABLED &&
      intent_sequence_ > resume_after_intent_ && time - intent_at_ <= config_.command_timeout_s)
    {
      scheduler_->set_enabled(true, av::steady_seconds());
    }
    if (session_enabled_ && enable_motion_) {
      if (auto command = scheduler_->take_command(av::steady_seconds())) {
        // Keep the original session orientation even across internal zero/
        // watchdog pauses; measured IK error must not become the next target.
        if (!command->hold_position) {command->orientation = session_orientation_;}
        double squared_travel = 0.0;
        for (std::size_t index = 0; index < 3; ++index) {
          const double delta = command->target[index] - session_origin_[index];
          squared_travel += delta * delta;
        }
        if (maximum_session_travel_ > 0.0 && std::sqrt(squared_travel) > maximum_session_travel_) {
          session_enabled_ = false;
          session_limit_reached_ = true;
          scheduler_->request_stop(
            av::steady_seconds(), "stepwise session travel bound reached", false);
          if (session_limit_client_->service_is_ready()) {
            try {
              auto request = session_limit_client_->async_send_request(
                std::make_shared<std_srvs::srv::Trigger::Request>());
              session_limit_request_id_ = request.request_id;
              session_limit_future_ = request.future.share();
              session_limit_deadline_ = av::steady_seconds() + 2.0;
              session_limit_handoff_pending_ = true;
            } catch (...) {
              scheduler_->request_stop(
                av::steady_seconds(), "session-limit stop request failed", true);
            }
          } else {
            scheduler_->request_stop(
              av::steady_seconds(), "session-limit stop handoff unavailable", true);
          }
        } else if (!transport_->submit(*command, sample.joints)) {
          scheduler_->acknowledge(command->id, command->epoch, false, av::steady_seconds());
        }
      }
    }
    if (scheduler_->fault_pending()) {session_enabled_ = false;}
    previous_phase_ = scheduler_->state();
    publish_status(av::steady_seconds());
  }

  void publish_measured(const av::TransportSample & sample)
  {
    sensor_msgs::msg::JointState joints;
    const double source_age = std::max(0.0, av::steady_seconds() - sample.feedback.received_at);
    joints.header.stamp = now() - rclcpp::Duration::from_seconds(source_age);
    joints.name = names_;
    joints.position.assign(sample.joints.begin(), sample.joints.end());
    fr_traction::msg::EndEffectorState pose;
    pose.header.stamp = joints.header.stamp;
    pose.header.frame_id = pose_frame_;
    const auto & feedback = sample.feedback;
    pose.pose.position.x = feedback.position[0];
    pose.pose.position.y = feedback.position[1];
    pose.pose.position.z = feedback.position[2];
    pose.pose.orientation.x = feedback.orientation[0];
    pose.pose.orientation.y = feedback.orientation[1];
    pose.pose.orientation.z = feedback.orientation[2];
    pose.pose.orientation.w = feedback.orientation[3];
    if (previous_sample_) {
      const double dt = feedback.received_at - previous_sample_->feedback.received_at;
      if (dt > 0.0 && dt <= config_.feedback_timeout_s) {
        for (std::size_t index = 0; index < 6; ++index) {
          joints.velocity.push_back((sample.joints[index] - previous_sample_->joints[index]) / dt);
        }
        pose.twist.linear.x = (feedback.position[0] - previous_sample_->feedback.position[0]) / dt;
        pose.twist.linear.y = (feedback.position[1] - previous_sample_->feedback.position[1]) / dt;
        pose.twist.linear.z = (feedback.position[2] - previous_sample_->feedback.position[2]) / dt;
      }
    }
    joint_pub_->publish(joints);
    pose_pub_->publish(pose);
    previous_sample_ = sample;
  }

  void publish_status(double time)
  {
    bool healthy = false;
    std::string health_failure = connection_error_;
    if (connected_) {
      const auto sample = transport_->sample();
      healthy = sample.feedback.healthy &&
        time - sample.feedback.received_at <= config_.feedback_timeout_s &&
        time - sample.diagnosis_at <= telemetry_timeout_s_ &&
        time - sample.robot_state_at <= telemetry_timeout_s_ && !scheduler_->fault_pending();
      health_failure.clear();
      const auto append = [&health_failure](const std::string & reason) {
          if (!health_failure.empty()) {health_failure += "; ";}
          health_failure += reason;
        };
      if (!sample.feedback.healthy) {
        append("SDK pose/diagnosis unhealthy: " + sample.error);
      }
      if (time - sample.feedback.received_at > config_.feedback_timeout_s) {
        append("waypoint stale");
      }
      if (time - sample.diagnosis_at > telemetry_timeout_s_) {
        append("diagnosis stale");
      }
      if (time - sample.robot_state_at > telemetry_timeout_s_) {
        append("robot state stale");
      }
      if (scheduler_->fault_pending()) {append("motion fault: " + scheduler_->fault_reason());}
    }
    const bool health_changed = !health_status_published_ || healthy != published_healthy_;
    if (!healthy && health_changed) {
      last_health_failure_ = health_failure;
      RCLCPP_ERROR(get_logger(), "AUBO hardware health lost: %s", health_failure.c_str());
    }
    health_status_published_ = true;
    published_healthy_ = healthy;
    std_msgs::msg::Bool health;
    health.data = healthy;
    health_pub_->publish(health);
    std_msgs::msg::Bool idle;
    if (connected_) {
      const auto sample = transport_->sample();
      idle.data = healthy && scheduler_->motion_idle(time) && sample.feedback.stopped &&
        sample.robot_state_at >= state_fence_at_;
    }
    idle_pub_->publish(idle);
    std_msgs::msg::String identity;
    // Identity and health are separate facts. Keep identifying the connected
    // robot during a latched motion/diagnostic fault so the workstation can
    // show the fault while still reporting live communications accurately.
    identity.data = connected_ ? "AUBO_I5_V4" : "";
    identity_pub_->publish(identity);
    std_msgs::msg::String phase;
    phase.data = state_name(scheduler_->state());
    phase_pub_->publish(phase);
    // Hardware health is still published every tick; bounded diagnostic traffic
    // avoids flooding the Python bridge with a large reliable DDS sample.
    if (!health_changed && phase.data == last_diagnostic_phase_ &&
      time - last_diagnostic_at_ < 0.1) {return;}
    last_diagnostic_at_ = time;
    last_diagnostic_phase_ = phase.data;
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "aubo_v4_driver/state";
    status.hardware_id = "AUBOPE V4 stepwise";
    status.level =
      healthy ? diagnostic_msgs::msg::DiagnosticStatus::OK : diagnostic_msgs::msg::DiagnosticStatus
      ::ERROR;
    status.message = connected_ ? scheduler_->reason() : connection_error_;
    if (connected_ && !scheduler_->fault_reason().empty()) {
      status.message = scheduler_->fault_reason() + "; " + scheduler_->reason();
    }
    const auto add = [&status](const std::string & key, const std::string & value) {
        diagnostic_msgs::msg::KeyValue entry;
        entry.key = key;
        entry.value = value;
        status.values.push_back(entry);
      };
    add("backend", "stepwise");
    add("connected", connected_ ? "1" : "0");
    add("motion_interface_enabled", enable_motion_ ? "1" : "0");
    add("motion_stream_active", session_enabled_ ? "1" : "0");
    add("stepwise_phase", phase.data);
    add("motion_idle", idle.data ? "1" : "0");
    if (connected_) {
      const auto sample = transport_->sample();
      add("sdk_robot_stopped", sample.feedback.stopped ? "1" : "0");
      add("brake_on_hold", sample.brake_on_hold ? "1" : "0");
      add("stop_timeout_s", std::to_string(config_.stop_timeout_s));
      add("communication_recoveries", std::to_string(scheduler_->communication_recoveries()));
      add("measured_cartesian_speed_mps", std::to_string(sample.feedback.speed_mps));
      add("measured_joint_speed_norm_radps", std::to_string(sample.joint_speed_norm_radps));
      add(
        "drag_release_predicted_joint_step_rad",
        std::to_string(sample.drag_release_predicted_joint_step_rad));
      add("controller_event_type", std::to_string(sample.controller_event_type));
      add("controller_event_code", std::to_string(sample.controller_event_code));
      add("controller_event_content", sample.controller_event_content);
      add("controller_error_type", std::to_string(sample.controller_error_type));
      add("controller_error_content", sample.controller_error_content);
    }
    add("epoch", std::to_string(scheduler_->epoch()));
    add("last_sdk_result", last_sdk_result_);
    add("last_motion_failure", last_motion_failure_);
    add("fault_reason", scheduler_->fault_reason());
    add("health_failure_reason", health_failure);
    add("last_hardware_health_failure", last_health_failure_);
    add("planning_horizon_s", std::to_string(config_.horizon_s));
    add("maximum_step_m", std::to_string(config_.maximum_step_m));
    add("drag_maximum_step_m", std::to_string(config_.drag_maximum_step_m));
    add("drag_follow_horizon_s", std::to_string(config_.drag_follow_horizon_s));
    add("arrival_tolerance_m", std::to_string(config_.arrival_tolerance_m));
    add("maximum_speed_mps", std::to_string(config_.maximum_speed_mps));
    add("enable_follow_drag", enable_follow_drag_ ? "1" : "0");
    add("enable_follow_traction", enable_follow_traction_ ? "1" : "0");
    add("follow_update_interval_s", std::to_string(config_.follow_update_interval_s));
    add("follow_horizon_s", std::to_string(config_.follow_horizon_s));
    add("follow_hold_enabled", config_.follow_hold_enabled ? "1" : "0");
    add("follow_reference_enabled", config_.follow_reference_enabled ? "1" : "0");
    add("telemetry_timeout_s", std::to_string(telemetry_timeout_s_));
    add("session_limit_reached", session_limit_reached_ ? "1" : "0");
    add("session_limit_handoff_error", session_limit_handoff_error_);
    add("maximum_session_travel_m", std::to_string(maximum_session_travel_));
    add("session_orientation_tolerance_rad", std::to_string(session_orientation_tolerance_rad_));
    if (transport_) {
      const auto sample = transport_->sample();
      add("sdk_profile_speed_mps", std::to_string(sample.sdk_speed_mps));
      add("sdk_profile_acceleration_mps2", std::to_string(sample.sdk_acceleration_mps2));
      add("waypoint_interval_s", std::to_string(sample.waypoint_interval_s));
      add("waypoint_sequence", std::to_string(sample.feedback.sequence));
      add("motion_ack_count", std::to_string(sample.motion_ack_count));
      add("last_command_id", std::to_string(sample.last_command_id));
      add("last_ik_result", std::to_string(sample.last_ik_result));
      add("last_ik_max_joint_delta_rad", std::to_string(sample.last_ik_max_joint_delta_rad));
      add(
        "proposed_ik_max_joint_delta_rad",
        std::to_string(sample.proposed_ik_max_joint_delta_rad));
      add("follow_joint_subdivisions", std::to_string(sample.follow_joint_subdivisions));
      add("accepted_ik_scale", std::to_string(sample.accepted_ik_scale));
      add("accepted_cartesian_step_m", std::to_string(sample.accepted_cartesian_step_m));
      add("adaptive_ik_attempts", std::to_string(sample.adaptive_ik_attempts));
      add("fk_result", std::to_string(sample.fk_result));
      add("kinematics_verified", sample.kinematics_verified ? "1" : "0");
      add("fk_position_error_m", std::to_string(sample.fk_position_error_m));
      add("last_ik_joint_index", std::to_string(sample.last_ik_joint_index));
      add("last_ik_failure", sample.last_ik_failure);
      add("requested_speed_mps", std::to_string(sample.requested_speed_mps));
      add("diagnosis_age_s", std::to_string(time - sample.diagnosis_at));
      add("robot_state_age_s", std::to_string(time - sample.robot_state_at));
      add("waypoint_age_s", std::to_string(time - sample.feedback.received_at));
      add("diagnosis_result", std::to_string(sample.diagnosis_result));
      add("state_result", std::to_string(sample.state_result));
      add("sdk_normal_requests_serialized", "1");
      add("diagnosis_read_timeouts", std::to_string(sample.diagnosis_read_timeouts));
      add("state_read_timeouts", std::to_string(sample.state_read_timeouts));
      add("query_duration_s", std::to_string(sample.query_duration_s));
      add("ik_duration_s", std::to_string(sample.ik_duration_s));
      add("motion_call_duration_s", std::to_string(sample.motion_call_duration_s));
      add("dispatch_interval_s", std::to_string(sample.dispatch_interval_s));
      add("maximum_dispatch_interval_s", std::to_string(sample.maximum_dispatch_interval_s));
      add("maximum_ik_duration_s", std::to_string(sample.maximum_ik_duration_s));
      add("maximum_motion_call_duration_s", std::to_string(sample.maximum_motion_call_duration_s));
      add("telemetry_error", sample.error);
      add("last_telemetry_failure", sample.last_telemetry_failure);
      add("follow_joint_mode", sample.follow_joint_mode ? "1" : "0");
      add("follow_profile_updates", std::to_string(sample.follow_profile_updates));
      add("follow_joint_speed_max_radps", std::to_string(sample.follow_joint_speed_max_radps));
      add(
        "follow_joint_acceleration_max_radps2",
        std::to_string(sample.follow_joint_acceleration_max_radps2));
    }
    add("maximum_tick_gap_s", std::to_string(maximum_tick_gap_s_));
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    array.status.push_back(status);
    diagnostic_pub_->publish(array);
  }

  av::StepwiseConfig config_;
  std::unique_ptr<av::StepwiseScheduler> scheduler_;
  std::unique_ptr<av::StepwiseTransport> transport_;
  bool connected_{false}, enable_motion_{false}, session_enabled_{false};
  bool recovery_in_progress_{false};
  double recovery_started_at_{0.0};
  bool health_status_published_{false}, published_healthy_{false};
  double last_diagnostic_at_{0.0};
  std::string last_diagnostic_phase_, last_health_failure_;
  bool session_limit_reached_{false};
  bool session_limit_handoff_pending_{false};
  double session_limit_deadline_{0.0};
  std::int64_t session_limit_request_id_{0};
  std::string session_limit_handoff_error_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture session_limit_future_;
  bool enable_follow_drag_{false};
  bool enable_follow_traction_{false};
  double telemetry_timeout_s_{0.5};
  unsigned last_command_mode_{0};
  double mode_at_{0.0};
  std::string connection_error_, last_sdk_result_, last_motion_failure_, pose_frame_;
  std::vector<std::string> names_;
  std::uint64_t intent_sequence_{0}, resume_after_intent_{0}, published_sequence_{0};
  std::uint64_t handled_controller_error_{0};
  double intent_at_{0.0}, state_fence_at_{0.0};
  double last_tick_at_{0.0}, maximum_tick_gap_s_{0.0};
  double maximum_session_travel_{0.0};
  double session_orientation_tolerance_rad_{0.03};
  std::array<double, 3> session_origin_{};
  std::array<double, 4> session_orientation_{0, 0, 0, 1};
  av::StepwiseState previous_phase_{av::StepwiseState::DISABLED};
  std::optional<av::TransportSample> previous_sample_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<fr_traction::msg::EndEffectorState>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr health_pub_, idle_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr identity_pub_, phase_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr session_limit_client_;
  rclcpp::Subscription<fr_traction::msg::TractionCommand>::SharedPtr mode_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_sub_;
  rclcpp::Service<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_, recover_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<AuboV4StepwiseNode>());
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(rclcpp::get_logger("aubo_v4_stepwise"), "%s", exception.what());
    rclcpp::shutdown();
    return 2;
  }
  rclcpp::shutdown();
  return 0;
}
