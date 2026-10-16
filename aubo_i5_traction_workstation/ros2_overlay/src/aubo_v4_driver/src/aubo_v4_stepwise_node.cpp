#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <fr_traction/msg/end_effector_state.hpp>
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
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include "aubo_v4_driver/stepwise_scheduler.hpp"
#include "aubo_v4_driver/stepwise_transport.hpp"

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
    av::StepwiseConfig config;
    config.maximum_step_m = declare_parameter("maximum_step_m", 0.0005);
    config.maximum_speed_mps = declare_parameter("maximum_speed_mps", 0.002);
    config.horizon_s = declare_parameter("planning_horizon_s", 0.1);
    config.feedback_timeout_s = declare_parameter("feedback_timeout_s", 0.5);
    config.command_timeout_s = declare_parameter("command_timeout_s", 0.25);
    config.ack_timeout_s = declare_parameter("ack_timeout_s", 2.0);
    config.completion_timeout_s = declare_parameter("completion_timeout_s", 5.0);
    config.settling_time_s = declare_parameter("settling_time_s", 0.1);
    config.arrival_tolerance_m = declare_parameter("arrival_tolerance_m", 0.00005);
    config.quiet_speed_mps = declare_parameter("quiet_speed_mps", 0.0002);
    if (config.maximum_step_m > 0.001 || config.maximum_speed_mps > 0.005 ||
      config.horizon_s > 0.5 || config.feedback_timeout_s > 1.0)
    {
      throw std::invalid_argument("stepwise prototype step/speed/timing limits exceeded");
    }
    config_ = config;
    scheduler_ = std::make_unique<av::StepwiseScheduler>(config_);
    pose_frame_ = declare_parameter<std::string>("pose_frame_id", "base_link");
    maximum_session_travel_ = declare_parameter("maximum_session_travel_m", 0.05);
    if (!std::isfinite(maximum_session_travel_) || maximum_session_travel_ <= 0.0 ||
      maximum_session_travel_ > 0.5)
    {
      throw std::invalid_argument("maximum_session_travel_m must be in (0,0.5]");
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
      "/diagnostics", rclcpp::QoS(
        1).transient_local());
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
        if (session_enabled_) {
          response->ok = !scheduler_->fault_pending();
          return;
        }
        response->ok = connected_ && enable_motion_ &&
        scheduler_->set_enabled(true, av::steady_seconds());
        session_enabled_ = response->ok;
        if (response->ok) {
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
        response->message = "Stop requested; physical stop is not yet confirmed. Node restart required.";
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
    if (!connected_) {publish_status(av::steady_seconds()); return;}
    if (count_publishers("/aubo_i5/driver_identity") > 1 ||
      count_publishers("/controller_manager/command_cart_vel") > 1)
    {
      session_enabled_ = false;
      scheduler_->request_stop(
        av::steady_seconds(), "multiple robot drivers or motion-command publishers", true);
    }
    const auto sample = transport_->sample();
    while (const auto result = transport_->take_result()) {
      last_sdk_result_ = result->detail;
      const auto time = av::steady_seconds();
      state_fence_at_ = time;
      if (result->stop) {
        scheduler_->acknowledge_stop(result->epoch, result->success, time);
      } else {
        scheduler_->acknowledge(result->id, result->epoch, result->success, time);
      }
    }
    const auto time = av::steady_seconds();
    if (sample.feedback.sequence > 0) {
      auto feedback = sample.feedback;
      feedback.healthy = feedback.healthy &&
        time - sample.diagnosis_at <= config_.feedback_timeout_s &&
        time - sample.robot_state_at <= config_.feedback_timeout_s;
      feedback.stopped = feedback.stopped && sample.robot_state_at >= state_fence_at_;
      if (session_enabled_ && !feedback.healthy) {
        scheduler_->request_stop(time, "unhealthy or stale SDK diagnosis/state", true);
      }
      if (session_enabled_) {
        double quaternion_dot = 0.0;
        for (std::size_t index = 0; index < 4; ++index) {
          quaternion_dot += feedback.orientation[index] * session_orientation_[index];
        }
        const double angle = 2.0 * std::acos(std::min(1.0, std::abs(quaternion_dot)));
        if (!std::isfinite(angle) || angle > 0.01) {
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
      if (const auto command = scheduler_->take_command(av::steady_seconds())) {
        double squared_travel = 0.0;
        for (std::size_t index = 0; index < 3; ++index) {
          const double delta = command->target[index] - session_origin_[index];
          squared_travel += delta * delta;
        }
        if (std::sqrt(squared_travel) > maximum_session_travel_) {
          scheduler_->request_stop(
            av::steady_seconds(), "stepwise session travel bound reached", true);
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
    if (connected_) {
      const auto sample = transport_->sample();
      healthy = sample.feedback.healthy &&
        time - sample.feedback.received_at <= config_.feedback_timeout_s &&
        time - sample.diagnosis_at <= config_.feedback_timeout_s &&
        time - sample.robot_state_at <= config_.feedback_timeout_s && !scheduler_->fault_pending();
    }
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
    identity.data = connected_ && healthy ? "AUBO_I5_V4" : "";
    identity_pub_->publish(identity);
    std_msgs::msg::String phase;
    phase.data = state_name(scheduler_->state());
    phase_pub_->publish(phase);
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
    add("epoch", std::to_string(scheduler_->epoch()));
    add("last_sdk_result", last_sdk_result_);
    add("fault_reason", scheduler_->fault_reason());
    add("planning_horizon_s", std::to_string(config_.horizon_s));
    add("maximum_step_m", std::to_string(config_.maximum_step_m));
    add("arrival_tolerance_m", std::to_string(config_.arrival_tolerance_m));
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    array.status.push_back(status);
    diagnostic_pub_->publish(array);
  }

  av::StepwiseConfig config_;
  std::unique_ptr<av::StepwiseScheduler> scheduler_;
  std::unique_ptr<av::StepwiseTransport> transport_;
  bool connected_{false}, enable_motion_{false}, session_enabled_{false};
  std::string connection_error_, last_sdk_result_, pose_frame_;
  std::vector<std::string> names_;
  std::uint64_t intent_sequence_{0}, resume_after_intent_{0}, published_sequence_{0};
  double intent_at_{0.0}, state_fence_at_{0.0};
  double maximum_session_travel_{0.05};
  std::array<double, 3> session_origin_{};
  std::array<double, 4> session_orientation_{0, 0, 0, 1};
  av::StepwiseState previous_phase_{av::StepwiseState::DISABLED};
  std::optional<av::TransportSample> previous_sample_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<fr_traction::msg::EndEffectorState>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr health_pub_, idle_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr identity_pub_, phase_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_sub_;
  rclcpp::Service<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
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
