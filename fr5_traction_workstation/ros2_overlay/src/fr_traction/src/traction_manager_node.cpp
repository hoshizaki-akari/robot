#include "fr_traction/traction_math.hpp"
#include "fr_traction/direction_correction.hpp"
#include "fr_traction/traction_safety.hpp"
#include "fr_traction/traction_state_machine.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <future>
#include <iomanip>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "fairino_msgs/msg/pose_twist.hpp"
#include "fr_traction/msg/traction_command.hpp"
#include "fr_traction/msg/traction_history.hpp"
#include "fr_traction/msg/traction_record_summary.hpp"
#include "fr_traction/msg/traction_status.hpp"
#include "fr_traction/srv/set_target_force.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace fr_traction
{

class TractionManagerNode final : public rclcpp::Node
{
public:
  TractionManagerNode()
  : Node("traction_manager"), force_filter_(5.0)
  {
    control_rate_hz_ = declare_parameter("control_rate_hz", 100.0);
    status_rate_hz_ = declare_parameter("status_rate_hz", 20.0);
    force_filter_cutoff_hz_ = declare_parameter("force_filter_cutoff_hz", 5.0);
    direction_correction_mode_name_ = declare_parameter(
      "direction_correction_mode", std::string("active"));
    direction_fast_cutoff_hz_ = declare_parameter("direction_fast_cutoff_hz", 4.0);
    direction_slow_cutoff_hz_ = declare_parameter("direction_slow_cutoff_hz", 1.0);
    direction_robust_window_size_ = declare_parameter("direction_robust_window_size", 11);
    direction_minimum_force_n_ = declare_parameter("direction_minimum_force_n", 0.5);
    direction_recovery_confirm_s_ = declare_parameter("direction_recovery_confirm_s", 0.30);
    direction_change_confirm_s_ = declare_parameter("direction_change_confirm_s", 0.20);
    direction_settling_s_ = declare_parameter("direction_settling_s", 0.50);
    direction_candidate_max_dispersion_deg_ = declare_parameter(
      "direction_candidate_max_dispersion_deg", 8.0);
    direction_ambiguity_timeout_s_ = declare_parameter("direction_ambiguity_timeout_s", 8.0);
    direction_tracking_gain_s_inv_ = declare_parameter("direction_tracking_gain_s_inv", 5.0);
    direction_tracking_max_rate_deg_s_ = declare_parameter(
      "direction_tracking_max_rate_deg_s", 180.0);
    direction_follow_speed_gain_mps_per_rad_ = declare_parameter(
      "direction_follow_speed_gain_mps_per_rad", 0.020);
    direction_correction_max_speed_mps_ = declare_parameter(
      "direction_correction_max_speed_mps", 0.020);
    direction_follow_max_acceleration_mps2_ = declare_parameter(
      "direction_follow_max_acceleration_mps2", 0.10);
    // Use a two-stage command ramp: move quickly through the large error, then
    // slow down inside the final window so the compliant rope is not hit with
    // a uniform force step all the way to the target.
    target_ramp_fast_nps_ = declare_parameter("target_ramp_fast_nps", 3.0);
    target_ramp_slow_nps_ = declare_parameter("target_ramp_slow_nps", 0.5);
    target_ramp_slow_window_n_ = declare_parameter("target_ramp_slow_window_n", 1.0);
    pretension_detect_n_ = declare_parameter("pretension_detect_n", 1.0);
    pretension_target_n_ = declare_parameter("pretension_target_n", 3.0);
    calibration_min_force_n_ = declare_parameter("calibration_min_force_n", 0.5);
    pretension_speed_mps_ = declare_parameter("pretension_speed_mps", 0.002);
    pretension_timeout_s_ = declare_parameter("pretension_timeout_s", 10.0);
    pretension_max_travel_m_ = declare_parameter("pretension_max_travel_m", 0.020);
    calibration_window_s_ = declare_parameter("calibration_window_s", 1.0);
    calibration_min_samples_ = declare_parameter("calibration_min_samples", 80);
    calibration_max_angle_p95_deg_ = declare_parameter("calibration_max_angle_p95_deg", 15.0);
    target_force_min_n_ = declare_parameter("target_force_min_n", 1.0);
    target_force_max_n_ = declare_parameter("target_force_max_n", 20.0);
    validated_target_max_n_ = declare_parameter("validated_target_max_n", 20.0);
    force_tolerance_n_ = declare_parameter("force_tolerance_n", 1.0);
    force_deadband_n_ = declare_parameter("force_deadband_n", 0.15);
    // The timeout now covers the direct, low-speed position return. It is not
    // a force-unloading timeout because RELEASING no longer runs force control.
    release_timeout_s_ = declare_parameter("release_timeout_s", 60.0);
    axial_travel_limit_m_ = declare_parameter("axial_travel_limit_m", 0.050);
    wrench_timeout_s_ = declare_parameter("wrench_timeout_s", 0.10);
    ee_state_timeout_s_ = declare_parameter("ee_state_timeout_s", 0.20);
    motion_pause_timeout_s_ = declare_parameter("motion_pause_timeout_s", 0.10);
    ui_heartbeat_timeout_s_ = declare_parameter("ui_heartbeat_timeout_s", 2.0);
    controller_health_timeout_s_ = declare_parameter("controller_health_timeout_s", 0.20);
    require_ui_heartbeat_ = declare_parameter("require_ui_heartbeat", false);
    wrench_topic_ = declare_parameter(
      "wrench_topic", std::string("/force_torque_sensor_broadcaster/wrench"));
    ee_state_topic_ = declare_parameter(
      "ee_state_topic", std::string("/cartesian_velocity_controller/ee_state"));
    joint_state_topic_ = declare_parameter("joint_state_topic", std::string("/joint_states"));
    command_topic_ = declare_parameter("command_topic", std::string("/traction/command"));
    controller_health_topic_ = declare_parameter(
      "controller_health_topic", std::string("/traction_controller/healthy"));
    hardware_health_topic_ = declare_parameter(
      "hardware_health_topic", std::string("/controller_manager/healthy"));
    velocity_command_topic_ = declare_parameter(
      "velocity_command_topic", std::string("/traction/controller_velocity_cmd"));
    corrected_wrench_topic_ = declare_parameter(
      "corrected_wrench_topic", std::string("/traction/corrected_wrench"));
    ui_heartbeat_topic_ = declare_parameter(
      "ui_heartbeat_topic", std::string("/traction/ui_heartbeat"));
    slack_calibration_topic_ = declare_parameter(
      "slack_calibration_topic", std::string("/traction/slack_calibration"));
    pretraction_return_service_name_ = declare_parameter(
      "pretraction_return_service", std::string("/traction/return_pretraction_pose"));
    pretraction_return_tolerance_m_ = declare_parameter(
      "pretraction_return_tolerance_m", 0.0015);
    expected_wrench_frame_ = declare_parameter("expected_wrench_frame", std::string("base_link"));
    data_directory_ = declare_parameter("data_directory", std::string("debug/traction_sessions"));
    switch_service_name_ = declare_parameter(
      "controller_manager_switch_service", std::string("/controller_manager/switch_controller"));

    validate_parameters();
    SafetyLimits safety_limits;
    safety_limits.axial_travel_m = axial_travel_limit_m_;
    safety_monitor_.set_limits(safety_limits);
    force_filter_.set_cutoff(force_filter_cutoff_hz_);
    std::filesystem::create_directories(data_directory_);

    command_publisher_ = create_publisher<msg::TractionCommand>(
      command_topic_, rclcpp::QoS(10).reliable());
    status_publisher_ = create_publisher<msg::TractionStatus>(
      "/traction/status", rclcpp::QoS(10).reliable());
    // Keep the safety-critical corrected force stream reliable end to end.
    corrected_wrench_publisher_ = create_publisher<geometry_msgs::msg::WrenchStamped>(
      corrected_wrench_topic_, rclcpp::QoS(10).reliable());
    history_publisher_ = create_publisher<msg::TractionHistory>(
      "/traction/history", rclcpp::QoS(1).reliable().transient_local());
    slack_calibration_publisher_ = create_publisher<std_msgs::msg::Bool>(
      slack_calibration_topic_, rclcpp::QoS(1).reliable().transient_local());
    switch_client_ = create_client<controller_manager_msgs::srv::SwitchController>(
      switch_service_name_);
    pretraction_return_client_ = create_client<std_srvs::srv::Trigger>(
      pretraction_return_service_name_);

    wrench_subscription_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      // The direct FR5 driver publishes this stream reliably. Match that
      // QoS instead of downgrading the safety-critical feedback to best effort.
      wrench_topic_, rclcpp::QoS(10).reliable(),
      [this](const geometry_msgs::msg::WrenchStamped::SharedPtr message) {on_wrench(*message);});
    ee_state_subscription_ = create_subscription<fairino_msgs::msg::PoseTwist>(
      ee_state_topic_, rclcpp::QoS(10).reliable(),
      [this](const fairino_msgs::msg::PoseTwist::SharedPtr message) {on_ee_state(*message);});
    joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_, rclcpp::QoS(10).reliable(),
      [this](const sensor_msgs::msg::JointState::SharedPtr message) {on_joint_state(*message);});
    health_subscription_ = create_subscription<std_msgs::msg::Bool>(
      controller_health_topic_, rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::Bool::SharedPtr message) {
        controller_healthy_ = message->data;
        last_controller_health_at_ = now();
        last_controller_health_steady_at_ = std::chrono::steady_clock::now();
      });
    hardware_health_subscription_ = create_subscription<std_msgs::msg::Bool>(
      hardware_health_topic_, rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::Bool::SharedPtr message) {
        hardware_healthy_ = message->data;
        last_hardware_health_at_ = now();
        last_hardware_health_steady_at_ = std::chrono::steady_clock::now();
      });
    velocity_subscription_ = create_subscription<std_msgs::msg::Float64>(
      velocity_command_topic_, rclcpp::QoS(10).reliable(),
      [this](const std_msgs::msg::Float64::SharedPtr message) {
        if (std::isfinite(message->data)) {velocity_command_mps_ = message->data;}
      });
    heartbeat_subscription_ = create_subscription<std_msgs::msg::Empty>(
      ui_heartbeat_topic_, rclcpp::QoS(10).reliable(),
      [this](const std_msgs::msg::Empty::SharedPtr) {
        last_ui_heartbeat_at_ = now();
        last_ui_heartbeat_steady_at_ = std::chrono::steady_clock::now();
      });

    prepare_service_ = create_service<std_srvs::srv::Trigger>(
      "/traction/prepare",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
      const std_srvs::srv::Trigger::Response::SharedPtr response) {
        handle_prepare(response);
      });
    calibrate_service_ = create_service<std_srvs::srv::Trigger>(
      "/traction/calibrate_direction",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
      const std_srvs::srv::Trigger::Response::SharedPtr response) {
        handle_calibrate(response);
      });
    start_service_ = create_service<std_srvs::srv::Trigger>(
      "/traction/start",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
      const std_srvs::srv::Trigger::Response::SharedPtr response) {
        handle_start(response);
      });
    stop_service_ = create_service<std_srvs::srv::Trigger>(
      "/traction/stop",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
      const std_srvs::srv::Trigger::Response::SharedPtr response) {
        handle_stop(response);
      });
    emergency_service_ = create_service<std_srvs::srv::Trigger>(
      "/traction/emergency_stop",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
      const std_srvs::srv::Trigger::Response::SharedPtr response) {
        handle_emergency(response);
      });
    reset_fault_service_ = create_service<std_srvs::srv::Trigger>(
      "/traction/reset_fault",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
      const std_srvs::srv::Trigger::Response::SharedPtr response) {
        handle_reset_fault(response);
      });
    target_force_service_ = create_service<srv::SetTargetForce>(
      "/traction/set_target_force",
      [this](const srv::SetTargetForce::Request::SharedPtr request,
      const srv::SetTargetForce::Response::SharedPtr response) {
        handle_set_target(request, response);
      });

    load_history();
    publish_history();
    const auto control_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / control_rate_hz_));
    const auto status_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / status_rate_hz_));
    control_timer_ = create_wall_timer(control_period, [this]() {control_tick();});
    status_timer_ = create_wall_timer(status_period, [this]() {publish_status();});
    RCLCPP_INFO(
      get_logger(), "Traction manager initialized in INITIALIZING; waiting for live ROS2 data.");
  }

private:
  void validate_parameters()
  {
    if (direction_correction_mode_name_ != "off" &&
      direction_correction_mode_name_ != "shadow" &&
      direction_correction_mode_name_ != "active")
    {
      throw std::runtime_error(
              "direction_correction_mode must be off, shadow or active");
    }
    const bool valid = std::isfinite(control_rate_hz_) && control_rate_hz_ > 0.0 &&
      std::isfinite(status_rate_hz_) && status_rate_hz_ > 0.0 &&
      std::isfinite(force_filter_cutoff_hz_) && force_filter_cutoff_hz_ > 0.0 &&
      std::isfinite(pretension_detect_n_) && pretension_detect_n_ > 0.0 &&
      std::isfinite(pretension_target_n_) && pretension_target_n_ > pretension_detect_n_ &&
      std::isfinite(pretension_speed_mps_) && pretension_speed_mps_ > 0.0 &&
      std::isfinite(pretension_timeout_s_) && pretension_timeout_s_ > 0.0 &&
      std::isfinite(pretension_max_travel_m_) && pretension_max_travel_m_ > 0.0 &&
      std::isfinite(calibration_min_force_n_) && calibration_min_force_n_ > 0.0 &&
      std::isfinite(calibration_window_s_) && calibration_window_s_ > 0.0 &&
      calibration_min_samples_ > 0 && std::isfinite(calibration_max_angle_p95_deg_) &&
      calibration_max_angle_p95_deg_ > 0.0 && std::isfinite(target_force_min_n_) &&
      target_force_min_n_ > 0.0 && std::isfinite(target_force_max_n_) &&
      target_force_max_n_ >= target_force_min_n_ &&
      std::isfinite(validated_target_max_n_) && validated_target_max_n_ >= target_force_min_n_ &&
      validated_target_max_n_ <= target_force_max_n_ &&
      std::isfinite(target_ramp_fast_nps_) && target_ramp_fast_nps_ > 0.0 &&
      std::isfinite(target_ramp_slow_nps_) && target_ramp_slow_nps_ > 0.0 &&
      target_ramp_fast_nps_ >= target_ramp_slow_nps_ &&
      std::isfinite(target_ramp_slow_window_n_) && target_ramp_slow_window_n_ > 0.0 &&
      std::isfinite(force_tolerance_n_) &&
      std::isfinite(release_timeout_s_) && release_timeout_s_ > 0.0 &&
      std::isfinite(axial_travel_limit_m_) && axial_travel_limit_m_ > 0.0 &&
      std::isfinite(wrench_timeout_s_) && wrench_timeout_s_ > 0.0 &&
      std::isfinite(ee_state_timeout_s_) && ee_state_timeout_s_ > 0.0 &&
      std::isfinite(motion_pause_timeout_s_) && motion_pause_timeout_s_ > 0.0 &&
      motion_pause_timeout_s_ <= wrench_timeout_s_ &&
      motion_pause_timeout_s_ <= ee_state_timeout_s_ &&
      std::isfinite(ui_heartbeat_timeout_s_) && ui_heartbeat_timeout_s_ > 0.0 &&
      std::isfinite(controller_health_timeout_s_) && controller_health_timeout_s_ > 0.0 &&
      std::isfinite(pretraction_return_tolerance_m_) && pretraction_return_tolerance_m_ > 0.0;
    const bool direction_valid =
      std::isfinite(direction_fast_cutoff_hz_) && direction_fast_cutoff_hz_ > 0.0 &&
      std::isfinite(direction_slow_cutoff_hz_) && direction_slow_cutoff_hz_ > 0.0 &&
      direction_slow_cutoff_hz_ <= direction_fast_cutoff_hz_ &&
      direction_robust_window_size_ >= 3 && direction_robust_window_size_ <= 101 &&
      std::isfinite(direction_minimum_force_n_) && direction_minimum_force_n_ > 0.0 &&
      std::isfinite(direction_recovery_confirm_s_) && direction_recovery_confirm_s_ > 0.0 &&
      std::isfinite(direction_change_confirm_s_) && direction_change_confirm_s_ > 0.0 &&
      std::isfinite(direction_settling_s_) && direction_settling_s_ > 0.0 &&
      std::isfinite(direction_candidate_max_dispersion_deg_) &&
      direction_candidate_max_dispersion_deg_ > 0.0 &&
      direction_candidate_max_dispersion_deg_<90.0 &&
        std::isfinite(direction_ambiguity_timeout_s_) && direction_ambiguity_timeout_s_>0.0 &&
      std::isfinite(direction_tracking_gain_s_inv_) && direction_tracking_gain_s_inv_ > 0.0 &&
      std::isfinite(direction_tracking_max_rate_deg_s_) &&
      direction_tracking_max_rate_deg_s_ > 0.0 &&
      std::isfinite(direction_follow_speed_gain_mps_per_rad_) &&
      direction_follow_speed_gain_mps_per_rad_ > 0.0 &&
      std::isfinite(direction_correction_max_speed_mps_) &&
      direction_correction_max_speed_mps_ > 0.0 &&
      std::isfinite(direction_follow_max_acceleration_mps2_) &&
      direction_follow_max_acceleration_mps2_ > 0.0;
    if (!valid || !direction_valid) {
      throw std::runtime_error("invalid traction manager safety or timing parameter");
    }
  }

  static Vec3 force_from_message(const geometry_msgs::msg::WrenchStamped & message)
  {
    return {message.wrench.force.x, message.wrench.force.y, message.wrench.force.z};
  }

  static Vec3 point_from_message(const fairino_msgs::msg::PoseTwist & message)
  {
    return {message.pose.position.x, message.pose.position.y, message.pose.position.z};
  }

  static Vec3 direction_from_message(const msg::TractionCommand & message)
  {
    return {message.locked_direction_base.x, message.locked_direction_base.y,
      message.locked_direction_base.z};
  }

  void on_wrench(const geometry_msgs::msg::WrenchStamped & message)
  {
    raw_wrench_ = force_from_message(message);
    if (!wrench_baseline_valid_ && finite(raw_wrench_)) {
      wrench_baseline_ = raw_wrench_;
      wrench_baseline_valid_ = true;
    }
    latest_wrench_ = raw_wrench_ - wrench_baseline_;
    if (corrected_wrench_publisher_) {
      auto corrected = message;
      corrected.wrench.force.x = latest_wrench_.x;
      corrected.wrench.force.y = latest_wrench_.y;
      corrected.wrench.force.z = latest_wrench_.z;
      corrected_wrench_publisher_->publish(corrected);
    }
    latest_torque_finite_ = std::isfinite(message.wrench.torque.x) &&
      std::isfinite(message.wrench.torque.y) && std::isfinite(message.wrench.torque.z);
    wrench_frame_valid_ = message.header.frame_id == expected_wrench_frame_;
    wrench_valid_ = finite(latest_wrench_) && latest_torque_finite_ && wrench_frame_valid_;
    last_wrench_at_ = now();
    last_wrench_steady_at_ = std::chrono::steady_clock::now();
    ++wrench_sequence_;
  }

  void on_ee_state(const fairino_msgs::msg::PoseTwist & message)
  {
    latest_ee_position_ = point_from_message(message);
    latest_ee_linear_velocity_ = {
      message.twist.linear.x, message.twist.linear.y, message.twist.linear.z};
    latest_ee_angular_velocity_ = {
      message.twist.angular.x, message.twist.angular.y, message.twist.angular.z};
    ee_valid_ = finite(latest_ee_position_) && finite(latest_ee_linear_velocity_) &&
      finite(latest_ee_angular_velocity_);
    last_ee_at_ = now();
    last_ee_steady_at_ = std::chrono::steady_clock::now();
  }

  void on_joint_state(const sensor_msgs::msg::JointState & message)
  {
    joint_state_valid_ = message.velocity.size() >= 6;
    latest_joint_speed_norm_ = 0.0;
    if (!joint_state_valid_) {return;}
    for (std::size_t index = 0; index < 6; ++index) {
      if (!std::isfinite(message.velocity[index])) {
        joint_state_valid_ = false;
        break;
      }
      latest_joint_speed_norm_ = std::hypot(latest_joint_speed_norm_, message.velocity[index]);
    }
    last_joint_state_at_ = now();
    last_joint_state_steady_at_ = std::chrono::steady_clock::now();
  }

  bool fresh(const std::chrono::steady_clock::time_point & stamp, double timeout_s) const
  {
    if (stamp.time_since_epoch().count() == 0) {return false;}
    const auto age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - stamp).count();
    return std::isfinite(age) && age >= 0.0 && age <= timeout_s;
  }

  double age_seconds(const std::chrono::steady_clock::time_point & stamp) const
  {
    if (stamp.time_since_epoch().count() == 0) {
      return std::numeric_limits<double>::infinity();
    }
    const double age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - stamp).count();
    return std::isfinite(age) && age >= 0.0 ? age : std::numeric_limits<double>::infinity();
  }

  bool live_wrench() const
  {
    return wrench_valid_ && fresh(last_wrench_steady_at_, wrench_timeout_s_);
  }
  bool live_ee() const {return ee_valid_ && fresh(last_ee_steady_at_, ee_state_timeout_s_);}
  bool motion_feedback_fresh() const
  {
    return wrench_valid_ && ee_valid_ &&
           fresh(last_wrench_steady_at_, motion_pause_timeout_s_) &&
           fresh(last_ee_steady_at_, motion_pause_timeout_s_);
  }
  bool live_joint() const
  {
    return joint_state_valid_ && fresh(last_joint_state_steady_at_, ee_state_timeout_s_);
  }
  bool live_controller() const
  {
    return controller_healthy_ &&
           fresh(last_controller_health_steady_at_, controller_health_timeout_s_);
  }
  bool live_hardware() const
  {
    return hardware_healthy_ &&
           fresh(last_hardware_health_steady_at_, controller_health_timeout_s_);
  }

  bool readiness_check(std::string & reason) const
  {
    if (!live_controller()) {reason = "CONTROLLER_NOT_HEALTHY"; return false;}
    if (!live_hardware()) {reason = "FR5_HARDWARE_NOT_HEALTHY"; return false;}
    if (!live_wrench()) {
      reason = wrench_frame_valid_ ? "WRENCH_NOT_FRESH" : "WRENCH_FRAME_INVALID"; return false;
    }
    if (!live_joint()) {reason = "JOINT_STATE_NOT_FRESH"; return false;}
    if (!finite(latest_wrench_)) {
      reason = "NONFINITE_INPUT"; return false;
    }
    return true;
  }

  bool transition(TractionState next)
  {
    const auto current = state_machine_.state();
    if (!state_machine_.transition(next)) {
      RCLCPP_ERROR(
        get_logger(), "Illegal traction state transition %s -> %s.",
        state_name(current), state_name(next));
      return false;
    }
    RCLCPP_INFO(get_logger(), "Traction state: %s -> %s", state_name(current), state_name(next));
    return true;
  }

  std::string state_and_allowed(const std::string & allowed) const
  {
    return "Current state is " + std::string(state_name(state_machine_.state())) +
           "; allowed operation: " + allowed + ".";
  }

  void handle_prepare(const std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    const auto state = state_machine_.state();
    if (state != TractionState::READY && state != TractionState::COMPLETED &&
      state != TractionState::MANUAL_SETUP && state != TractionState::DIRECTION_LOCKED &&
      state != TractionState::FAULT && state != TractionState::EMERGENCY_STOP)
    {
      response->success = false;
      response->message = state_and_allowed("initial calibration while traction is stopped");
      return;
    }
    std::string reason;
    // This service is explicitly the operator's way to replace a stale
    // baseline after loosening the band. The previous baseline may therefore
    // report a large force even though the current physical state is slack;
    // requiring the old baseline check here would make the recovery button
    // permanently unavailable. The operator confirms that the band is slack
    // by pressing this button; the running traction safety checks remain
    // unchanged.
    const bool live_inputs = live_controller() && live_hardware() && live_wrench() && live_joint();
    if (!live_inputs || !finite(raw_wrench_)) {
      reason = "LIVE_INPUTS_NOT_READY";
    }
    if (!reason.empty()) {
      response->success = false;
      response->message = "Prepare rejected: " + reason + ".";
      return;
    }
    if ((state == TractionState::COMPLETED || state == TractionState::FAULT ||
      state == TractionState::EMERGENCY_STOP) && !transition(TractionState::READY))
    {
      response->success = false;
      response->message = "Initial calibration could not reset the stopped state.";
      return;
    }
    if (wrench_baseline_valid_) {
      // Re-capture the complete mounted tool/sensor load while the operator
      // confirms that the band is slack. This is the software fallback for
      // controllers that reject FT_SetZero and also removes tool gravity.
      wrench_baseline_ = raw_wrench_;
      latest_wrench_ = {};
      filtered_wrench_ = {};
    }
    std_msgs::msg::Bool slack_calibration;
    slack_calibration.data = true;
    slack_calibration_publisher_->publish(slack_calibration);
    // Every prepare starts a clean record.  The previous implementation only
    // closed a record when prepare was repeated from MANUAL_SETUP.  Repeating
    // prepare from DIRECTION_LOCKED left record_stream_ open, so begin_session()
    // attempted to open a second file on the same stream and terminated the
    // manager.  Close any active session before resetting its state.
    if (session_active_) {
      stop_reason_ = state == TractionState::MANUAL_SETUP ?
        "PREPARE_RESTARTED_AFTER_BASELINE_RESET" : "INITIAL_CALIBRATION_RESTARTED";
      finalize_session();
    }
    reset_session_state();
    begin_session();
    if (state != TractionState::MANUAL_SETUP) {
      transition(TractionState::MANUAL_SETUP);
    }
    response->success = true;
    response->message =
      "Initial calibration completed. Move the FR5 by teach pendant, then confirm direction.";
  }

  void handle_calibrate(const std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    if (state_machine_.state() != TractionState::MANUAL_SETUP) {
      response->success = false;
      response->message = state_and_allowed("calibrate_direction only from MANUAL_SETUP");
      return;
    }
    if (!live_wrench() || !live_joint() || !live_controller() || !live_hardware()) {
      response->success = false;
      response->message =
        "Calibration rejected: live Wrench, joint state and controller health are required.";
      return;
    }
    if (norm(filtered_wrench_) < calibration_min_force_n_) {
      response->success = false;
      response->message = "Calibration rejected: maintain at least 0.5 N manual tension first.";
      return;
    }
    if (latest_joint_speed_norm_ > 0.02) {
      response->success = false;
      response->message = "Calibration rejected: stop the teach pendant and hold the flange still.";
      return;
    }
    if (!normalize(filtered_wrench_, temporary_direction_)) {
      response->success = false;
      response->message = "Calibration rejected: the measured force direction is invalid.";
      return;
    }
    temporary_direction_valid_ = true;
    calibration_requested_ = false;
    calibration_samples_.clear();
    calibration_started_at_ = now();
    last_calibration_wrench_sequence_ = wrench_sequence_;
    transition(TractionState::CALIBRATING);
    response->success = true;
    response->message = "Direction calibration started. Hold the manually tensioned band still.";
  }

  void handle_start(const std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    if (state_machine_.state() != TractionState::DIRECTION_LOCKED) {
      response->success = false;
      response->message = state_and_allowed("start only from DIRECTION_LOCKED");
      return;
    }
    if (!direction_locked_ || !live_wrench() || !live_controller() || !live_hardware()) {
      response->success = false;
      response->message =
        "Start rejected: locked direction, Wrench and controller health are required.";
      return;
    }
    if (!target_in_range(target_force_n_)) {
      response->success = false;
      response->message = "Start rejected: target must be between 1 N and 20 N.";
      return;
    }
    if (!target_force_configured_) {
      response->success = false;
      response->message =
        "Start rejected: set the target force after direction lock before starting.";
      return;
    }
    if (target_force_n_ > validated_target_max_n_) {
      response->success = false;
      response->message = "Start rejected: this hardware stage is validated only through " +
        std::to_string(validated_target_max_n_) + " N.";
      return;
    }
    if (controller_start_pending_) {
      response->success = false;
      response->message = "Start is already waiting for the Cartesian controller handoff.";
      return;
    }
    if (!request_controller_start()) {
      response->success = false;
      response->message = "Start rejected: Cartesian controller could not be activated.";
      return;
    }
    current_command_target_n_ = std::clamp(current_metrics().actual_force_n, 0.0, target_force_n_);
    response->success = true;
    response->message =
      "Cartesian handoff requested; traction will start after controller confirmation.";
  }

  void handle_stop(const std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    if (state_machine_.state() != TractionState::TRACTION) {
      response->success = false;
      response->message = state_and_allowed("stop only from TRACTION");
      return;
    }
    transition(TractionState::RELEASING);
    release_started_at_ = now();
    controller_stop_requested_ = false;
    pretraction_return_call_pending_ = false;
    pretraction_return_requested_ = false;
    pretraction_return_failed_ = false;
    // The logged control target is zero from the first release sample onward:
    // this makes it explicit that the force controller is no longer driving
    // the return motion.
    current_command_target_n_ = 0.0;
    response->success = true;
    response->message = "牵引已停止，已退出力控，正在返回牵引起始位置。";
  }

  void handle_emergency(const std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    if (state_machine_.state() != TractionState::EMERGENCY_STOP) {
      transition(TractionState::EMERGENCY_STOP);
    }
    fault_code_ = "SOFTWARE_EMERGENCY_STOP";
    if (stop_reason_.empty()) {stop_reason_ = "SOFTWARE_EMERGENCY_STOP";}
    publish_disabled();
    request_controller_stop();
    finalize_session();
    response->success = true;
    response->message =
      "Software emergency stop latched; it does not replace the physical emergency stop.";
  }

  void handle_reset_fault(const std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    const auto state = state_machine_.state();
    if (state != TractionState::FAULT && state != TractionState::EMERGENCY_STOP) {
      response->success = false;
      response->message = state_and_allowed("reset_fault only from FAULT or EMERGENCY_STOP");
      return;
    }
    if (!live_controller() || !live_hardware() || !live_wrench() || !live_joint() ||
      latest_joint_speed_norm_ > 0.02 ||
      std::abs(velocity_command_mps_) > 1e-9)
    {
      response->success = false;
      response->message =
        "Reset rejected: controller must be reactivated, disabled, and at zero velocity with fresh data.";
      return;
    }
    if (!transition(TractionState::READY)) {
      response->success = false;
      response->message = "Reset rejected: state transition failed.";
      return;
    }
    fault_code_.clear();
    stop_reason_.clear();
    response->success = true;
    response->message = "Fault reset. Prepare must be called before another motion.";
  }

  void handle_set_target(
    const srv::SetTargetForce::Request::SharedPtr request,
    const srv::SetTargetForce::Response::SharedPtr response)
  {
    if (!target_in_range(request->target_force_n)) {
      response->success = false;
      response->message = "Target rejected: target_force_n must be in [1.0, 20.0] N.";
      return;
    }
    const auto state = state_machine_.state();
    if (state != TractionState::READY && state != TractionState::COMPLETED &&
      state != TractionState::MANUAL_SETUP && state != TractionState::DIRECTION_LOCKED)
    {
      response->success = false;
      response->message =
        state_and_allowed(
        "set_target_force from READY, COMPLETED, MANUAL_SETUP or DIRECTION_LOCKED");
      return;
    }
    target_force_n_ = request->target_force_n;
    target_force_configured_ = true;
    response->success = true;
    response->message = "Target force set to " + std::to_string(target_force_n_) + " N.";
  }

  bool target_in_range(double target) const
  {
    return std::isfinite(target) && target >= target_force_min_n_ && target <= target_force_max_n_;
  }

  void reset_session_state()
  {
    calibration_requested_ = false;
    direction_locked_ = false;
    temporary_direction_valid_ = false;
    locked_direction_ = {};
    temporary_direction_ = {};
    pretension_detection_started_at_.reset();
    pretension_started_at_.reset();
    calibration_started_at_.reset();
    release_started_at_.reset();
    calibration_samples_.clear();
    pretension_detection_samples_.clear();
    safety_monitor_.reset();
    force_filter_.reset();
    controller_start_pending_ = false;
    controller_activation_confirming_ = false;
    controller_stop_requested_ = false;
    pretraction_return_call_pending_ = false;
    pretraction_return_requested_ = false;
    pretraction_return_failed_ = false;
    pretraction_return_failure_reason_.clear();
    target_force_configured_ = false;
    if (direction_estimator_) {direction_estimator_->reset();}
    if (direction_controller_) {direction_controller_->reset();}
    direction_estimate_ = {};
    lateral_correction_result_ = {};
    lateral_correction_velocity_base_ = {};
    velocity_command_mps_ = 0.0;
    stop_reason_.clear();
    fault_code_.clear();
  }

  std::string make_session_id(const rclcpp::Time & timestamp) const
  {
    const auto milliseconds = timestamp.nanoseconds() / 1000000;
    return "session_" + std::to_string(milliseconds);
  }

  void begin_session()
  {
    session_id_ = make_session_id(now());
    session_start_at_ = now();
    session_start_position_ = latest_ee_position_;
    session_record_path_ =
      (std::filesystem::path(data_directory_) / session_id_ / "traction.csv").string();
    std::filesystem::create_directories(std::filesystem::path(session_record_path_).parent_path());
    record_stream_.open(session_record_path_, std::ios::out | std::ios::trunc);
    if (!record_stream_) {
      throw std::runtime_error("could not open traction record: " + session_record_path_);
    }
    record_stream_ << "timestamp,elapsed_s,state,target_force_n,actual_force_n,lateral_force_n,"
      "fx,fy,fz,dir_x,dir_y,dir_z,initial_dir_x,initial_dir_y,initial_dir_z,"
      "raw_dir_x,raw_dir_y,raw_dir_z,robust_dir_x,robust_dir_y,robust_dir_z,"
      "filtered_dir_x,filtered_dir_y,filtered_dir_z,"
      "candidate_dir_x,candidate_dir_y,candidate_dir_z,"
      "ee_x,ee_y,ee_z,axis_displacement_m,"
      "velocity_cmd_mps,direction_track_state,direction_error_rad,"
      "direction_fast_slow_error_rad,direction_candidate_dispersion_rad,"
      "direction_candidate_elapsed_s,direction_ambiguity_elapsed_s,"
      "direction_candidate_confirmed,direction_correction_requested_velocity_mps,"
      "direction_correction_velocity_mps,combined_velocity_mps,"
      "direction_correction_displacement_m,direction_vx,direction_vy,direction_vz,"
      "stop_reason\n";
    record_stream_ << std::setprecision(10);
    session_active_ = true;
    last_record_flush_at_ = session_start_at_;
    session_force_sum_ = 0.0;
    session_force_max_ = 0.0;
    session_sample_count_ = 0;
  }

  Vec3 active_direction() const
  {
    if (direction_locked_) {
      Vec3 tracked;
      if (normalize(direction_estimate_.tracked_direction, tracked)) {return tracked;}
      return locked_direction_;
    }
    if (temporary_direction_valid_) {return temporary_direction_;}
    return {};
  }

  ForceMetrics current_metrics() const
  {
    ForceMetrics metrics;
    const Vec3 direction = active_direction();
    if (!project_force(filtered_wrench_, direction, metrics)) {return {};}
    // The force loop controls rope tension magnitude. Lateral force is kept
    // only as the component perpendicular to the current tracked direction.
    metrics.actual_force_n = norm(filtered_wrench_);
    return metrics;
  }

  uint8_t direction_correction_command_mode() const
  {
    if (direction_correction_mode_name_ == "active") {
      return msg::TractionCommand::DIRECTION_CORRECTION_ACTIVE;
    }
    if (direction_correction_mode_name_ == "shadow") {
      return msg::TractionCommand::DIRECTION_CORRECTION_SHADOW;
    }
    return msg::TractionCommand::DIRECTION_CORRECTION_OFF;
  }

  void initialize_direction_tracking()
  {
    constexpr double kRadiansPerDegree = 3.14159265358979323846 / 180.0;
    DirectionFilterConfig filter_config;
    filter_config.fast_cutoff_hz = direction_fast_cutoff_hz_;
    filter_config.slow_cutoff_hz = direction_slow_cutoff_hz_;
    filter_config.robust_window_size = static_cast<std::size_t>(direction_robust_window_size_);
    filter_config.minimum_force_n = direction_minimum_force_n_;
    filter_config.recovery_confirm_s = direction_recovery_confirm_s_;
    filter_config.change_confirm_s = direction_change_confirm_s_;
    filter_config.settling_s = direction_settling_s_;
    filter_config.candidate_max_dispersion_rad =
      direction_candidate_max_dispersion_deg_ * kRadiansPerDegree;
    filter_config.ambiguity_timeout_s = direction_ambiguity_timeout_s_;
    filter_config.tracking_gain_s_inv = direction_tracking_gain_s_inv_;
    filter_config.tracking_max_rate_rad_s =
      direction_tracking_max_rate_deg_s_ * kRadiansPerDegree;
    direction_estimator_ = std::make_unique<DirectionEstimator>(locked_direction_, filter_config);
    direction_noise_profile_ = estimate_direction_noise(
      calibration_samples_, locked_direction_, calibration_min_force_n_);
    if (direction_noise_profile_.valid) {
      direction_estimator_->set_noise_profile(direction_noise_profile_);
    }
    AdaptiveFollowConfig follow_config;
    follow_config.speed_gain_mps_per_rad = direction_follow_speed_gain_mps_per_rad_;
    follow_config.maximum_speed_mps = direction_correction_max_speed_mps_;
    follow_config.maximum_acceleration_mps2 = direction_follow_max_acceleration_mps2_;
    direction_controller_ = std::make_unique<AdaptiveDirectionFollower>(follow_config);
    direction_estimate_ = {};
    lateral_correction_result_ = {};
    lateral_correction_velocity_base_ = {};
  }

  void update_direction_correction(double dt_s)
  {
    lateral_correction_velocity_base_ = {};
    lateral_correction_result_ = {};
    if (direction_correction_command_mode() == msg::TractionCommand::DIRECTION_CORRECTION_OFF ||
      !direction_estimator_ || !direction_controller_)
    {
      direction_estimate_ = {};
      return;
    }
    direction_estimate_ = direction_estimator_->update(filtered_wrench_, dt_s);
    const bool active = direction_correction_command_mode() ==
      msg::TractionCommand::DIRECTION_CORRECTION_ACTIVE;
    lateral_correction_result_ = direction_controller_->update(
      direction_estimate_, active, dt_s);
    if (active && lateral_correction_result_.valid) {
      lateral_correction_velocity_base_ = lateral_correction_result_.velocity_base;
    }
  }

  double axis_displacement() const
  {
    const Vec3 direction = active_direction();
    if (!finite(direction) || norm(direction) < 0.5 || !finite(latest_ee_position_)) {return 0.0;}
    Vec3 origin = session_start_position_;
    if (state_machine_.state() == TractionState::PRETENSION && pretension_start_position_valid_) {
      origin = pretension_start_position_;
    }
    return dot(latest_ee_position_ - origin, direction);
  }

  void publish_command(
    uint8_t mode,
    const Vec3 & direction,
    double target,
    uint8_t direction_correction_mode = msg::TractionCommand::DIRECTION_CORRECTION_OFF,
    const Vec3 & lateral_velocity = {})
  {
    msg::TractionCommand command;
    command.header.stamp = now();
    command.header.frame_id = "base_link";
    command.mode = mode;
    command.locked_direction_base.x = direction.x;
    command.locked_direction_base.y = direction.y;
    command.locked_direction_base.z = direction.z;
    command.target_force_n = target;
    command.direction_correction_mode = direction_correction_mode;
    command.lateral_velocity_base.x = lateral_velocity.x;
    command.lateral_velocity_base.y = lateral_velocity.y;
    command.lateral_velocity_base.z = lateral_velocity.z;
    command_publisher_->publish(command);
  }

  void publish_disabled()
  {
    publish_command(msg::TractionCommand::DISABLED, {}, 0.0);
    velocity_command_mps_ = 0.0;
  }

  void enter_fault(const std::string & code, const std::string & reason)
  {
    if (state_machine_.state() == TractionState::FAULT ||
      state_machine_.state() == TractionState::EMERGENCY_STOP ||
      state_machine_.state() == TractionState::COMPLETED)
    {
      return;
    }
    fault_code_ = code;
    if (stop_reason_.empty()) {stop_reason_ = reason;}
    transition(TractionState::FAULT);
    publish_disabled();
    request_controller_stop();
    finalize_session();
    RCLCPP_ERROR(get_logger(), "Traction fault latched: %s (%s).", code.c_str(), reason.c_str());
  }

  void request_controller_stop()
  {
    controller_activation_confirming_ = false;
    controller_start_pending_ = false;
    if (!switch_client_ || !switch_client_->service_is_ready()) {
      RCLCPP_ERROR(
        get_logger(), "Controller switch service is unavailable during software emergency stop.");
      return;
    }
    auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    request->stop_controllers.push_back("cartesian_velocity_controller");
    request->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
    request->start_asap = false;
    request->activate_asap = false;
    switch_client_->async_send_request(request);
  }

  bool request_controller_start()
  {
    if (!switch_client_ || !switch_client_->service_is_ready()) {
      RCLCPP_ERROR(get_logger(), "Controller switch service is unavailable during traction start.");
      return false;
    }
    auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    request->start_controllers.push_back("cartesian_velocity_controller");
    request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
    request->start_asap = true;
    request->activate_asap = true;
    controller_start_pending_ = true;
    switch_client_->async_send_request(
      request,
      [this](rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future) {
        controller_start_pending_ = false;
        try {
          const auto result = future.get();
          if (!result->ok) {
            enter_fault(
              "CARTESIAN_CONTROLLER_START_FAILED", "CARTESIAN_CONTROLLER_START_FAILED");
            return;
          }
          if (state_machine_.state() != TractionState::DIRECTION_LOCKED) {return;}
          controller_activation_confirming_ = true;
          controller_activation_started_at_ = now();
          RCLCPP_INFO(
            get_logger(),
            "Cartesian controller switch completed; waiting for fresh EE feedback.");
        } catch (const std::exception & error) {
          enter_fault("CARTESIAN_CONTROLLER_START_EXCEPTION", error.what());
        }
      });
    return true;
  }

  void check_safety(double monotonic_now_s)
  {
    const auto state = state_machine_.state();
    const bool needs_live_data = state == TractionState::PRETENSION ||
      state == TractionState::TRACTION || state == TractionState::RELEASING;
    if (!needs_live_data) {
      safety_monitor_.reset();
      return;
    }
    SafetySample sample;
    sample.wrench_valid = wrench_frame_valid_ && wrench_valid_;
    sample.wrench_fresh = live_wrench();
    sample.ee_fresh = live_ee();
    sample.controller_healthy = live_controller() && live_hardware();
    sample.ui_heartbeat_fresh = fresh(last_ui_heartbeat_steady_at_, ui_heartbeat_timeout_s_);
    sample.raw_wrench = latest_wrench_;
    sample.metrics = current_metrics();
    sample.axis_displacement_m = axis_displacement();
    if (!sample.wrench_fresh || !sample.ee_fresh) {
      RCLCPP_ERROR(
        get_logger(),
        "Feedback freshness failed before safety fault: wrench_valid=%s frame_valid=%s "
        "wrench_age=%.3f s ee_valid=%s ee_age=%.3f s "
        "wrench_timeout=%.3f s ee_timeout=%.3f s.",
        wrench_valid_ ? "true" : "false", wrench_frame_valid_ ? "true" : "false",
        age_seconds(last_wrench_steady_at_), ee_valid_ ? "true" : "false",
        age_seconds(last_ee_steady_at_), wrench_timeout_s_, ee_state_timeout_s_);
    }
    if (!sample.controller_healthy) {
      RCLCPP_ERROR(
        get_logger(),
        "Health check failed before safety fault: controller_value=%s controller_age=%.3f s "
        "hardware_value=%s hardware_age=%.3f s timeout=%.3f s.",
        controller_healthy_ ? "true" : "false", age_seconds(last_controller_health_steady_at_),
        hardware_healthy_ ? "true" : "false", age_seconds(last_hardware_health_steady_at_),
        controller_health_timeout_s_);
    }
    const auto fault = safety_monitor_.update(
      sample, monotonic_now_s, require_ui_heartbeat_ && state == TractionState::TRACTION);
    if (fault != SafetyFault::NONE) {
      enter_fault(SafetyMonitor::code(fault), SafetyMonitor::code(fault));
    }
  }

  void handle_manual_setup(const rclcpp::Time & current_time)
  {
    // Manual setup is intentionally passive. The operator uses the official
    // teach pendant; this node only observes force and feedback.
    (void)current_time;
  }

  void handle_pretension(const rclcpp::Time & current_time)
  {
    if (!motion_feedback_fresh()) {
      publish_disabled();
      return;
    }
    if (!temporary_direction_valid_) {
      enter_fault("PRETENSION_DIRECTION_INVALID", "PRETENSION_DIRECTION_INVALID");
      return;
    }
    publish_command(msg::TractionCommand::PRETENSION, temporary_direction_, pretension_target_n_);
    current_command_target_n_ = pretension_target_n_;
    const auto metrics = current_metrics();
    if (metrics.actual_force_n >= pretension_target_n_ - 0.3 &&
      metrics.actual_force_n <= pretension_target_n_ + 0.3)
    {
      publish_disabled();
      calibration_samples_.clear();
      calibration_started_at_ = current_time;
      last_calibration_wrench_sequence_ = wrench_sequence_;
      transition(TractionState::CALIBRATING);
      return;
    }
    if (!pretension_started_at_ ||
      (current_time - *pretension_started_at_).seconds() > pretension_timeout_s_)
    {
      enter_fault("PRETENSION_TIMEOUT", "PRETENSION_TIMEOUT");
      return;
    }
    if (std::abs(axis_displacement()) >= pretension_max_travel_m_) {
      enter_fault("PRETENSION_TRAVEL_LIMIT", "PRETENSION_TRAVEL_LIMIT");
    }
  }

  void handle_calibration(const rclcpp::Time & current_time)
  {
    publish_disabled();
    if (wrench_sequence_ != last_calibration_wrench_sequence_) {
      calibration_samples_.push_back(filtered_wrench_);
      last_calibration_wrench_sequence_ = wrench_sequence_;
    }
    if (!calibration_started_at_ ||
      (current_time - *calibration_started_at_).seconds() < calibration_window_s_)
    {
      return;
    }
    const auto result = robust_calibrate_direction(
      calibration_samples_, static_cast<std::size_t>(calibration_min_samples_),
      calibration_max_angle_p95_deg_);
    if (!result.success) {
      enter_fault("DIRECTION_CALIBRATION_FAILED", result.reason);
      return;
    }
    locked_direction_ = result.direction;
    direction_locked_ = true;
    target_force_configured_ = false;
    calibration_requested_ = false;
    temporary_direction_valid_ = false;
    initialize_direction_tracking();
    transition(TractionState::DIRECTION_LOCKED);
    RCLCPP_INFO(
      get_logger(), "Direction locked: [%.6f, %.6f, %.6f], retained=%.1f%%, p95=%.2f deg.",
      locked_direction_.x, locked_direction_.y, locked_direction_.z,
      result.retained_fraction * 100.0, result.angle_p95_deg);
  }

  void request_pretraction_return()
  {
    if (pretraction_return_call_pending_ || pretraction_return_requested_) {return;}
    if (!pretraction_return_client_ || !pretraction_return_client_->service_is_ready()) {
      return;
    }
    pretraction_return_call_pending_ = true;
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    pretraction_return_client_->async_send_request(
      request,
      [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        pretraction_return_call_pending_ = false;
        try {
          const auto result = future.get();
          if (result->success) {
            pretraction_return_requested_ = true;
            return;
          }
          // The direct driver may still be finishing ServoMoveEnd. Retry a
          // transient busy response; an absent pre-traction pose is a real
          // fault because the arm cannot safely know where to return.
          if (result->message.find("busy") != std::string::npos ||
          result->message.find("Busy") != std::string::npos)
          {
            return;
          }
          pretraction_return_failed_ = true;
          pretraction_return_failure_reason_ = result->message;
        } catch (const std::exception & error) {
          pretraction_return_call_pending_ = false;
          pretraction_return_failed_ = true;
          pretraction_return_failure_reason_ = error.what();
        }
      });
  }

  void handle_releasing(const rclcpp::Time & current_time)
  {
    if (!release_started_at_) {
      release_started_at_ = current_time;
    }
    if (!motion_feedback_fresh()) {
      publish_disabled();
      return;
    }
    const double elapsed = (current_time - *release_started_at_).seconds();
    if (pretraction_return_failed_) {
      enter_fault("PRETRACTION_RETURN_FAILED", pretraction_return_failure_reason_);
      return;
    }
    // Ending a run is position-driven. The force controller must be disabled
    // before asking the direct FR5 driver to return to the pose captured at
    // controller activation. Do not publish a RELEASING force command here:
    // that old behavior could keep the arm regulating force while waiting for
    // the measured force to fall below a threshold, causing RELEASE_TIMEOUT.
    publish_disabled();
    if (!controller_stop_requested_) {
      request_controller_stop();
      controller_stop_requested_ = true;
      controller_stop_requested_at_ = current_time;
    }
    if (!pretraction_return_requested_ &&
      (current_time - controller_stop_requested_at_).seconds() >= 0.15)
    {
      request_pretraction_return();
    }
    if (pretraction_return_requested_) {
      const double return_distance = norm(latest_ee_position_ - session_start_position_);
      if (return_distance <= pretraction_return_tolerance_m_ && latest_joint_speed_norm_ < 0.02) {
        stop_reason_ = "NORMAL_RELEASE_COMPLETED";
        transition(TractionState::COMPLETED);
        finalize_session();
        // COMPLETED is retained in history, while the live state returns to
        // the stopped/direction-locked group so the next run can reuse the
        // confirmed direction and read a fresh target from the UI.
        target_force_configured_ = false;
        transition(TractionState::DIRECTION_LOCKED);
        return;
      }
    }
    if (elapsed > release_timeout_s_) {
      enter_fault("PRETRACTION_RETURN_TIMEOUT", "PRETRACTION_RETURN_TIMEOUT");
    }
  }

  double ramped_command_target(double dt_s) const
  {
    const double remaining = std::max(0.0, target_force_n_ - current_command_target_n_);
    if (remaining <= 0.0) {return target_force_n_;}
    const double rate = remaining <= target_ramp_slow_window_n_ ?
      target_ramp_slow_nps_ : target_ramp_fast_nps_;
    return std::min(target_force_n_, current_command_target_n_ + rate * dt_s);
  }

  void control_tick()
  {
    const rclcpp::Time current_time = now();
    const double monotonic_now_s = std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
    static rclcpp::Time previous_tick(0, 0, RCL_ROS_TIME);
    double dt_s = 1.0 / control_rate_hz_;
    if (previous_tick.nanoseconds() != 0 && current_time >= previous_tick) {
      dt_s = (current_time - previous_tick).seconds();
    }
    previous_tick = current_time;
    if (!std::isfinite(dt_s) || dt_s <= 0.0 || dt_s > 0.25) {return;}
    if (live_wrench()) {
      filtered_wrench_ = force_filter_.update(latest_wrench_, dt_s);
    }

    if (state_machine_.state() == TractionState::INITIALIZING) {
      std::string reason;
      if (readiness_check(reason)) {transition(TractionState::READY);}
    }
    check_safety(monotonic_now_s);
    if (state_machine_.state() == TractionState::DIRECTION_LOCKED &&
      controller_activation_confirming_)
    {
      publish_disabled();
      const bool activation_inputs_ready = live_wrench() && live_controller() && live_ee();
      if (activation_inputs_ready) {
        controller_activation_confirming_ = false;
        session_start_position_ = latest_ee_position_;
        if (direction_estimator_) {direction_estimator_->reset();}
        if (direction_controller_) {direction_controller_->reset();}
        direction_estimate_ = {};
        lateral_correction_result_ = {};
        lateral_correction_velocity_base_ = {};
        current_command_target_n_ = std::clamp(
          current_metrics().actual_force_n, 0.0, target_force_n_);
        transition(TractionState::TRACTION);
        RCLCPP_INFO(get_logger(), "Fresh EE feedback confirmed; traction control started.");
      } else if ((current_time - controller_activation_started_at_).seconds() > 2.0) {
        controller_activation_confirming_ = false;
        enter_fault("CONTROLLER_ACTIVATION_TIMEOUT", "CONTROLLER_ACTIVATION_TIMEOUT");
      }
    }
    switch (state_machine_.state()) {
      case TractionState::MANUAL_SETUP: handle_manual_setup(current_time); break;
      case TractionState::PRETENSION: handle_pretension(current_time); break;
      case TractionState::CALIBRATING: handle_calibration(current_time); break;
      case TractionState::TRACTION:
        if (!motion_feedback_fresh()) {
          publish_disabled();
          break;
        }
        current_command_target_n_ = ramped_command_target(dt_s);
        update_direction_correction(dt_s);
        if (direction_estimate_.ambiguity_timed_out) {
          enter_fault(
            "DIRECTION_UNCONFIRMED_TIMEOUT",
            "The measured direction remained mutually inconsistent for too long.");
          break;
        }
        publish_command(
          msg::TractionCommand::TRACTION, active_direction(), current_command_target_n_,
          direction_correction_command_mode(), lateral_correction_velocity_base_);
        break;
      case TractionState::RELEASING: handle_releasing(current_time); break;
      case TractionState::INITIALIZING:
      case TractionState::READY:
      case TractionState::DIRECTION_LOCKED:
      case TractionState::COMPLETED:
      case TractionState::FAULT:
      case TractionState::EMERGENCY_STOP:
        publish_disabled();
        break;
    }
    write_record_sample(current_time);
  }

  void write_record_sample(const rclcpp::Time & current_time)
  {
    if (!session_active_ || !record_stream_) {return;}
    const auto metrics = current_metrics();
    const auto direction = active_direction();
    const Vec3 combined_velocity = direction * velocity_command_mps_ +
      lateral_correction_velocity_base_;
    const double elapsed = (current_time - session_start_at_).seconds();
    record_stream_ << std::fixed << current_time.seconds() << ',' << elapsed << ','
                   << state_name(state_machine_.state()) << ',' << current_command_target_n_ << ','
                   << metrics.actual_force_n << ',' << metrics.lateral_force_n << ','
                   << latest_wrench_.x << ',' << latest_wrench_.y << ',' << latest_wrench_.z << ','
                   << direction.x << ',' << direction.y << ',' << direction.z << ','
                   << locked_direction_.x << ',' << locked_direction_.y << ',' <<
      locked_direction_.z << ','
                   << direction_estimate_.raw_direction.x << ',' <<
      direction_estimate_.raw_direction.y << ',' << direction_estimate_.raw_direction.z << ','
                   << direction_estimate_.robust_direction.x << ',' <<
      direction_estimate_.robust_direction.y << ',' << direction_estimate_.robust_direction.z << ','
                   << direction_estimate_.fast_direction.x << ',' <<
      direction_estimate_.fast_direction.y << ',' << direction_estimate_.fast_direction.z << ','
                   << direction_estimate_.candidate_direction.x << ',' <<
      direction_estimate_.candidate_direction.y << ',' <<
      direction_estimate_.candidate_direction.z << ','
                   << latest_ee_position_.x << ',' << latest_ee_position_.y << ',' <<
      latest_ee_position_.z << ','
                   << axis_displacement() << ',' << velocity_command_mps_ << ','
                   << static_cast<int>(direction_estimate_.state) << ','
                   << direction_estimate_.fast_angle_rad << ','
                   << direction_estimate_.fast_slow_angle_rad << ','
                   << direction_estimate_.candidate_dispersion_rad << ','
                   << direction_estimate_.candidate_elapsed_s << ','
                   << direction_estimate_.ambiguity_elapsed_s << ','
                   << (direction_estimate_.candidate_confirmed ? 1 : 0) << ','
                   << lateral_correction_result_.requested_speed_mps << ','
                   << lateral_correction_result_.applied_speed_mps << ','
                   << norm(combined_velocity) << ','
                   << lateral_correction_result_.accumulated_displacement_m << ','
                   << lateral_correction_velocity_base_.x << ','
                   << lateral_correction_velocity_base_.y << ','
                   << lateral_correction_velocity_base_.z << ',' << stop_reason_ << '\n';
    session_force_sum_ += metrics.actual_force_n;
    session_force_max_ = std::max(session_force_max_, metrics.actual_force_n);
    ++session_sample_count_;
    if ((current_time - last_record_flush_at_).seconds() >= 1.0) {
      record_stream_.flush();
      last_record_flush_at_ = current_time;
    }
  }

  static builtin_interfaces::msg::Time to_builtin_time(const rclcpp::Time & value)
  {
    builtin_interfaces::msg::Time result;
    const int64_t nanoseconds = value.nanoseconds();
    int64_t seconds = nanoseconds / 1000000000LL;
    int64_t remainder = nanoseconds % 1000000000LL;
    if (remainder < 0) {
      --seconds;
      remainder += 1000000000LL;
    }
    result.sec = static_cast<int32_t>(seconds);
    result.nanosec = static_cast<uint32_t>(remainder);
    return result;
  }

  void finalize_session()
  {
    if (!session_active_) {return;}
    const auto end_time = now();
    if (record_stream_) {record_stream_.flush(); record_stream_.close();}
    msg::TractionRecordSummary summary;
    summary.header.stamp = end_time;
    summary.header.frame_id = "base_link";
    summary.session_id = session_id_;
    summary.start_time = to_builtin_time(session_start_at_);
    summary.end_time = to_builtin_time(end_time);
    summary.target_force_n = target_force_n_;
    summary.average_force_n = session_sample_count_ ==
      0 ? 0.0 : session_force_sum_ / session_sample_count_;
    summary.max_force_n = session_force_max_;
    summary.final_state = static_cast<uint8_t>(state_machine_.state());
    summary.stop_reason = stop_reason_;
    summary.record_path = session_record_path_;
    history_.summaries.push_back(summary);
    const auto sessions_path = (std::filesystem::path(data_directory_) / "sessions.csv").string();
    const bool exists = std::filesystem::exists(sessions_path);
    std::ofstream sessions(sessions_path, std::ios::out | std::ios::app);
    if (sessions) {
      if (!exists) {
        sessions << "session_id,start_time,end_time,target_force_n,average_force_n,max_force_n,"
          "final_state,stop_reason,record_path\n";
      }
      const double start_seconds = static_cast<double>(summary.start_time.sec) +
        static_cast<double>(summary.start_time.nanosec) * 1e-9;
      const double end_seconds = static_cast<double>(summary.end_time.sec) +
        static_cast<double>(summary.end_time.nanosec) * 1e-9;
      sessions << summary.session_id << ',' << start_seconds << ',' << end_seconds << ','
               << summary.target_force_n << ','
               << summary.average_force_n << ',' << summary.max_force_n << ','
               << static_cast<int>(summary.final_state) << ',' << summary.stop_reason << ','
               << summary.record_path << '\n';
    }
    session_active_ = false;
    publish_history();
  }

  void load_history()
  {
    const auto path = (std::filesystem::path(data_directory_) / "sessions.csv").string();
    std::ifstream sessions(path);
    if (!sessions) {return;}
    std::string line;
    std::getline(sessions, line);
    while (std::getline(sessions, line)) {
      std::stringstream stream(line);
      std::vector<std::string> fields;
      std::string field;
      while (std::getline(stream, field, ',')) {fields.push_back(field);}
      if (fields.size() < 9) {continue;}
      try {
        msg::TractionRecordSummary summary;
        summary.session_id = fields[0];
        const auto start_ns = static_cast<int64_t>(std::stod(fields[1]) * 1e9);
        const auto end_ns = static_cast<int64_t>(std::stod(fields[2]) * 1e9);
        summary.start_time = to_builtin_time(rclcpp::Time(start_ns, RCL_ROS_TIME));
        summary.end_time = to_builtin_time(rclcpp::Time(end_ns, RCL_ROS_TIME));
        summary.target_force_n = std::stod(fields[3]);
        summary.average_force_n = std::stod(fields[4]);
        summary.max_force_n = std::stod(fields[5]);
        summary.final_state = static_cast<uint8_t>(std::stoi(fields[6]));
        summary.stop_reason = fields[7];
        summary.record_path = fields[8];
        history_.summaries.push_back(summary);
      } catch (const std::exception &) {
        RCLCPP_WARN(get_logger(), "Ignoring malformed sessions.csv row.");
      }
    }
  }

  void publish_history()
  {
    history_.header.stamp = now();
    history_.header.frame_id = "base_link";
    history_publisher_->publish(history_);
  }

  void publish_status()
  {
    msg::TractionStatus status;
    status.header.stamp = now();
    status.header.frame_id = "base_link";
    status.state = static_cast<uint8_t>(state_machine_.state());
    status.ready = state_machine_.state() != TractionState::INITIALIZING &&
      state_machine_.state() != TractionState::FAULT &&
      state_machine_.state() != TractionState::EMERGENCY_STOP &&
      live_controller() && live_hardware();
    // The message exposes the operator-selected target, not the instantaneous
    // release/disabled command. This keeps READY/DIRECTION_LOCKED observable
    // without turning the target into zero between motion phases.
    status.target_force_n = target_force_n_;
    const auto metrics = current_metrics();
    // Before direction lock there is no valid axial projection yet. The
    // baseline-subtracted norm is shown so the operator can see the moment
    // the slack band becomes tensioned and then confirm its direction.
    status.actual_force_n = direction_locked_ ? metrics.actual_force_n : norm(filtered_wrench_);
    status.lateral_force_n = direction_locked_ ? metrics.lateral_force_n : 0.0;
    status.fx = latest_wrench_.x;
    status.fy = latest_wrench_.y;
    status.fz = latest_wrench_.z;
    const auto direction = active_direction();
    status.locked_direction_base.x = locked_direction_.x;
    status.locked_direction_base.y = locked_direction_.y;
    status.locked_direction_base.z = locked_direction_.z;
    Vec3 measured_direction;
    status.force_direction_valid = normalize(
      filtered_wrench_, measured_direction, pretension_detect_n_);
    status.measured_force_direction_base.x = measured_direction.x;
    status.measured_force_direction_base.y = measured_direction.y;
    status.measured_force_direction_base.z = measured_direction.z;
    Vec3 increase_direction = direction;
    if (norm(increase_direction) < 0.9 && status.force_direction_valid) {
      increase_direction = measured_direction;
    }
    status.increase_direction_base.x = increase_direction.x;
    status.increase_direction_base.y = increase_direction.y;
    status.increase_direction_base.z = increase_direction.z;
    status.ee_position_base.x = latest_ee_position_.x;
    status.ee_position_base.y = latest_ee_position_.y;
    status.ee_position_base.z = latest_ee_position_.z;
    status.axis_displacement_m = axis_displacement();
    status.velocity_cmd_mps = velocity_command_mps_;
    status.direction_track_state = static_cast<uint8_t>(direction_estimate_.state);
    status.direction_correction_active = direction_correction_command_mode() ==
      msg::TractionCommand::DIRECTION_CORRECTION_ACTIVE;
    status.direction_error_rad = direction_estimate_.fast_angle_rad;
    status.direction_fast_slow_error_rad = direction_estimate_.fast_slow_angle_rad;
    status.direction_entry_threshold_rad = direction_estimate_.entry_angle_rad;
    status.direction_correction_velocity_mps =
      status.direction_correction_active ? lateral_correction_result_.applied_speed_mps :
      lateral_correction_result_.requested_speed_mps;
    status.direction_correction_displacement_m =
      lateral_correction_result_.accumulated_displacement_m;
    status.lateral_correction_velocity_base.x = lateral_correction_velocity_base_.x;
    status.lateral_correction_velocity_base.y = lateral_correction_velocity_base_.y;
    status.lateral_correction_velocity_base.z = lateral_correction_velocity_base_.z;
    status.fault_code = fault_code_;
    status.stop_reason = stop_reason_;
    switch (state_machine_.state()) {
      case TractionState::INITIALIZING: status.message = "正在连接 FR5、力传感器和牵引控制器"; break;
      case TractionState::READY: status.message = "设备已就绪；牵引带松弛时点击初始校准"; break;
      case TractionState::MANUAL_SETUP: status.message = "请用法奥示教器移动，张紧后保持法兰静止"; break;
      case TractionState::CALIBRATING: status.message = "正在采集张紧方向，请保持法兰静止"; break;
      case TractionState::DIRECTION_LOCKED: status.message = "方向已锁定；可以设置目标力并开始牵引"; break;
      case TractionState::TRACTION: status.message = "恒力牵引中，正在自适应跟随受力方向"; break;
      case TractionState::RELEASING: status.message = "已停止力控，正在返回牵引起始位置"; break;
      case TractionState::COMPLETED: status.message = "牵引完成"; break;
      case TractionState::FAULT: status.message = "设备故障；确认安全后重新初始校准"; break;
      case TractionState::EMERGENCY_STOP: status.message = "已急停；确认安全后重新初始校准"; break;
      case TractionState::PRETENSION: status.message = "正在自动预张紧"; break;
    }
    status_publisher_->publish(status);
  }

  double control_rate_hz_ = 100.0;
  double status_rate_hz_ = 20.0;
  double force_filter_cutoff_hz_ = 5.0;
  std::string direction_correction_mode_name_ = "active";
  double direction_fast_cutoff_hz_ = 4.0;
  double direction_slow_cutoff_hz_ = 1.0;
  int direction_robust_window_size_ = 11;
  double direction_minimum_force_n_ = 0.5;
  double direction_recovery_confirm_s_ = 0.30;
  double direction_change_confirm_s_ = 0.20;
  double direction_settling_s_ = 0.50;
  double direction_candidate_max_dispersion_deg_ = 8.0;
  double direction_ambiguity_timeout_s_ = 8.0;
  double direction_tracking_gain_s_inv_ = 5.0;
  double direction_tracking_max_rate_deg_s_ = 180.0;
  double direction_follow_speed_gain_mps_per_rad_ = 0.020;
  double direction_correction_max_speed_mps_ = 0.020;
  double direction_follow_max_acceleration_mps2_ = 0.10;
  double pretension_detect_n_ = 1.0;
  double pretension_target_n_ = 3.0;
  double calibration_min_force_n_ = 0.5;
  double pretension_speed_mps_ = 0.002;
  double pretension_timeout_s_ = 10.0;
  double pretension_max_travel_m_ = 0.020;
  double calibration_window_s_ = 1.0;
  int calibration_min_samples_ = 80;
  double calibration_max_angle_p95_deg_ = 15.0;
  double target_force_min_n_ = 1.0;
  double target_force_max_n_ = 20.0;
  double target_ramp_fast_nps_ = 3.0;
  double target_ramp_slow_nps_ = 0.5;
  double target_ramp_slow_window_n_ = 1.0;
  double validated_target_max_n_ = 20.0;
  double force_tolerance_n_ = 1.0;
  double force_deadband_n_ = 0.15;
  double release_timeout_s_ = 60.0;
  double axial_travel_limit_m_ = 0.050;
  double wrench_timeout_s_ = 0.10;
  double ee_state_timeout_s_ = 0.20;
  double motion_pause_timeout_s_ = 0.10;
  double ui_heartbeat_timeout_s_ = 2.0;
  double controller_health_timeout_s_ = 0.20;
  bool require_ui_heartbeat_ = false;
  std::string wrench_topic_;
  std::string ee_state_topic_;
  std::string joint_state_topic_;
  std::string command_topic_;
  std::string controller_health_topic_;
  std::string hardware_health_topic_;
  std::string velocity_command_topic_;
  std::string corrected_wrench_topic_;
  std::string ui_heartbeat_topic_;
  std::string slack_calibration_topic_;
  std::string expected_wrench_frame_;
  std::string data_directory_;
  std::string switch_service_name_;
  std::string pretraction_return_service_name_;
  double pretraction_return_tolerance_m_ = 0.0015;
  std::unique_ptr<DirectionEstimator> direction_estimator_;
  std::unique_ptr<AdaptiveDirectionFollower> direction_controller_;
  DirectionNoiseProfile direction_noise_profile_;
  DirectionEstimate direction_estimate_;
  AdaptiveFollowResult lateral_correction_result_;
  Vec3 lateral_correction_velocity_base_;

  StateMachine state_machine_;
  FirstOrderLowPass force_filter_;
  SafetyMonitor safety_monitor_;
  Vec3 raw_wrench_;
  Vec3 wrench_baseline_;
  Vec3 latest_wrench_;
  Vec3 filtered_wrench_;
  Vec3 latest_ee_position_;
  Vec3 latest_ee_linear_velocity_;
  Vec3 latest_ee_angular_velocity_;
  Vec3 temporary_direction_;
  Vec3 locked_direction_;
  Vec3 session_start_position_;
  Vec3 pretension_start_position_;
  bool wrench_valid_ = false;
  bool wrench_baseline_valid_ = false;
  bool wrench_frame_valid_ = false;
  bool latest_torque_finite_ = false;
  bool ee_valid_ = false;
  bool controller_healthy_ = false;
  bool hardware_healthy_ = false;
  bool calibration_requested_ = false;
  bool temporary_direction_valid_ = false;
  bool direction_locked_ = false;
  bool pretension_start_position_valid_ = false;
  uint64_t wrench_sequence_ = 0;
  uint64_t last_calibration_wrench_sequence_ = 0;
  rclcpp::Time last_wrench_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_ee_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_controller_health_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_hardware_health_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_ui_heartbeat_at_{0, 0, RCL_ROS_TIME};
  std::chrono::steady_clock::time_point last_wrench_steady_at_{};
  std::chrono::steady_clock::time_point last_ee_steady_at_{};
  std::chrono::steady_clock::time_point last_joint_state_steady_at_{};
  std::chrono::steady_clock::time_point last_controller_health_steady_at_{};
  std::chrono::steady_clock::time_point last_hardware_health_steady_at_{};
  std::chrono::steady_clock::time_point last_ui_heartbeat_steady_at_{};
  std::optional<rclcpp::Time> pretension_detection_started_at_;
  std::optional<rclcpp::Time> pretension_started_at_;
  std::optional<rclcpp::Time> calibration_started_at_;
  std::optional<rclcpp::Time> release_started_at_;
  std::vector<Vec3> pretension_detection_samples_;
  std::vector<Vec3> calibration_samples_;
  double target_force_n_ = 10.0;
  bool target_force_configured_ = false;
  double current_command_target_n_ = 0.0;
  double velocity_command_mps_ = 0.0;
  std::string fault_code_;
  std::string stop_reason_;

  bool session_active_ = false;
  std::string session_id_;
  std::string session_record_path_;
  rclcpp::Time session_start_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_record_flush_at_{0, 0, RCL_ROS_TIME};
  std::ofstream record_stream_;
  double session_force_sum_ = 0.0;
  double session_force_max_ = 0.0;
  uint64_t session_sample_count_ = 0;
  msg::TractionHistory history_;

  rclcpp::Publisher<msg::TractionCommand>::SharedPtr command_publisher_;
  rclcpp::Publisher<msg::TractionStatus>::SharedPtr status_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr corrected_wrench_publisher_;
  rclcpp::Publisher<msg::TractionHistory>::SharedPtr history_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr slack_calibration_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_subscription_;
  rclcpp::Subscription<fairino_msgs::msg::PoseTwist>::SharedPtr ee_state_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr health_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr hardware_health_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_subscription_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr heartbeat_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr prepare_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibrate_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr emergency_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_fault_service_;
  rclcpp::Service<srv::SetTargetForce>::SharedPtr target_force_service_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr pretraction_return_client_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  Vec3 latest_joint_velocity_;
  double latest_joint_speed_norm_ = 0.0;
  bool joint_state_valid_ = false;
  bool controller_start_pending_ = false;
  bool controller_activation_confirming_ = false;
  bool controller_stop_requested_ = false;
  bool pretraction_return_call_pending_ = false;
  bool pretraction_return_requested_ = false;
  bool pretraction_return_failed_ = false;
  std::string pretraction_return_failure_reason_;
  rclcpp::Time controller_activation_started_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time controller_stop_requested_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_joint_state_at_{0, 0, RCL_ROS_TIME};
};

}  // namespace fr_traction

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<fr_traction::TractionManagerNode>());
  } catch (const std::exception & error) {
    fprintf(stderr, "traction_manager_node: %s\n", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
