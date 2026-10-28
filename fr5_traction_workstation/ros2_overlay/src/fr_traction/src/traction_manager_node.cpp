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
#include "fr_traction/msg/position_control_diagnostics.hpp"
#include "fr_traction/msg/traction_command.hpp"
#include "fr_traction/msg/traction_history.hpp"
#include "fr_traction/msg/traction_record_summary.hpp"
#include "fr_traction/msg/traction_status.hpp"
#include "fr_traction/srv/set_target_force.hpp"
#include "fr_traction/srv/set_operation_mode.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace fr_traction
{

enum class OperationMode : uint8_t
{
  CONSTANT_FORCE = 0,
  POSITION_TRACTION = 1,
  ASSISTED_DRAG = 2,
  UNLOADING = 3
};

const char * operation_mode_name(OperationMode mode)
{
  switch (mode) {
    case OperationMode::CONSTANT_FORCE: return "CONSTANT_FORCE";
    case OperationMode::POSITION_TRACTION: return "POSITION_TRACTION";
    case OperationMode::ASSISTED_DRAG: return "ASSISTED_DRAG";
    case OperationMode::UNLOADING: return "UNLOADING";
  }
  return "UNKNOWN";
}

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
    target_ramp_fast_nps_ = declare_parameter("target_ramp_fast_nps", 12.0);
    target_ramp_slow_nps_ = declare_parameter("target_ramp_slow_nps", 1.5);
    target_ramp_slow_window_n_ = declare_parameter("target_ramp_slow_window_n", 5.0);
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
    target_force_max_n_ = declare_parameter("target_force_max_n", 100.0);
    auto_calibration_window_s_ = declare_parameter("auto_calibration_window_s", 0.5);
    auto_direction_min_force_n_ = declare_parameter("auto_direction_min_force_n", 3.0);
    auto_direction_update_s_ = declare_parameter("auto_direction_update_s", 2.0);
    direction_search_distance_m_ = declare_parameter("direction_search_distance_m", 0.30);
    direction_search_duration_s_ = declare_parameter("direction_search_duration_s", 5.0);
    direction_search_max_speed_mps_ = declare_parameter("direction_search_max_speed_mps", 0.12);
    unload_stop_force_n_ = declare_parameter("unload_stop_force_n", 5.0);
    unload_speed_mps_ = declare_parameter("unload_speed_mps", 0.020);
    const auto initial_mode = declare_parameter("operation_mode", 2);
    if (initial_mode < 1 || initial_mode > 3) {
      throw std::runtime_error("operation_mode must be target(1), drag(2) or unload(3)");
    }
    operation_mode_ = static_cast<OperationMode>(initial_mode);
    force_tolerance_n_ = declare_parameter("force_tolerance_n", 1.0);
    force_deadband_n_ = declare_parameter("force_deadband_n", 0.15);
    // The timeout now covers the direct, low-speed position return. It is not
    // a force-unloading timeout because RELEASING no longer runs force control.
    release_timeout_s_ = declare_parameter("release_timeout_s", 300.0);
    position_reached_tolerance_n_ = declare_parameter("position_reached_tolerance_n", 2.5);
    position_reached_confirm_s_ = declare_parameter("position_reached_confirm_s", 1.5);
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
    velocity_vector_topic_ = declare_parameter(
      "velocity_vector_topic", std::string("/traction/controller_velocity_vector"));
    position_diagnostics_topic_ = declare_parameter(
      "position_diagnostics_topic", std::string("/traction/position_control_diagnostics"));
    corrected_wrench_topic_ = declare_parameter(
      "corrected_wrench_topic", std::string("/traction/corrected_wrench"));
    ui_heartbeat_topic_ = declare_parameter(
      "ui_heartbeat_topic", std::string("/traction/ui_heartbeat"));
    slack_calibration_topic_ = declare_parameter(
      "slack_calibration_topic", std::string("/traction/slack_calibration"));
    native_drag_start_service_name_ = declare_parameter(
      "native_drag_start_service", std::string("/traction/native_drag_start"));
    native_drag_stop_service_name_ = declare_parameter(
      "native_drag_stop_service", std::string("/traction/native_drag_stop"));
    native_drag_gate_stop_service_name_ = declare_parameter(
      "native_drag_gate_stop_service", std::string("/traction/native_drag_gate_stop"));
    expected_wrench_frame_ = declare_parameter("expected_wrench_frame", std::string("base_link"));
    data_directory_ = declare_parameter("data_directory", std::string("debug/traction_sessions"));
    switch_service_name_ = declare_parameter(
      "controller_manager_switch_service", std::string("/controller_manager/switch_controller"));

    validate_parameters();
    force_filter_.set_cutoff(force_filter_cutoff_hz_);
    parameter_callback_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        double proposed = target_force_max_n_;
        for (const auto & parameter : parameters) {
          if (parameter.get_name() != "target_force_max_n") {continue;}
          if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE ||
          !std::isfinite(parameter.as_double()) || parameter.as_double() < target_force_min_n_)
          {
            result.successful = false;
            result.reason = "target_force_max_n must be a finite positive double";
            return result;
          }
          proposed = parameter.as_double();
        }
        if ((is_motion_state(state_machine_.state()) ||
        state_machine_.state() == TractionState::POSITION_HOLD) && target_force_n_ > proposed)
        {
          result.successful = false;
          result.reason = "Stop traction before reducing the configured maximum below its target";
          return result;
        }
        target_force_max_n_ = proposed;
        target_force_n_ = std::min(target_force_n_, target_force_max_n_);
        return result;
      });
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
    native_drag_start_client_ = create_client<std_srvs::srv::Trigger>(
      native_drag_start_service_name_);
    native_drag_stop_client_ = create_client<std_srvs::srv::Trigger>(
      native_drag_stop_service_name_);
    native_drag_gate_stop_client_ = create_client<std_srvs::srv::Trigger>(
      native_drag_gate_stop_service_name_);

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
    physical_teach_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "/controller_manager/physical_teach_drag_active", rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::Bool::SharedPtr state) {
        physical_teach_active_ = state->data;
        if (state->data) {teach_exit_tare_pending_ = true;}
      });
    teach_drag_exit_subscription_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/controller_manager/teach_drag_exit_wrench", rclcpp::QoS(10).reliable(),
      [this](const geometry_msgs::msg::WrenchStamped::SharedPtr event) {
        const Vec3 baseline{event->wrench.force.x, event->wrench.force.y, event->wrench.force.z};
        const auto stamp = rclcpp::Time(event->header.stamp);
        if (event->header.frame_id != "base_link" || !finite(baseline) ||
          stamp.nanoseconds() <= last_teach_drag_exit_stamp_ns_ ||
          std::abs((now() - stamp).seconds()) > 1.0) {return;}
        last_teach_drag_exit_stamp_ns_ = stamp.nanoseconds();
        // Never erase force during a running traction task or fault. The
        // physical-button event is eligible only after the workstation stops.
        if (state_machine_.state() == TractionState::FAULT ||
          state_machine_.state() == TractionState::EMERGENCY_STOP) {return;}
        pending_teach_baseline_ = baseline;
      });
    velocity_subscription_ = create_subscription<std_msgs::msg::Float64>(
      velocity_command_topic_, rclcpp::QoS(10).reliable(),
      [this](const std_msgs::msg::Float64::SharedPtr message) {
        if (std::isfinite(message->data)) {velocity_command_mps_ = message->data;}
      });
    velocity_vector_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      velocity_vector_topic_, rclcpp::QoS(10).reliable(),
      [this](const geometry_msgs::msg::Twist::SharedPtr message) {
        const Vec3 velocity{message->linear.x, message->linear.y, message->linear.z};
        if (finite(velocity)) {controller_velocity_base_ = velocity;}
      });
    position_diagnostics_subscription_ =
      create_subscription<msg::PositionControlDiagnostics>(
      position_diagnostics_topic_, rclcpp::QoS(10).reliable(),
      [this](const msg::PositionControlDiagnostics::SharedPtr message) {
        if (message->phase <= msg::PositionControlDiagnostics::SETTLING &&
        std::isfinite(message->force_rate_nps) &&
        std::isfinite(message->predicted_force_n) &&
        std::isfinite(message->estimated_stiffness_n_per_m) &&
        std::isfinite(message->raw_velocity_mps) &&
        std::isfinite(message->limited_velocity_mps))
        {
          position_diagnostics_ = *message;
          last_position_diagnostics_steady_at_ = std::chrono::steady_clock::now();
        }
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
    unload_service_ = create_service<std_srvs::srv::Trigger>(
      "/traction/unload",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
      const std_srvs::srv::Trigger::Response::SharedPtr response) {
        handle_unload(response);
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
    reset_transient_fault_service_ = create_service<std_srvs::srv::Trigger>(
      "/traction/reset_transient_fault",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
      const std_srvs::srv::Trigger::Response::SharedPtr response) {
        // Recheck the whitelist in the manager callback: a hardware fault or
        // emergency may have replaced the timeout after the web-side check.
        if (state_machine_.state() != TractionState::FAULT ||
          (fault_code_ != "WRENCH_TIMEOUT" && fault_code_ != "EE_STATE_TIMEOUT" &&
          fault_code_ != "UI_HEARTBEAT_TIMEOUT") || !live_ee() ||
          (require_ui_heartbeat_ &&
          !fresh(last_ui_heartbeat_steady_at_, ui_heartbeat_timeout_s_)))
        {
          response->success = false;
          response->message = "仅允许在实时反馈恢复后自动复位软件通信超时";
          return;
        }
        handle_reset_fault(response);
      });
    target_force_service_ = create_service<srv::SetTargetForce>(
      "/traction/set_target_force",
      [this](const srv::SetTargetForce::Request::SharedPtr request,
      const srv::SetTargetForce::Response::SharedPtr response) {
        handle_set_target(request, response);
      });
    operation_mode_service_ = create_service<srv::SetOperationMode>(
      "/traction/set_operation_mode",
      [this](const srv::SetOperationMode::Request::SharedPtr request,
      const srv::SetOperationMode::Response::SharedPtr response) {
        handle_set_operation_mode(request, response);
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
      std::isfinite(auto_calibration_window_s_) && auto_calibration_window_s_ > 0.0 &&
      std::isfinite(auto_direction_min_force_n_) && auto_direction_min_force_n_ > 0.0 &&
      std::isfinite(auto_direction_update_s_) && auto_direction_update_s_ > 0.0 &&
      std::isfinite(direction_search_distance_m_) && direction_search_distance_m_ > 0.0 &&
      direction_search_distance_m_ <= 0.30 &&
      std::isfinite(direction_search_duration_s_) && direction_search_duration_s_ > 0.0 &&
      direction_search_duration_s_ <= 5.0 &&
      std::isfinite(direction_search_max_speed_mps_) && direction_search_max_speed_mps_ > 0.0 &&
      direction_search_max_speed_mps_ <= 0.12 &&
      unload_stop_force_n_ == 5.0 &&
      std::isfinite(unload_speed_mps_) && unload_speed_mps_ > 0.0 && unload_speed_mps_ <= 0.020 &&
      std::isfinite(target_ramp_fast_nps_) && target_ramp_fast_nps_ > 0.0 &&
      std::isfinite(target_ramp_slow_nps_) && target_ramp_slow_nps_ > 0.0 &&
      target_ramp_fast_nps_ >= target_ramp_slow_nps_ &&
      std::isfinite(target_ramp_slow_window_n_) && target_ramp_slow_window_n_ > 0.0 &&
      std::isfinite(force_tolerance_n_) &&
      std::isfinite(release_timeout_s_) && release_timeout_s_ > 0.0 &&
      std::isfinite(position_reached_tolerance_n_) && position_reached_tolerance_n_ > 0.0 &&
      std::isfinite(position_reached_confirm_s_) && position_reached_confirm_s_ > 0.0 &&
      std::isfinite(wrench_timeout_s_) && wrench_timeout_s_ > 0.0 &&
      std::isfinite(ee_state_timeout_s_) && ee_state_timeout_s_ > 0.0 &&
      std::isfinite(motion_pause_timeout_s_) && motion_pause_timeout_s_ > 0.0 &&
      motion_pause_timeout_s_ <= wrench_timeout_s_ &&
      motion_pause_timeout_s_ <= ee_state_timeout_s_ &&
      std::isfinite(ui_heartbeat_timeout_s_) && ui_heartbeat_timeout_s_ > 0.0 &&
      std::isfinite(controller_health_timeout_s_) && controller_health_timeout_s_ > 0.0;
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
    const auto & q = message.pose.orientation;
    const double qnorm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    tool_search_direction_valid_ = std::isfinite(qnorm) && std::abs(qnorm - 1.0) < 0.01 &&
      message.header.frame_id == "base_link";
    if (tool_search_direction_valid_) {
      const double x = q.x / qnorm, y = q.y / qnorm, z = q.z / qnorm, w = q.w / qnorm;
      tool_search_direction_ =
      {-2.0 * (x * z + w * y), -2.0 * (y * z - w * x), -(1.0 - 2.0 * (x * x + y * y))};
    }
    latest_ee_linear_velocity_ = {
      message.twist.linear.x, message.twist.linear.y, message.twist.linear.z};
    latest_ee_angular_velocity_ = {
      message.twist.angular.x, message.twist.angular.y, message.twist.angular.z};
    ee_valid_ = tool_search_direction_valid_ && finite(latest_ee_position_) && finite(
      latest_ee_linear_velocity_) &&
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
    std::string reason;
    if (!normal_stopped() || physical_teach_active_ ||
      controller_stop_count_ > 0 || controller_start_pending_ ||
      controller_activation_confirming_ ||
      native_drag_stop_pending_ || !readiness_check(reason) || !live_ee() ||
      latest_joint_speed_norm_ > 0.02 || !finite(raw_wrench_))
    {
      response->success = false;
      response->message = "重新校准仅允许在设备健康、反馈新鲜且机械臂停止时执行";
      return;
    }
    if (session_active_) {stop_reason_ = "MANUAL_RECALIBRATION"; finalize_session();}
    if (state != TractionState::READY) {transition(TractionState::READY);}
    reset_session_state();
    latest_direction_valid_ = false;
    recorded_direction_valid_ = false;
    auto_direction_samples_.clear();
    auto_direction_started_ = {};
    pending_teach_baseline_.reset();
    apply_baseline(raw_wrench_);
    response->success = true;
    response->message = "重新校准完成；后续结束、切换模式或重连无需重复校准";
  }


  bool normal_stopped() const
  {
    const auto state = state_machine_.state();
    return state == TractionState::READY || state == TractionState::MANUAL_SETUP ||
           state == TractionState::DIRECTION_LOCKED || state == TractionState::COMPLETED;
  }

  void apply_baseline(const Vec3 & baseline)
  {
    wrench_baseline_ = baseline;
    wrench_baseline_valid_ = true;
    latest_wrench_ = raw_wrench_ - baseline;
    filtered_wrench_ = {};
    force_filter_.reset();
    calibrated_ = true;
    teach_exit_tare_pending_ = false;
    std_msgs::msg::Bool event;
    event.data = true;
    slack_calibration_publisher_->publish(event);
    RCLCPP_INFO(
      get_logger(),
      "Software slack baseline calibrated; preserved until explicit recalibration.");
  }

  bool teach_recalibration_allowed() const
  {
    return normal_stopped() || (state_machine_.state() == TractionState::DRAGGING &&
      !native_drag_gate_active_ && !controller_start_pending_ && !native_drag_stop_pending_);
  }

  void update_teach_drag_calibration()
  {
    if (state_machine_.state() == TractionState::FAULT ||
      state_machine_.state() == TractionState::EMERGENCY_STOP)
    {
      pending_teach_baseline_.reset();
      teach_exit_tare_pending_ = false;
      return;
    }
    if (!pending_teach_baseline_ || !teach_recalibration_allowed() || physical_teach_active_) {return;}
    std::string reason;
    if (!readiness_check(reason) || !live_ee() || latest_joint_speed_norm_ > 0.02 ||
      controller_stop_count_ > 0 || controller_start_pending_ || controller_activation_confirming_ ||
      native_drag_stop_pending_) {return;}
    latest_direction_valid_ = false;
    recorded_direction_valid_ = false;
    direction_locked_ = false;
    locked_direction_ = {};
    auto_direction_samples_.clear();
    auto_direction_started_ = {};
    apply_baseline(*pending_teach_baseline_);
    pending_teach_baseline_.reset();
    RCLCPP_INFO(get_logger(), "Force baseline reset after physical drag-teach exit (loaded tare allowed).");
  }

  void update_startup_calibration()
  {
    if (calibrated_) {return;}
    std::string reason;
    if (!normal_stopped() || physical_teach_active_ || teach_exit_tare_pending_ ||
      !readiness_check(reason) || !live_ee() || latest_joint_speed_norm_ > 0.02)
    {
      startup_baseline_samples_.clear();
      startup_baseline_started_ = {};
      return;
    }
    if (startup_baseline_started_ == std::chrono::steady_clock::time_point{}) {
      startup_baseline_started_ = std::chrono::steady_clock::now();
    }
    if (wrench_sequence_ != last_startup_wrench_sequence_) {
      startup_baseline_samples_.push_back(raw_wrench_);
      last_startup_wrench_sequence_ = wrench_sequence_;
    }
    if (age_seconds(startup_baseline_started_) < auto_calibration_window_s_ ||
      startup_baseline_samples_.size() < 20) {return;}
    Vec3 mean{};
    for (const auto & sample : startup_baseline_samples_) {mean = mean + sample;}
    apply_baseline(mean * (1.0 / startup_baseline_samples_.size()));
    startup_baseline_samples_.clear();
  }

  void update_idle_direction()
  {
    if (!calibrated_ || operation_mode_ != OperationMode::POSITION_TRACTION ||
      !normal_stopped() || direction_locked_ || controller_start_pending_ ||
      controller_activation_confirming_ || !motion_feedback_fresh()) {return;}
    if (auto_direction_started_ == std::chrono::steady_clock::time_point{}) {
      auto_direction_started_ = std::chrono::steady_clock::now();
    }
    if (wrench_sequence_ != last_auto_direction_wrench_sequence_) {
      if (norm(filtered_wrench_) >= auto_direction_min_force_n_) {
        auto_direction_samples_.push_back(filtered_wrench_);
      }
      last_auto_direction_wrench_sequence_ = wrench_sequence_;
    }
    if (age_seconds(auto_direction_started_) < auto_direction_update_s_) {return;}
    const auto result = robust_calibrate_direction(
      auto_direction_samples_, 10,
      calibration_max_angle_p95_deg_);
    if (result.success) {
      latest_direction_ = result.direction;
      latest_direction_valid_ = true;
    }
    auto_direction_samples_.clear();
    auto_direction_started_ = std::chrono::steady_clock::now();
  }

  void begin_motion_record()
  {
    stop_reason_.clear();
    fault_code_.clear();
    position_reached_ = false;
    position_reached_started_at_.reset();
    if (!session_active_) {begin_session();}
  }

  void complete_motion(const std::string & reason)
  {
    publish_disabled();
    request_controller_stop();
    stop_reason_ = reason;
    transition(TractionState::COMPLETED);
    write_record_sample(now());
    finalize_session();
    direction_locked_ = false;
    search_active_ = false;
    search_requested_ = false;
    search_contact_pending_ = false;
    search_contact_samples_.clear();
    transition(TractionState::READY);
  }

  void start_unload_motion(const Vec3 & direction)
  {
    const bool reuse_cartesian = state_machine_.state() == TractionState::TRACTION;
    if (session_active_) {stop_reason_ = "SWITCHED_TO_UNLOADING"; finalize_session();}
    operation_mode_ = OperationMode::UNLOADING;
    locked_direction_ = direction;
    direction_locked_ = true;
    begin_motion_record();
    transition(TractionState::RELEASING);
    release_started_at_ = now();
    if (!reuse_cartesian && !request_controller_start()) {
      enter_fault("UNLOAD_CONTROLLER_UNAVAILABLE", "卸力控制器暂不可用");
    }
  }

  void handle_unload(const std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    Vec3 direction;
    const bool have_direction = recorded_direction_valid_ ?
      normalize(recorded_direction_, direction) :
      (latest_direction_valid_ && normalize(latest_direction_, direction));
    std::string reason;
    const auto state = state_machine_.state();
    if (!calibrated_ || !have_direction || physical_teach_active_ || teach_exit_tare_pending_ ||
      !readiness_check(reason) || !live_ee() ||
      (!normal_stopped() && state != TractionState::TRACTION &&
      state != TractionState::POSITION_HOLD && state != TractionState::DRAGGING) ||
      controller_stop_count_ > 0 || controller_start_pending_ ||
      controller_activation_confirming_ || native_drag_stop_pending_)
    {
      response->success = false;
      response->message = "卸力需要已有牵引方向或最新受力方向，以及正常的实时反馈";
      return;
    }
    if (norm(latest_wrench_) < unload_stop_force_n_ && state != TractionState::DRAGGING) {
      if (state == TractionState::TRACTION || state == TractionState::POSITION_HOLD) {
        complete_motion("UNLOAD_BELOW_5N");
        operation_mode_ = OperationMode::UNLOADING;
      }
      response->success = true;
      response->message = "当前受力已低于5 N，无需卸力";
      return;
    }
    publish_disabled();
    if (state == TractionState::DRAGGING) {
      if (!native_drag_stop_client_->service_is_ready()) {
        response->success = false;
        response->message = "省力拖拽停止服务不可用，无法交接卸力";
        return;
      }
      native_drag_stop_pending_ = true;
      unload_handoff_cancelled_ = false;
      const auto epoch = ++motion_request_epoch_;
      native_drag_stop_client_->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>(),
        [this, direction, epoch](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
          if (epoch != motion_request_epoch_) {return;}
          native_drag_stop_pending_ = false;
          try {
            const auto result = future.get();
            if (!result->success) {enter_fault("NATIVE_DRAG_STOP_FAILED", result->message); return;}
            if (unload_handoff_cancelled_ || norm(latest_wrench_) < unload_stop_force_n_) {
              stop_reason_ = unload_handoff_cancelled_ ? "UNLOAD_CANCELLED" : "UNLOAD_BELOW_5N";
              transition(TractionState::COMPLETED);
              write_record_sample(now());
              finalize_session();
              direction_locked_ = false;
              if (!unload_handoff_cancelled_) {operation_mode_ = OperationMode::UNLOADING;}
              transition(TractionState::READY);
              return;
            }
            start_unload_motion(direction);
          } catch (const std::exception & error) {
            enter_fault("UNLOAD_HANDOFF_FAILED", error.what());
          }
        });
    } else {
      start_unload_motion(direction);
    }
    response->success = true;
    response->message = "正在沿已记录方向减少张力，低于5 N后停止";
  }


  void handle_start(const std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    if (operation_mode_ == OperationMode::UNLOADING) {handle_unload(response); return;}
    std::string reason;
    if (!calibrated_ || !normal_stopped() || physical_teach_active_ ||
      !readiness_check(reason) || !live_ee() ||
      controller_stop_count_ > 0 || controller_start_pending_ ||
      controller_activation_confirming_ || native_drag_stop_pending_)
    {
      response->success = false;
      response->message = "等待自动校准完成和实时反馈就绪后开始；无需重复校准";
      return;
    }
    begin_motion_record();
    if (operation_mode_ == OperationMode::ASSISTED_DRAG) {
      if (state_machine_.state() != TractionState::MANUAL_SETUP) {
        transition(TractionState::MANUAL_SETUP);
      }
    } else {
      if (!target_in_range(target_force_n_)) {
        response->success = false;
        response->message = "目标力超出当前设置范围";
        return;
      }
      // Clicking Start can capture an already taut band immediately, rather
      // than move it just because the next 2-second display update is pending.
      if (!latest_direction_valid_ && norm(filtered_wrench_) >= auto_direction_min_force_n_) {
        latest_direction_valid_ = normalize(filtered_wrench_, latest_direction_);
      }
      if (latest_direction_valid_) {
        locked_direction_ = latest_direction_;
        recorded_direction_ = locked_direction_;
        recorded_direction_valid_ = true;
        direction_locked_ = true;
        transition(TractionState::DIRECTION_LOCKED);
      } else {
        if (!tool_search_direction_valid_) {
          response->success = false;
          response->message = "当前工具姿态不可用，无法自动寻找方向";
          return;
        }
        search_requested_ = true;
        search_active_ = false;
        search_contact_pending_ = false;
        search_contact_samples_.clear();
        transition(TractionState::PRETENSION);
      }
    }
    if (!request_controller_start()) {
      enter_fault("CONTROLLER_START_UNAVAILABLE", "控制器服务暂不可用");
      response->success = false;
      response->message = "控制器暂不可用";
      return;
    }
    response->success = true;
    response->message = search_requested_ ?
      "正在沿工具Z负方向平滑寻找受力，最多30厘米/5秒，达到3N后停止确认方向" :
      "启动已请求；本次目标牵引方向保持锁定直至结束";
  }


  void handle_stop(const std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    const auto state = state_machine_.state();
    if (!is_motion_state(state) && state != TractionState::POSITION_HOLD &&
      !controller_start_pending_ && !controller_activation_confirming_)
    {
      response->success = false;
      response->message = "当前没有需要结束的运动";
      return;
    }
    publish_disabled();
    if (operation_mode_ == OperationMode::ASSISTED_DRAG) {
      if (native_drag_stop_pending_) {
        unload_handoff_cancelled_ = true;
        response->success = true; response->message = "正在停止省力拖拽"; return;
      }
      if (!native_drag_stop_client_->service_is_ready()) {
        response->success = false; response->message = "省力拖拽停止服务不可用"; return;
      }
      const auto epoch = ++motion_request_epoch_;
      controller_start_pending_ = false;
      controller_activation_confirming_ = false;
      native_drag_stop_pending_ = true;
      native_drag_stop_client_->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>(),
        [this, epoch](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
          if (epoch != motion_request_epoch_) {return;}
          native_drag_stop_pending_ = false;
          try {
            const auto result = future.get();
            if (!result->success) {enter_fault("NATIVE_DRAG_STOP_FAILED", result->message); return;}
            stop_reason_ = "DRAG_COMPLETED";
            transition(TractionState::COMPLETED);
            write_record_sample(now());
            finalize_session();
            direction_locked_ = false;
            transition(TractionState::READY);
          } catch (const std::exception & error) {
            enter_fault("NATIVE_DRAG_STOP_EXCEPTION", error.what());
          }
        });
    } else {
      complete_motion(
        state == TractionState::PRETENSION ? "DIRECTION_SEARCH_CANCELLED" :
        (state == TractionState::RELEASING ? "UNLOAD_STOPPED" : "TARGET_TRACTION_STOPPED"));
    }
    response->success = true;
    response->message = "正在结束并保持当前位置；校准仍有效";
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
    response->message = "Emergency stop latched after the FR5 hardware stop request.";
  }

  void handle_reset_fault(const std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    const auto state = state_machine_.state();
    const bool emergency_recovery = state == TractionState::EMERGENCY_STOP;
    if (state != TractionState::FAULT && state != TractionState::EMERGENCY_STOP) {
      response->success = false;
      response->message = state_and_allowed("reset_fault only from FAULT or EMERGENCY_STOP");
      return;
    }
    if (controller_stop_count_ > 0 || native_drag_stop_pending_ ||
      !live_controller() || !live_hardware() || !live_wrench() || !live_joint() ||
      latest_joint_speed_norm_ > 0.02 ||
      std::abs(velocity_command_mps_) > 1e-9)
    {
      response->success = false;
      response->message =
        "Reset rejected: FR5 must be stationary and reporting fresh data.";
      return;
    }
    if (!transition(TractionState::READY)) {
      response->success = false;
      response->message = "Reset rejected: state transition failed.";
      return;
    }
    fault_code_.clear();
    stop_reason_.clear();
    direction_locked_ = false;
    search_active_ = false;
    search_requested_ = false;
    search_contact_pending_ = false;
    search_contact_samples_.clear();
    response->success = true;
    response->message = emergency_recovery ?
      "急停已恢复；原校准继续有效，请主动开始新任务" :
      "故障已复位；原校准继续有效，请主动开始新任务";
  }

  void handle_set_target(
    const srv::SetTargetForce::Request::SharedPtr request,
    const srv::SetTargetForce::Response::SharedPtr response)
  {
    if (operation_mode_ != OperationMode::POSITION_TRACTION) {
      response->success = false;
      response->message = "只有目标牵引使用目标力";
      return;
    }
    if (!target_in_range(request->target_force_n)) {
      response->success = false;
      response->message = "目标力超出设置中的可调范围";
      return;
    }
    const auto state = state_machine_.state();
    const bool live_traction_update = state == TractionState::TRACTION &&
      (operation_mode_ == OperationMode::CONSTANT_FORCE ||
      operation_mode_ == OperationMode::POSITION_TRACTION);
    const bool resume_position_from_hold = state == TractionState::POSITION_HOLD &&
      operation_mode_ == OperationMode::POSITION_TRACTION;
    if (state != TractionState::READY && state != TractionState::COMPLETED &&
      state != TractionState::MANUAL_SETUP && state != TractionState::DIRECTION_LOCKED &&
      !live_traction_update && !resume_position_from_hold)
    {
      response->success = false;
      response->message =
        state_and_allowed(
        "set_target_force while stopped, or during constant/position traction");
      return;
    }
    if (resume_position_from_hold && controller_stop_count_ > 0) {
      response->success = false;
      response->message = "正在确认停止，请稍后修改目标";
      return;
    }
    if (resume_position_from_hold && !controller_start_pending_ &&
      !controller_activation_confirming_ && !request_controller_start())
    {
      response->success = false;
      response->message = "Target update rejected: Cartesian controller could not resume.";
      return;
    }
    target_force_n_ = request->target_force_n;
    target_force_configured_ = true;
    if (operation_mode_ == OperationMode::POSITION_TRACTION &&
      (live_traction_update || resume_position_from_hold))
    {
      position_reached_ = false;
      position_reached_started_at_.reset();
    }
    response->success = true;
    response->message = resume_position_from_hold ?
      "Position traction is resuming toward " + std::to_string(target_force_n_) + " N." :
      (live_traction_update ?
      "Live traction target updated to " + std::to_string(target_force_n_) + " N." :
      "Target force set to " + std::to_string(target_force_n_) + " N.");
  }

  void handle_set_operation_mode(
    const srv::SetOperationMode::Request::SharedPtr request,
    const srv::SetOperationMode::Response::SharedPtr response)
  {
    if (request->mode < 1 || request->mode > srv::SetOperationMode::Request::UNLOADING) {
      response->success = false;
      response->message = "Unknown operation mode.";
      return;
    }
    const auto state = state_machine_.state();
    const bool stopped_state = state == TractionState::READY ||
      state == TractionState::MANUAL_SETUP || state == TractionState::DIRECTION_LOCKED ||
      state == TractionState::COMPLETED;
    if (!stopped_state) {
      response->success = false;
      response->message = "Mode can only be changed from a normal stopped state.";
      return;
    }
    if (request->mode == static_cast<uint8_t>(operation_mode_)) {
      response->success = true;
      response->message = std::string("Mode already selected: ") +
        operation_mode_name(operation_mode_);
      return;
    }
    if (is_motion_state(state) || state == TractionState::CALIBRATING ||
      state == TractionState::POSITION_HOLD || controller_start_pending_ ||
      controller_activation_confirming_ || native_drag_stop_pending_ || controller_stop_count_ > 0)
    {
      response->success = false;
      response->message = "Mode can only be changed while the robot is stopped.";
      return;
    }
    if (session_active_) {
      stop_reason_ = "MODE_CHANGED";
      finalize_session();
    }
    if (state == TractionState::DIRECTION_LOCKED || state == TractionState::MANUAL_SETUP ||
      state == TractionState::COMPLETED)
    {
      transition(TractionState::READY);
    }
    operation_mode_ = static_cast<OperationMode>(request->mode);
    direction_locked_ = false;
    auto_direction_samples_.clear();
    auto_direction_started_ = {};
    position_reached_ = false;
    response->success = true;
    response->message = std::string("Mode selected: ") + operation_mode_name(operation_mode_);
  }

  bool target_in_range(double target) const
  {
    return std::isfinite(target) && target >= target_force_min_n_ && target <= target_force_max_n_;
  }

  void reset_session_state()
  {
    search_active_ = false;
    search_requested_ = false;
    search_contact_pending_ = false;
    search_contact_samples_.clear();
    calibration_requested_ = false;
    direction_locked_ = false;
    temporary_direction_valid_ = false;
    locked_direction_ = {};
    temporary_direction_ = {};
    pretension_detection_started_at_.reset();
    pretension_started_at_.reset();
    calibration_started_at_.reset();
    release_started_at_.reset();
    position_reached_started_at_.reset();
    calibration_samples_.clear();
    pretension_detection_samples_.clear();
    safety_monitor_.reset();
    force_filter_.reset();
    controller_start_pending_ = false;
    controller_activation_confirming_ = false;
    native_drag_stop_pending_ = false;
    controller_stop_requested_ = false;
    pretraction_return_call_pending_ = false;
    pretraction_return_requested_ = false;
    pretraction_return_failed_ = false;
    pretraction_return_failure_reason_.clear();
    target_force_configured_ = false;
    position_reached_ = false;
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
    record_stream_ <<
      "timestamp,elapsed_s,state,operation_mode,target_force_n,actual_force_n,lateral_force_n,"
      "fx,fy,fz,dir_x,dir_y,dir_z,initial_dir_x,initial_dir_y,initial_dir_z,"
      "raw_dir_x,raw_dir_y,raw_dir_z,robust_dir_x,robust_dir_y,robust_dir_z,"
      "filtered_dir_x,filtered_dir_y,filtered_dir_z,"
      "candidate_dir_x,candidate_dir_y,candidate_dir_z,"
      "ee_x,ee_y,ee_z,axis_displacement_m,total_travel_m,"
      "velocity_cmd_mps,direction_track_state,direction_error_rad,"
      "direction_fast_slow_error_rad,direction_candidate_dispersion_rad,"
      "direction_candidate_elapsed_s,direction_ambiguity_elapsed_s,"
      "direction_candidate_confirmed,direction_correction_requested_velocity_mps,"
      "direction_correction_velocity_mps,combined_velocity_mps,"
      "direction_correction_displacement_m,direction_vx,direction_vy,direction_vz,"
      "command_vx,command_vy,command_vz,"
      "position_phase,position_force_rate_nps,position_predicted_force_n,"
      "position_stiffness_n_per_m,position_raw_velocity_mps,"
      "position_limited_velocity_mps,position_speed_limited,"
      "position_acceleration_limited,"
      "stop_reason\n";
    record_stream_ << std::setprecision(10);
    session_active_ = true;
    last_record_flush_at_ = session_start_at_;
    session_force_sum_ = 0.0;
    session_force_max_ = 0.0;
    session_sample_count_ = 0;
    position_diagnostics_ = msg::PositionControlDiagnostics();
    last_position_diagnostics_steady_at_ = {};
  }

  Vec3 active_direction() const
  {
    if (direction_locked_) {return locked_direction_;}
    if (operation_mode_ == OperationMode::UNLOADING && recorded_direction_valid_) {
      return recorded_direction_;
    }
    if (latest_direction_valid_) {return latest_direction_;}
    return {};
  }


  ForceMetrics current_metrics() const
  {
    ForceMetrics metrics;
    project_force(filtered_wrench_, active_direction(), metrics);
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

  double total_travel_distance() const
  {
    if (!finite(latest_ee_position_) || !finite(session_start_position_)) {return 0.0;}
    return norm(latest_ee_position_ - session_start_position_);
  }

  void publish_command(
    uint8_t mode,
    const Vec3 & direction,
    double target,
    uint8_t direction_correction_mode = msg::TractionCommand::DIRECTION_CORRECTION_OFF,
    const Vec3 & lateral_velocity = {},
    double linear_speed_mps = 0.0)
  {
    msg::TractionCommand command;
    command.header.stamp = now();
    command.header.frame_id = "base_link";
    command.mode = mode;
    command.locked_direction_base.x = direction.x;
    command.locked_direction_base.y = direction.y;
    command.locked_direction_base.z = direction.z;
    command.target_force_n = target;
    command.linear_speed_mps = linear_speed_mps;
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
    stop_reason_ = reason;
    transition(TractionState::FAULT);
    publish_disabled();
    request_controller_stop();
    // Preserve the terminal state and reason before finalize_session() closes
    // the detailed CSV stream.
    write_record_sample(now());
    finalize_session();
    RCLCPP_ERROR(get_logger(), "Traction fault latched: %s (%s).", code.c_str(), reason.c_str());
  }

  void request_controller_stop()
  {
    ++motion_request_epoch_;
    native_drag_gate_active_ = false;
    native_drag_above_started_ = {};
    controller_activation_confirming_ = false;
    controller_start_pending_ = false;
    if (operation_mode_ == OperationMode::ASSISTED_DRAG) {
      if (!native_drag_stop_client_ || !native_drag_stop_client_->service_is_ready()) {
        RCLCPP_ERROR(
          get_logger(), "FR5 native drag stop service is unavailable during stop.");
        return;
      }
      native_drag_stop_pending_ = true;
      native_drag_stop_client_->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>(),
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
          native_drag_stop_pending_ = false;
          try {
            const auto result = future.get();
            if (!result->success) {
              RCLCPP_ERROR(
                get_logger(), "FR5 native drag stop failed: %s", result->message.c_str());
            }
          } catch (const std::exception & error) {
            RCLCPP_ERROR(get_logger(), "FR5 native drag stop exception: %s", error.what());
          }
        });
      return;
    }
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
    ++controller_stop_count_;
    const auto epoch = motion_request_epoch_;
    switch_client_->async_send_request(
      request,
      [this,
      epoch](
        rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future) {
        --controller_stop_count_;
        try {
          if (!future.get()->ok && epoch == motion_request_epoch_) {
            enter_fault("CARTESIAN_STOP_FAILED", "Cartesian stop could not be confirmed");
          }
        } catch (const std::exception & error) {
          if (epoch == motion_request_epoch_) {
            enter_fault("CARTESIAN_STOP_EXCEPTION", error.what());
          }
        }
      });
  }

  bool request_controller_start()
  {
    if (physical_teach_active_ || teach_exit_tare_pending_) {return false;}
    const auto epoch = ++motion_request_epoch_;
    controller_start_requested_steady_ = std::chrono::steady_clock::now();
    if (operation_mode_ == OperationMode::ASSISTED_DRAG) {
      if (!native_drag_start_client_->service_is_ready() ||
        !native_drag_stop_client_->service_is_ready() ||
        !native_drag_gate_stop_client_->service_is_ready()) {return false;}
      native_drag_gate_active_ = false;
      native_drag_above_started_ = {};
      transition(TractionState::DRAGGING);
      return true;
    }
    if (!switch_client_->service_is_ready()) {return false;}
    auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    request->start_controllers.push_back("cartesian_velocity_controller");
    request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
    request->start_asap = true;
    request->activate_asap = true;
    controller_start_pending_ = true;
    switch_client_->async_send_request(
      request,
      [this,
      epoch](
        rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future) {
        if (epoch != motion_request_epoch_) {return;}
        controller_start_pending_ = false;
        try {
          if (!future.get()->ok) {
            enter_fault("CARTESIAN_CONTROLLER_START_FAILED", "Cartesian start rejected"); return;
          }
          controller_activation_confirming_ = true;
          controller_activation_started_at_ = now();
        } catch (const std::exception & error) {
          enter_fault("CARTESIAN_CONTROLLER_START_EXCEPTION", error.what());
        }
      });
    return true;
  }


  void update_native_drag_gate()
  {
    if (controller_start_pending_ || native_drag_stop_pending_) {return;}
    const bool above = !physical_teach_active_ && !teach_exit_tare_pending_ &&
      !pending_teach_baseline_ &&
      motion_feedback_fresh() &&
      std::round(current_metrics().actual_force_n * 10.0) / 10.0 > 5.0;
    if (!above) {native_drag_above_started_ = {};}
    if (above && !native_drag_gate_active_) {
      if (native_drag_above_started_ == std::chrono::steady_clock::time_point{}) {
        native_drag_above_started_ = std::chrono::steady_clock::now();
      }
      // A short rising-edge confirmation avoids ON/OFF chatter at 5 N;
      // the falling edge has no debounce and immediately requests OFF.
      if (age_seconds(native_drag_above_started_) < 0.10) {return;}
      controller_start_pending_ = true;
      controller_start_requested_steady_ = std::chrono::steady_clock::now();
      const auto epoch = motion_request_epoch_;
      native_drag_start_client_->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>(),
        [this, epoch](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
          if (epoch != motion_request_epoch_) {return;}
          controller_start_pending_ = false;
          try {
            const auto response = future.get();
            if (!response->success) {
              if (response->message.rfind("PHYSICAL_TEACH_ACTIVE:", 0) == 0) {
                native_drag_above_started_ = {};
                return;
              }
              enter_fault("NATIVE_DRAG_START_FAILED", response->message); return;
            }
            native_drag_gate_active_ = true;
          } catch (const std::exception & error) {
            enter_fault("NATIVE_DRAG_START_EXCEPTION", error.what());
          }
        });
    } else if (!above && native_drag_gate_active_) {
      native_drag_stop_pending_ = true;
      const auto epoch = motion_request_epoch_;
      native_drag_gate_stop_client_->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>(),
        [this, epoch](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
          if (epoch != motion_request_epoch_) {return;}
          native_drag_stop_pending_ = false;
          try {
            const auto response = future.get();
            if (!response->success) {enter_fault("NATIVE_DRAG_STOP_FAILED", response->message); return;}
            native_drag_gate_active_ = false;
          } catch (const std::exception & error) {
            enter_fault("NATIVE_DRAG_STOP_EXCEPTION", error.what());
          }
        });
    }
  }

  void check_safety(double monotonic_now_s)
  {
    const auto state = state_machine_.state();
    const bool needs_live_data = controller_start_pending_ || controller_activation_confirming_ ||
      state == TractionState::PRETENSION ||
      state == TractionState::TRACTION || state == TractionState::RELEASING ||
      state == TractionState::DRAGGING || state == TractionState::POSITION_HOLD;
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

  void handle_pretension(double dt_s)
  {
    if (!search_active_ || controller_start_pending_ || controller_activation_confirming_) {
      publish_disabled(); return;
    }
    if (!motion_feedback_fresh()) {
      // A gap cancels this one-shot search, rather than resuming its travel
      // after fresh feedback returns under the same Start request.
      complete_motion("DIRECTION_SEARCH_FEEDBACK_INTERRUPTED"); return;
    }
    if (!tool_search_direction_valid_) {
      complete_motion("DIRECTION_SEARCH_ORIENTATION_UNAVAILABLE"); return;
    }
    if (search_contact_pending_) {
      publish_disabled();
      // A force spike during motion can be inertial, not rope tension.
      // Wait for fresh stationary samples before retaining a direction.
      const double contact_age = age_seconds(search_contact_started_steady_);
      const bool stationary = live_joint() && latest_joint_speed_norm_ <= 0.02 &&
        norm(latest_ee_linear_velocity_) <= 0.002 &&
        norm(latest_ee_angular_velocity_) <= 0.02;
      if (contact_age >= 0.15 && stationary &&
        norm(latest_wrench_) >= auto_direction_min_force_n_)
      {
        if (search_contact_stable_started_.time_since_epoch().count() == 0) {
          search_contact_stable_started_ = std::chrono::steady_clock::now();
        }
        if (wrench_sequence_ != last_search_contact_wrench_sequence_) {
          search_contact_samples_.push_back(latest_wrench_);
          last_search_contact_wrench_sequence_ = wrench_sequence_;
        }
        if (age_seconds(search_contact_stable_started_) >= 0.20) {
          const auto result = robust_calibrate_direction(search_contact_samples_, 5, 15.0);
          if (result.success) {
            latest_direction_ = result.direction;
            latest_direction_valid_ = true;
            locked_direction_ = result.direction;
            recorded_direction_ = result.direction;
            recorded_direction_valid_ = true;
            direction_locked_ = true;
            search_active_ = false;
            search_requested_ = false;
            search_contact_pending_ = false;
            search_contact_samples_.clear();
            transition(TractionState::DIRECTION_LOCKED);
            // Use exactly the normal target-start path after a zero-command
            // settling interval; never carry search velocity into traction.
            controller_activation_confirming_ = true;
            controller_activation_started_at_ = now();
            return;
          }
        }
      } else {
        search_contact_stable_started_ = {};
        search_contact_samples_.clear();
      }
      if (contact_age >= 0.35 && stationary &&
        norm(latest_wrench_) < auto_direction_min_force_n_)
      {
        // A transient spike stopped motion without establishing tension.
        // Continue the same search; never reset its deadline/travel origin.
        search_contact_pending_ = false;
        search_contact_samples_.clear();
        search_command_speed_mps_ = 0.0;
        search_command_acceleration_mps2_ = 0.0;
        if (age_seconds(search_started_steady_) >= direction_search_duration_s_ ||
          norm(latest_ee_position_ - search_start_position_) >= direction_search_distance_m_)
        {
          complete_motion("DIRECTION_SEARCH_NO_FORCE");
        }
        return;
      }
      if (contact_age >= 1.0) {
        complete_motion("DIRECTION_SEARCH_CONTACT_UNSTABLE");
      }
      return;
    }
    if (norm(latest_wrench_) >= auto_direction_min_force_n_) {
      publish_disabled();
      search_contact_pending_ = true;
      search_contact_started_steady_ = std::chrono::steady_clock::now();
      search_contact_stable_started_ = {};
      last_search_contact_wrench_sequence_ = wrench_sequence_;
      search_contact_samples_.clear();
      search_command_speed_mps_ = 0.0;
      search_command_acceleration_mps2_ = 0.0;
      return;
    }
    const double elapsed = age_seconds(search_started_steady_);
    const double distance = norm(latest_ee_position_ - search_start_position_);
    if (elapsed >= direction_search_duration_s_ || distance >= direction_search_distance_m_) {
      complete_motion("DIRECTION_SEARCH_NO_FORCE");
      return;
    }
    const double u = std::clamp(elapsed / direction_search_duration_s_, 0.0, 1.0);
    // The uninterrupted 5 s quintic trajectory integrates to 30 cm. A
    // contact pause never resets the original time/distance allowance.
    const double planned_distance = std::min(direction_search_distance_m_,
      direction_search_max_speed_mps_ * direction_search_duration_s_ / 1.875);
    const double desired_speed = planned_distance / direction_search_duration_s_ *
      30.0 * u * u * (1.0 - u) * (1.0 - u);
    const double trajectory_dt = std::clamp(dt_s, 0.0, 0.02);
    if (trajectory_dt > 0.0) {
      // Inactive for the normal curve; limits acceleration when a transient
      // contact pause resumes midway through it from zero velocity.
      const double wanted_acceleration = std::clamp(
        (desired_speed - search_command_speed_mps_) / trajectory_dt, -0.08, 0.08);
      search_command_acceleration_mps2_ += std::clamp(
        wanted_acceleration - search_command_acceleration_mps2_,
        -0.25 * trajectory_dt, 0.25 * trajectory_dt);
      const double previous_speed = search_command_speed_mps_;
      search_command_speed_mps_ = std::clamp(
        previous_speed + search_command_acceleration_mps2_ * trajectory_dt,
        0.0, direction_search_max_speed_mps_);
      if ((desired_speed - previous_speed) *
        (desired_speed - search_command_speed_mps_) <= 0.0)
      {
        search_command_speed_mps_ = desired_speed;
      }
    }
    publish_command(
      msg::TractionCommand::DIRECTION_SEARCH, search_direction_, 0.0,
      msg::TractionCommand::DIRECTION_CORRECTION_OFF, {}, search_command_speed_mps_);
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
      // Releasing or moving the band during the short direction-sampling
      // window is an ordinary operator retry, not a robot/device fault.
      // Return to passive manual setup and let the operator tension the band
      // and confirm direction again without resetting the whole workstation.
      calibration_samples_.clear();
      calibration_started_at_.reset();
      calibration_requested_ = false;
      temporary_direction_valid_ = false;
      temporary_direction_ = {};
      direction_locked_ = false;
      locked_direction_ = {};
      target_force_configured_ = false;
      transition(TractionState::MANUAL_SETUP);
      RCLCPP_WARN(
        get_logger(), "Direction calibration was not retained (%s); waiting for another attempt.",
        result.reason.c_str());
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


  void handle_releasing(const rclcpp::Time & current_time)
  {
    if (controller_start_pending_ || controller_activation_confirming_) {
      publish_disabled(); return;
    }
    if (!motion_feedback_fresh()) {publish_disabled(); return;}
    if (norm(latest_wrench_) < unload_stop_force_n_) {
      complete_motion("UNLOAD_BELOW_5N"); return;
    }
    if (!direction_locked_ || !finite(locked_direction_)) {
      enter_fault("UNLOAD_DIRECTION_UNAVAILABLE", "卸力方向不可用"); return;
    }
    publish_command(
      msg::TractionCommand::UNLOADING, locked_direction_, unload_stop_force_n_,
      msg::TractionCommand::DIRECTION_CORRECTION_OFF, {}, unload_speed_mps_);
    if (release_started_at_ &&
      (current_time - *release_started_at_).seconds() > release_timeout_s_)
    {
      complete_motion("UNLOAD_TIMEOUT");
    }
  }


  double ramped_command_target(double dt_s) const
  {
    return next_force_command_target(
      current_command_target_n_, target_force_n_, dt_s,
      target_ramp_fast_nps_, target_ramp_slow_nps_, target_ramp_slow_window_n_);
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
      if (readiness_check(reason) && live_ee()) {transition(TractionState::READY);}
    }
    update_teach_drag_calibration();
    update_startup_calibration();
    update_idle_direction();
    check_safety(monotonic_now_s);
    if (controller_start_pending_ && age_seconds(controller_start_requested_steady_) > 20.0) {
      enter_fault("CONTROLLER_START_RESPONSE_TIMEOUT", "启动控制器未按时回应，请检查设备连接");
    }
    if (controller_activation_confirming_) {
      publish_disabled();
      if (live_wrench() && live_controller() && live_hardware() && live_ee()) {
        controller_activation_confirming_ = false;
        if (operation_mode_ == OperationMode::ASSISTED_DRAG) {
          transition(TractionState::DRAGGING);
        } else if (state_machine_.state() == TractionState::PRETENSION) {
          search_start_position_ = latest_ee_position_;
          search_direction_ = tool_search_direction_;
          search_started_steady_ = std::chrono::steady_clock::now();
          search_command_speed_mps_ = 0.0;
          search_command_acceleration_mps2_ = 0.0;
          search_active_ = true;
        } else if (state_machine_.state() == TractionState::RELEASING) {
          release_started_at_ = current_time;
        } else {
          position_reached_ = false;
          position_reached_started_at_.reset();
          transition(TractionState::TRACTION);
        }
      } else if ((current_time - controller_activation_started_at_).seconds() > 2.0) {
        enter_fault("CONTROLLER_ACTIVATION_TIMEOUT", "CONTROLLER_ACTIVATION_TIMEOUT");
      }
    }
    switch (state_machine_.state()) {
      case TractionState::MANUAL_SETUP: handle_manual_setup(current_time); break;
      case TractionState::PRETENSION: handle_pretension(dt_s); break;
      case TractionState::CALIBRATING: handle_calibration(current_time); break;
      case TractionState::TRACTION:
        if (!motion_feedback_fresh()) {
          position_reached_started_at_.reset();
          publish_disabled();
          break;
        }
        if (operation_mode_ == OperationMode::POSITION_TRACTION) {
          // Position traction has its own predictive velocity controller. It
          // receives the final target directly and performs fast approach,
          // early braking and fine correction without inheriting the virtual
          // momentum used by continuous constant-force traction.
          current_command_target_n_ = target_force_n_;
          publish_command(
            msg::TractionCommand::POSITIONING, locked_direction_, current_command_target_n_,
            msg::TractionCommand::DIRECTION_CORRECTION_OFF);
          const double force_error = std::abs(current_metrics().actual_force_n - target_force_n_);
          // Keep aiming for the requested center while independently timing
          // continuous occupancy of the user-facing arrival band.
          if (force_error <= position_reached_tolerance_n_) {
            const auto reached_now = std::chrono::steady_clock::now();
            if (!position_reached_started_at_) {position_reached_started_at_ = reached_now;}
            if (std::chrono::duration<double>(reached_now - *position_reached_started_at_).count() >=
              position_reached_confirm_s_)
            {
              publish_disabled();
              request_controller_stop();
              position_reached_ = true;
              transition(TractionState::POSITION_HOLD);
            }
          } else {
            position_reached_started_at_.reset();
          }
          break;
        }
        publish_disabled();
        break;
      case TractionState::DRAGGING:
        publish_disabled();
        update_native_drag_gate();
        break;
      case TractionState::RELEASING: handle_releasing(current_time); break;
      case TractionState::INITIALIZING:
      case TractionState::READY:
      case TractionState::DIRECTION_LOCKED:
      case TractionState::COMPLETED:
      case TractionState::FAULT:
      case TractionState::EMERGENCY_STOP:
      case TractionState::POSITION_HOLD:
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
    const Vec3 combined_velocity = controller_velocity_base_;
    const double elapsed = (current_time - session_start_at_).seconds();
    record_stream_ << std::fixed << current_time.seconds() << ',' << elapsed << ','
                   << state_name(state_machine_.state()) << ',' << operation_mode_name(
      operation_mode_)
                   << ',' << current_command_target_n_ << ','
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
                   << axis_displacement() << ',' << total_travel_distance() << ','
                   << velocity_command_mps_ << ','
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
                   << lateral_correction_velocity_base_.z << ','
                   << controller_velocity_base_.x << ',' << controller_velocity_base_.y << ','
                   << controller_velocity_base_.z << ','
                   << static_cast<unsigned>(position_diagnostics_.phase) << ','
                   << position_diagnostics_.force_rate_nps << ','
                   << position_diagnostics_.predicted_force_n << ','
                   << position_diagnostics_.estimated_stiffness_n_per_m << ','
                   << position_diagnostics_.raw_velocity_mps << ','
                   << position_diagnostics_.limited_velocity_mps << ','
                   << (position_diagnostics_.speed_limited ? 1 : 0) << ','
                   << (position_diagnostics_.acceleration_limited ? 1 : 0) << ','
                   << stop_reason_ << '\n';
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
    summary.operation_mode = static_cast<uint8_t>(operation_mode_);
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
    const auto sessions_path =
      (std::filesystem::path(data_directory_) / "sessions_multimode.csv").string();
    const bool exists = std::filesystem::exists(sessions_path);
    std::ofstream sessions(sessions_path, std::ios::out | std::ios::app);
    if (sessions) {
      if (!exists) {
        sessions <<
          "session_id,operation_mode,start_time,end_time,target_force_n,average_force_n,max_force_n,"
          "final_state,stop_reason,record_path\n";
      }
      const double start_seconds = static_cast<double>(summary.start_time.sec) +
        static_cast<double>(summary.start_time.nanosec) * 1e-9;
      const double end_seconds = static_cast<double>(summary.end_time.sec) +
        static_cast<double>(summary.end_time.nanosec) * 1e-9;
      sessions << summary.session_id << ',' << static_cast<int>(summary.operation_mode) << ','
               << start_seconds << ',' << end_seconds << ','
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
    const std::vector<std::string> paths = {
      (std::filesystem::path(data_directory_) / "sessions.csv").string(),
      (std::filesystem::path(data_directory_) / "sessions_multimode.csv").string()};
    for (const auto & path : paths) {
      std::ifstream sessions(path);
      if (!sessions) {continue;}
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
          const bool has_mode = fields.size() >= 10;
          const std::size_t offset = has_mode ? 1 : 0;
          summary.operation_mode = has_mode ? static_cast<uint8_t>(std::stoi(fields[1])) :
            static_cast<uint8_t>(OperationMode::CONSTANT_FORCE);
          const auto start_ns = static_cast<int64_t>(std::stod(fields[1 + offset]) * 1e9);
          const auto end_ns = static_cast<int64_t>(std::stod(fields[2 + offset]) * 1e9);
          summary.start_time = to_builtin_time(rclcpp::Time(start_ns, RCL_ROS_TIME));
          summary.end_time = to_builtin_time(rclcpp::Time(end_ns, RCL_ROS_TIME));
          summary.target_force_n = std::stod(fields[3 + offset]);
          summary.average_force_n = std::stod(fields[4 + offset]);
          summary.max_force_n = std::stod(fields[5 + offset]);
          summary.final_state = static_cast<uint8_t>(std::stoi(fields[6 + offset]));
          summary.stop_reason = fields[7 + offset];
          summary.record_path = fields[8 + offset];
          history_.summaries.push_back(summary);
        } catch (const std::exception &) {
          RCLCPP_WARN(get_logger(), "Ignoring malformed sessions.csv row.");
        }
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
    status.operation_mode = static_cast<uint8_t>(operation_mode_);
    status.position_reached = position_reached_;
    status.calibrated = calibrated_;
    status.direction_available = recorded_direction_valid_ || latest_direction_valid_;
    status.direction_locked = direction_locked_;
    status.ready = state_machine_.state() != TractionState::INITIALIZING &&
      state_machine_.state() != TractionState::FAULT &&
      state_machine_.state() != TractionState::EMERGENCY_STOP &&
      live_controller() && live_hardware() && controller_stop_count_ == 0 &&
      !controller_start_pending_ && !controller_activation_confirming_ &&
      !native_drag_stop_pending_ && !physical_teach_active_ && !teach_exit_tare_pending_;
    // The message exposes the operator-selected target, not the instantaneous
    // release/disabled command. This keeps READY/DIRECTION_LOCKED observable
    // without turning the target into zero between motion phases.
    status.target_force_n = target_force_n_;
    status.target_force_max_n = target_force_max_n_;
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
    status.locked_direction_base.x = direction.x;
    status.locked_direction_base.y = direction.y;
    status.locked_direction_base.z = direction.z;
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
    status.direction_correction_active = false;
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
      case TractionState::READY:
        status.message = !calibrated_ ? "正在自动初始校准，请保持机械臂静止" :
          (stop_reason_ == "DIRECTION_SEARCH_NO_FORCE" ? "低速搜索已结束但未达到3 N，请检查绳带后重试" :
          (stop_reason_ == "DIRECTION_SEARCH_CONTACT_UNSTABLE" ? "搜索受力未保持稳定，已停止；请张紧绳带后重新开始" :
          (operation_mode_ == OperationMode::POSITION_TRACTION ?
          (latest_direction_valid_ ? "方向每2秒更新；点击开始后锁定" : "等待3 N以上施力；也可直接开始自动寻找方向") :
          "已校准，可以开始；无需重复校准")));
        break;
      case TractionState::MANUAL_SETUP: status.message = "已校准，可以开始"; break;
      case TractionState::CALIBRATING: status.message = "正在采集张紧方向，请保持法兰静止"; break;
      case TractionState::DIRECTION_LOCKED: status.message = "本次方向已锁定，正在准备目标牵引"; break;
      case TractionState::TRACTION:
        status.message = operation_mode_ == OperationMode::POSITION_TRACTION ?
          "目标牵引中，方向已锁定" : "牵引中";
        break;
      case TractionState::RELEASING: status.message = "正在卸力，受力低于5 N后停止"; break;
      case TractionState::COMPLETED: status.message = "牵引完成"; break;
      case TractionState::FAULT: status.message = "设备故障，请检查原因并恢复；无需重复校准"; break;
      case TractionState::EMERGENCY_STOP: status.message = "已急停并下使能；确认后点击急停恢复"; break;
      case TractionState::PRETENSION:
        status.message = search_contact_pending_ ?
          "搜索已停止，正在静止确认受力方向" :
          "正在沿工具Z负方向平滑搜索，达到3 N或30厘米/5秒后停止";
        break;
      case TractionState::DRAGGING:
        status.message = native_drag_stop_pending_ ?
          "正在停止省力拖拽" :
          (native_drag_gate_active_ ? "省力拖拽中；合力降至5N及以下停住" : "省力拖拽待施力；合力超过5N后拖动");
        break;
      case TractionState::POSITION_HOLD: status.message = "已到位，机械臂保持当前位置"; break;
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
  double target_force_max_n_ = 100.0;
  double target_ramp_fast_nps_ = 12.0;
  double target_ramp_slow_nps_ = 1.5;
  double target_ramp_slow_window_n_ = 5.0;
  double auto_calibration_window_s_ = 0.5;
  double auto_direction_min_force_n_ = 3.0;
  double auto_direction_update_s_ = 2.0;
  double direction_search_distance_m_ = 0.30;
  double direction_search_duration_s_ = 5.0;
  double direction_search_max_speed_mps_ = 0.12;
  double search_command_speed_mps_ = 0.0;
  double search_command_acceleration_mps2_ = 0.0;
  double unload_stop_force_n_ = 5.0;
  double unload_speed_mps_ = 0.020;
  double force_tolerance_n_ = 1.0;
  double force_deadband_n_ = 0.15;
  double release_timeout_s_ = 300.0;
  double position_reached_tolerance_n_ = 2.5;
  double position_reached_confirm_s_ = 1.5;
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
  std::string velocity_vector_topic_;
  std::string position_diagnostics_topic_;
  std::string corrected_wrench_topic_;
  std::string ui_heartbeat_topic_;
  std::string slack_calibration_topic_;
  std::string expected_wrench_frame_;
  std::string data_directory_;
  std::string switch_service_name_;
  std::string native_drag_start_service_name_;
  std::string native_drag_stop_service_name_;
  std::string native_drag_gate_stop_service_name_;
  std::unique_ptr<DirectionEstimator> direction_estimator_;
  std::unique_ptr<AdaptiveDirectionFollower> direction_controller_;
  DirectionNoiseProfile direction_noise_profile_;
  DirectionEstimate direction_estimate_;
  AdaptiveFollowResult lateral_correction_result_;
  Vec3 lateral_correction_velocity_base_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
  bool calibrated_ = false;
  bool latest_direction_valid_ = false;
  bool recorded_direction_valid_ = false;
  bool tool_search_direction_valid_ = false;
  bool search_requested_ = false;
  bool search_active_ = false;
  bool search_contact_pending_ = false;
  uint64_t motion_request_epoch_ = 0;
  uint64_t last_startup_wrench_sequence_ = 0;
  uint64_t last_auto_direction_wrench_sequence_ = 0;
  uint64_t last_search_contact_wrench_sequence_ = 0;
  Vec3 latest_direction_;
  Vec3 recorded_direction_;
  Vec3 tool_search_direction_;
  Vec3 search_direction_;
  Vec3 search_start_position_;
  std::vector<Vec3> startup_baseline_samples_;
  std::vector<Vec3> auto_direction_samples_;
  std::vector<Vec3> search_contact_samples_;
  std::chrono::steady_clock::time_point startup_baseline_started_{};
  std::chrono::steady_clock::time_point auto_direction_started_{};
  std::chrono::steady_clock::time_point search_started_steady_{};
  std::chrono::steady_clock::time_point search_contact_started_steady_{};
  std::chrono::steady_clock::time_point search_contact_stable_started_{};
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
  std::chrono::steady_clock::time_point last_position_diagnostics_steady_at_{};
  std::optional<rclcpp::Time> pretension_detection_started_at_;
  std::optional<rclcpp::Time> pretension_started_at_;
  std::optional<rclcpp::Time> calibration_started_at_;
  std::optional<rclcpp::Time> release_started_at_;
  std::optional<std::chrono::steady_clock::time_point> position_reached_started_at_;
  std::vector<Vec3> pretension_detection_samples_;
  std::vector<Vec3> calibration_samples_;
  double target_force_n_ = 10.0;
  bool target_force_configured_ = false;
  bool physical_teach_active_ = false;
  bool teach_exit_tare_pending_ = false;
  bool native_drag_gate_active_ = false;
  std::chrono::steady_clock::time_point native_drag_above_started_{};
  std::optional<Vec3> pending_teach_baseline_;
  int64_t last_teach_drag_exit_stamp_ns_ = 0;
  bool position_reached_ = false;
  OperationMode operation_mode_ = OperationMode::ASSISTED_DRAG;
  double current_command_target_n_ = 0.0;
  double velocity_command_mps_ = 0.0;
  Vec3 controller_velocity_base_;
  msg::PositionControlDiagnostics position_diagnostics_;
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
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr teach_drag_exit_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr health_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr hardware_health_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr physical_teach_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_vector_subscription_;
  rclcpp::Subscription<msg::PositionControlDiagnostics>::SharedPtr
    position_diagnostics_subscription_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr heartbeat_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr prepare_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr unload_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr emergency_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_fault_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_transient_fault_service_;
  rclcpp::Service<srv::SetTargetForce>::SharedPtr target_force_service_;
  rclcpp::Service<srv::SetOperationMode>::SharedPtr operation_mode_service_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr native_drag_start_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr native_drag_stop_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr native_drag_gate_stop_client_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  Vec3 latest_joint_velocity_;
  double latest_joint_speed_norm_ = 0.0;
  bool joint_state_valid_ = false;
  bool controller_start_pending_ = false;
  unsigned controller_stop_count_ = 0;
  bool controller_activation_confirming_ = false;
  bool native_drag_stop_pending_ = false;
  bool unload_handoff_cancelled_ = false;
  std::chrono::steady_clock::time_point controller_start_requested_steady_{};
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
