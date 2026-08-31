#include "fr_traction/traction_controller_core.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>

#include "fr_traction/msg/traction_command.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"

namespace fr_traction
{

class TractionControllerNode final : public rclcpp::Node
{
public:
  TractionControllerNode()
  : Node("traction_controller"),
    force_filter_(5.0),
    core_(10.0, 80.0, 0.5, 0.005, 0.02)
  {
    control_rate_hz_ = declare_parameter("control_rate_hz", 100.0);
    force_filter_cutoff_hz_ = declare_parameter("force_filter_cutoff_hz", 5.0);
    integral_gain_s_inv_ = declare_parameter("integral_gain_s_inv", 0.25);
    integral_limit_n_ = declare_parameter("integral_limit_n", 3.0);
    virtual_mass_ = declare_parameter("virtual_mass", 10.0);
    virtual_damping_ = declare_parameter("virtual_damping", 80.0);
    force_deadband_n_ = declare_parameter("force_deadband_n", 0.5);
    max_speed_mps_ = declare_parameter("traction_max_speed_mps", 0.005);
    max_acceleration_mps2_ = declare_parameter("traction_max_acc_mps2", 0.02);
    pretension_speed_mps_ = declare_parameter("pretension_speed_mps", 0.002);
    wrench_timeout_s_ = declare_parameter("wrench_timeout_s", 0.10);
    command_timeout_s_ = declare_parameter("command_timeout_s", 0.10);
    wrench_topic_ = declare_parameter(
      "wrench_topic", std::string("/force_torque_sensor_broadcaster/wrench"));
    command_topic_ = declare_parameter("command_topic", std::string("/traction/command"));
    cartesian_command_topic_ = declare_parameter(
      "cartesian_command_topic", std::string("/cartesian_velocity_controller/command_cart_vel"));
    velocity_command_topic_ = declare_parameter(
      "velocity_command_topic", std::string("/traction/controller_velocity_cmd"));

    if (!std::isfinite(control_rate_hz_) || control_rate_hz_ <= 0.0 ||
      !std::isfinite(force_filter_cutoff_hz_) || force_filter_cutoff_hz_ <= 0.0 ||
      !std::isfinite(wrench_timeout_s_) || wrench_timeout_s_ <= 0.0 ||
      !std::isfinite(command_timeout_s_) || command_timeout_s_ <= 0.0 ||
      !std::isfinite(pretension_speed_mps_) || pretension_speed_mps_ <= 0.0)
    {
      throw std::runtime_error("invalid traction controller timing parameters");
    }

    force_filter_.set_cutoff(force_filter_cutoff_hz_);
    core_ = TractionControllerCore(
      virtual_mass_, virtual_damping_, force_deadband_n_, max_speed_mps_,
      max_acceleration_mps2_, integral_gain_s_inv_, integral_limit_n_);

    command_subscription_ = create_subscription<msg::TractionCommand>(
      command_topic_, rclcpp::QoS(10).reliable(),
      [this](const msg::TractionCommand::SharedPtr message) {on_command(*message);});
    wrench_subscription_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      wrench_topic_, rclcpp::SensorDataQoS(),
      [this](const geometry_msgs::msg::WrenchStamped::SharedPtr message) {on_wrench(*message);});
    twist_publisher_ = create_publisher<geometry_msgs::msg::Twist>(
      cartesian_command_topic_, rclcpp::QoS(10).reliable());
    velocity_publisher_ = create_publisher<std_msgs::msg::Float64>(
      velocity_command_topic_, rclcpp::QoS(10).reliable());
    health_publisher_ = create_publisher<std_msgs::msg::Bool>(
      "~/healthy", rclcpp::QoS(1).transient_local().reliable());

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / control_rate_hz_));
    timer_ = create_wall_timer(period, [this]() {control_tick();});
    publish_health(true);
    RCLCPP_INFO(
      get_logger(), "One-dimensional traction controller ready at %.1f Hz.", control_rate_hz_);
  }

private:
  static Vec3 vector_from_message(const geometry_msgs::msg::Vector3 & value)
  {
    return {value.x, value.y, value.z};
  }

  static Vec3 force_from_message(const geometry_msgs::msg::WrenchStamped & message)
  {
    return {message.wrench.force.x, message.wrench.force.y, message.wrench.force.z};
  }

  void on_command(const msg::TractionCommand & message)
  {
    command_ = message;
    last_command_at_ = now();
    ++command_generation_;
    command_valid_ = message.mode <= msg::TractionCommand::RELEASING &&
      std::isfinite(message.target_force_n) && message.target_force_n >= 0.0 &&
      finite(vector_from_message(message.locked_direction_base));
  }

  void on_wrench(const geometry_msgs::msg::WrenchStamped & message)
  {
    latest_wrench_ = force_from_message(message);
    last_wrench_at_ = now();
    wrench_valid_ = message.header.frame_id == "base_link" && finite(latest_wrench_) &&
      std::isfinite(message.wrench.torque.x) && std::isfinite(message.wrench.torque.y) &&
      std::isfinite(message.wrench.torque.z);
  }

  void publish_health(bool healthy)
  {
    if (health_publisher_) {
      std_msgs::msg::Bool message;
      message.data = healthy;
      health_publisher_->publish(message);
    }
  }

  void publish_output(const ControllerOutput & output)
  {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = output.linear_velocity.x;
    twist.linear.y = output.linear_velocity.y;
    twist.linear.z = output.linear_velocity.z;
    // Angular velocity is intentionally always zero for this controller.
    twist_publisher_->publish(twist);
    std_msgs::msg::Float64 scalar;
    scalar.data = output.scalar_velocity_mps;
    velocity_publisher_->publish(scalar);
  }

  void publish_zero()
  {
    publish_output(ControllerOutput{});
  }

  void control_tick()
  {
    const rclcpp::Time current_time = now();
    double dt_s = 1.0 / control_rate_hz_;
    if (last_control_at_.nanoseconds() != 0) {
      if (current_time < last_control_at_) {
        publish_zero();
        blocked_generation_ = command_generation_;
        core_.reset();
        publish_health(false);
        return;
      }
      dt_s = (current_time - last_control_at_).seconds();
    }
    last_control_at_ = current_time;
    if (!std::isfinite(dt_s) || dt_s <= 0.0 || dt_s > 0.25) {
      publish_zero();
      core_.reset();
      publish_health(false);
      return;
    }

    const double command_age = last_command_at_.nanoseconds() == 0 ?
      std::numeric_limits<double>::infinity() : (current_time - last_command_at_).seconds();
    const double wrench_age = last_wrench_at_.nanoseconds() == 0 ?
      std::numeric_limits<double>::infinity() : (current_time - last_wrench_at_).seconds();
    const bool command_fresh = command_valid_ && command_age >= 0.0 &&
      command_age <= command_timeout_s_;
    const bool wrench_fresh = wrench_valid_ && wrench_age >= 0.0 &&
      wrench_age <= wrench_timeout_s_;
    if (!command_fresh || !wrench_fresh || blocked_generation_ == command_generation_) {
      publish_zero();
      core_.reset();
      if (!command_fresh || !wrench_fresh) {
        blocked_generation_ = command_generation_;
      }
      publish_health(true);
      return;
    }

    const Vec3 filtered_wrench = force_filter_.update(latest_wrench_, dt_s);
    const auto mode = static_cast<ControlMode>(command_.mode);
    const ControllerOutput output = core_.update(
      mode, vector_from_message(command_.locked_direction_base), command_.target_force_n,
      filtered_wrench, dt_s);
    if (!output.valid) {
      publish_zero();
      publish_health(false);
      return;
    }
    ControllerOutput bounded_output = output;
    if (mode == ControlMode::PRETENSION &&
      std::abs(bounded_output.scalar_velocity_mps) > pretension_speed_mps_)
    {
      bounded_output.scalar_velocity_mps = std::copysign(
        pretension_speed_mps_, bounded_output.scalar_velocity_mps);
      bounded_output.linear_velocity = vector_from_message(command_.locked_direction_base);
      Vec3 unit;
      if (normalize(bounded_output.linear_velocity, unit)) {
        bounded_output.linear_velocity = unit * bounded_output.scalar_velocity_mps;
      }
    }
    publish_output(bounded_output);
    publish_health(true);
  }

  double control_rate_hz_ = 100.0;
  double force_filter_cutoff_hz_ = 5.0;
  double integral_gain_s_inv_ = 0.25;
  double integral_limit_n_ = 3.0;
  double virtual_mass_ = 10.0;
  double virtual_damping_ = 80.0;
  double force_deadband_n_ = 0.5;
  double max_speed_mps_ = 0.005;
  double max_acceleration_mps2_ = 0.02;
  double pretension_speed_mps_ = 0.002;
  double wrench_timeout_s_ = 0.10;
  double command_timeout_s_ = 0.10;
  std::string wrench_topic_;
  std::string command_topic_;
  std::string cartesian_command_topic_;
  std::string velocity_command_topic_;

  msg::TractionCommand command_;
  Vec3 latest_wrench_;
  bool command_valid_ = false;
  bool wrench_valid_ = false;
  uint64_t command_generation_ = 0;
  uint64_t blocked_generation_ = 0;
  rclcpp::Time last_command_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_wrench_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_control_at_{0, 0, RCL_ROS_TIME};
  FirstOrderLowPass force_filter_;
  TractionControllerCore core_;

  rclcpp::Subscription<msg::TractionCommand>::SharedPtr command_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr velocity_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr health_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace fr_traction

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<fr_traction::TractionControllerNode>());
  } catch (const std::exception & error) {
    fprintf(stderr, "traction_controller_node: %s\n", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
