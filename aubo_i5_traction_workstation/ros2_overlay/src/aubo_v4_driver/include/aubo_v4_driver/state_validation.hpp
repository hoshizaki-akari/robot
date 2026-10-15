#pragma once
#include <array>
#include <string>
namespace aubo_v4_driver
{
constexpr std::size_t kArmDof = 6;
struct PoseSample { std::array<double, 3> position{}; std::array<double, 4> orientation{};
  std::array<double, kArmDof> joints{}; };
struct DiagnosisSample { bool arm_power_status{false}; unsigned int arm_canbus_status{0U};
  bool remote_halt{false}; bool soft_emergency{false}; bool remote_emergency{false};
  bool robot_collision{false}; bool encoder_error_status{false};
  unsigned int joint_collision_detect{0U}; bool encoder_lines_error{false};
  bool joint_error_status{false}; bool singularity_overspeed_alarm{false};
  bool robot_current_alarm{false}; unsigned int tool_io_error{0U}; };
bool valid_pose_sample(const PoseSample & sample, std::string * reason);
bool diagnosis_is_healthy(const DiagnosisSample & sample, std::string * reason);
// The legacy driver does not enqueue identical stationary waypoints.
bool needs_stream_waypoint(const std::array<double, 3> & velocity);
}  // namespace aubo_v4_driver
