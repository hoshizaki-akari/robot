#ifndef FR_TRACTION__ZERO_POSE_POLICY_HPP_
#define FR_TRACTION__ZERO_POSE_POLICY_HPP_

#include <array>
#include <algorithm>
#include <cmath>
#include <limits>

namespace fr_traction
{

struct ZeroPose
{
  std::array<double, 3> position{};
  std::array<double, 4> orientation_xyzw{0.0, 0.0, 0.0, 1.0};
};

inline bool zero_pose_finite(const ZeroPose & pose)
{
  for (const double value : pose.position) {
    if (!std::isfinite(value)) {return false;}
  }
  for (const double value : pose.orientation_xyzw) {
    if (!std::isfinite(value)) {return false;}
  }
  return true;
}

inline bool zero_pose_unit_quaternion(const std::array<double, 4> & quaternion)
{
  const double norm = std::hypot(
    std::hypot(quaternion[0], quaternion[1]),
    std::hypot(quaternion[2], quaternion[3]));
  return std::isfinite(norm) && std::abs(norm - 1.0) <= 1.0e-3;
}

inline double zero_pose_distance_m(const ZeroPose & left, const ZeroPose & right)
{
  return std::hypot(
    left.position[0] - right.position[0], left.position[1] - right.position[1],
    left.position[2] - right.position[2]);
}

inline double zero_pose_orientation_error_rad(const ZeroPose & left, const ZeroPose & right)
{
  double dot = 0.0;
  double left_norm_squared = 0.0;
  double right_norm_squared = 0.0;
  for (std::size_t index = 0; index < left.orientation_xyzw.size(); ++index) {
    dot += left.orientation_xyzw[index] * right.orientation_xyzw[index];
    left_norm_squared += left.orientation_xyzw[index] * left.orientation_xyzw[index];
    right_norm_squared += right.orientation_xyzw[index] * right.orientation_xyzw[index];
  }
  if (!(left_norm_squared > 0.0) || !(right_norm_squared > 0.0)) {
    return std::numeric_limits<double>::infinity();
  }
  dot /= std::sqrt(left_norm_squared * right_norm_squared);
  return 2.0 * std::acos(std::clamp(std::abs(dot), 0.0, 1.0));
}

inline bool zero_pose_capture_allowed(
  const ZeroPose & current, bool stopped, bool fresh_measured_idle, bool live_ee,
  bool live_joint, bool live_hardware, bool controller_pending)
{
  return stopped && fresh_measured_idle && live_ee && live_joint && live_hardware &&
         !controller_pending && zero_pose_finite(current) &&
         zero_pose_unit_quaternion(current.orientation_xyzw);
}

inline bool zero_pose_return_allowed(
  const ZeroPose & current, const ZeroPose & saved, bool saved_valid, bool stopped,
  bool fresh_measured_idle, bool live_ee, bool live_joint, bool live_hardware,
  bool controller_pending, double unloaded_force_n, double max_force_n,
  double max_orientation_error_rad, double max_travel_m)
{
  return saved_valid && zero_pose_capture_allowed(
    current, stopped, fresh_measured_idle, live_ee, live_joint, live_hardware,
    controller_pending) && zero_pose_finite(saved) &&
         zero_pose_unit_quaternion(saved.orientation_xyzw) && std::isfinite(unloaded_force_n) &&
         unloaded_force_n >= 0.0 && unloaded_force_n <= max_force_n &&
         std::isfinite(max_force_n) && max_force_n >= 0.0 &&
         std::isfinite(max_orientation_error_rad) && max_orientation_error_rad >= 0.0 &&
         std::isfinite(max_travel_m) && max_travel_m >= 0.0 &&
         zero_pose_orientation_error_rad(current, saved) <= max_orientation_error_rad &&
         zero_pose_distance_m(current, saved) <= max_travel_m;
}

}  // namespace fr_traction

#endif  // FR_TRACTION__ZERO_POSE_POLICY_HPP_
