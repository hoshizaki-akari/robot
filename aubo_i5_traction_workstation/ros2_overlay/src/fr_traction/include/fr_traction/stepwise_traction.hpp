#pragma once
#include <algorithm>
#include <cmath>
#include "fr_traction/traction_controller_core.hpp"

namespace fr_traction
{
struct StepwiseTractionConfig
{
  bool force_polarity_verified{false}, drag_axes_verified{false};
  double axis_sign{0.0};
  double force_gain_mps_per_n{0.0002}, force_deadband_n{0.5};
  double position_tolerance_n{1.0}, maximum_speed_mps{0.002};
  // 0.1 s horizon gives >=0.12 mm, above twice the default arrival tolerance.
  // Actual minimum repeatable motion remains a hardware checkpoint.
  double minimum_command_speed_mps{0.0012};
  double drag_deadband_n{1.0}, drag_gain_mps_per_n{0.001};
  double drag_maximum_speed_mps{0.002};
  double drag_minimum_speed_mps{0.0012};
  Vec3 drag_signs{1.0, 1.0, 1.0};
  Vec3 drag_axis_mask{1.0, 1.0, 1.0};
};

// Stateless decision policy: no integral, virtual momentum or delay-based
// displacement accumulation. Hardware scheduler applies a fixed short horizon.
inline ControllerOutput stepwise_traction_output(
  const StepwiseTractionConfig & config, ControlMode mode, const Vec3 & direction,
  double target_force_n, const Vec3 & wrench, const Vec3 & lateral_velocity = {})
{
  ControllerOutput result;
  if (mode == ControlMode::DISABLED) {result.valid = true; return result;}
  if (!finite(wrench) || !finite(lateral_velocity) || !std::isfinite(target_force_n) ||
    target_force_n < 0.0 || !std::isfinite(config.maximum_speed_mps) ||
    config.maximum_speed_mps <= 0.0 || config.maximum_speed_mps > 0.005 ||
    !std::isfinite(config.minimum_command_speed_mps) || config.minimum_command_speed_mps <= 0.0 ||
    config.minimum_command_speed_mps > config.maximum_speed_mps)
  {
    return result;
  }
  const auto cap = [&config](Vec3 value) {
      const double speed = norm(value);
      if (!std::isfinite(speed)) {return Vec3{};}
      return speed > config.maximum_speed_mps ? value * (config.maximum_speed_mps / speed) : value;
    };
  if (mode == ControlMode::DRAGGING) {
    if (!config.drag_axes_verified || !finite(config.drag_signs) ||
      !finite(config.drag_axis_mask) ||
      std::abs(std::abs(config.drag_signs.x) - 1.0) > 1e-9 ||
      std::abs(std::abs(config.drag_signs.y) - 1.0) > 1e-9 ||
      std::abs(std::abs(config.drag_signs.z) - 1.0) > 1e-9 ||
      (config.drag_axis_mask.x != 0.0 && config.drag_axis_mask.x != 1.0) ||
      (config.drag_axis_mask.y != 0.0 && config.drag_axis_mask.y != 1.0) ||
      (config.drag_axis_mask.z != 0.0 && config.drag_axis_mask.z != 1.0) ||
      (config.drag_axis_mask.x + config.drag_axis_mask.y + config.drag_axis_mask.z < 1.0) ||
      !std::isfinite(config.drag_deadband_n) || config.drag_deadband_n < 0.0 ||
      !std::isfinite(config.drag_maximum_speed_mps) || config.drag_maximum_speed_mps <= 0.0 ||
      config.drag_maximum_speed_mps > 0.030 ||
      !std::isfinite(config.drag_minimum_speed_mps) || config.drag_minimum_speed_mps <= 0.0 ||
      config.drag_maximum_speed_mps < config.drag_minimum_speed_mps ||
      !std::isfinite(config.drag_gain_mps_per_n) || config.drag_gain_mps_per_n <= 0.0)
    {
      return result;
    }
    const Vec3 masked_wrench{
      wrench.x * config.drag_axis_mask.x,
      wrench.y * config.drag_axis_mask.y,
      wrench.z * config.drag_axis_mask.z};
    const double force = norm(masked_wrench);
    if (!std::isfinite(force)) {return result;}
    if (force > config.drag_deadband_n) {
      const double speed = std::clamp(
        config.drag_gain_mps_per_n * (force - config.drag_deadband_n),
        config.drag_minimum_speed_mps, config.drag_maximum_speed_mps);
      result.linear_velocity = {
        config.drag_signs.x * masked_wrench.x * speed / force,
        config.drag_signs.y * masked_wrench.y * speed / force,
        config.drag_signs.z * masked_wrench.z * speed / force};
    }
    result.scalar_velocity_mps = norm(result.linear_velocity);
    result.valid = finite(result.linear_velocity);
    return result;
  }
  if (mode == ControlMode::RELEASING) {
    result.linear_velocity = cap(lateral_velocity);
    result.valid = true;
    return result;
  }
  if (!config.force_polarity_verified || !std::isfinite(config.axis_sign) ||
    std::abs(std::abs(config.axis_sign) - 1.0) > 1e-9)
  {
    return result;
  }
  if (mode != ControlMode::PRETENSION && mode != ControlMode::TRACTION &&
    mode != ControlMode::POSITIONING) {return result;}
  Vec3 unit;
  if (!normalize(direction, unit) || !std::isfinite(config.force_gain_mps_per_n) ||
    config.force_gain_mps_per_n <= 0.0 || !std::isfinite(config.force_deadband_n) ||
    config.force_deadband_n < 0.0 || !std::isfinite(config.position_tolerance_n) ||
    config.position_tolerance_n <= 0.0)
  {
    return result;
  }
  // Preserve the original task semantics: positioning and constant-force
  // use total rope tension; pretension uses the confirmed axial projection.
  const double measured = mode == ControlMode::PRETENSION ?
    std::max(0.0, dot(wrench, unit)) : norm(wrench);
  if (!std::isfinite(measured)) {return result;}
  const double error = target_force_n - measured;
  const double band = mode == ControlMode::POSITIONING ?
    std::min(config.position_tolerance_n, std::max(0.05, 0.2 * target_force_n)) :
    config.force_deadband_n;
  const double excess = std::max(0.0, std::abs(error) - band);
  const double magnitude = excess > 0.0 ? std::clamp(
    config.force_gain_mps_per_n * excess,
    config.minimum_command_speed_mps, config.maximum_speed_mps) : 0.0;
  const double scalar = config.axis_sign * std::copysign(magnitude, error);
  result.linear_velocity = cap(
    unit * scalar +
    (mode == ControlMode::TRACTION ? lateral_velocity : Vec3{}));
  result.scalar_velocity_mps = dot(result.linear_velocity, unit);
  result.position_control.valid = true;
  result.position_control.predicted_force_n = measured;
  result.position_control.phase = std::abs(error) <= band ?
    PositionControlPhase::SETTLING : PositionControlPhase::APPROACH;
  result.position_control.velocity_mps = result.scalar_velocity_mps;
  result.position_control.raw_velocity_mps = scalar;
  result.valid = finite(result.linear_velocity);
  return result;
}

// Shared completion predicate used by manager and directly unit-tested.
inline bool measured_idle_allows_completion(
  bool required, bool idle, double age_s, double timeout_s)
{
  return !required || (idle && std::isfinite(age_s) && age_s >= 0.0 &&
         std::isfinite(timeout_s) && timeout_s > 0.0 && age_s <= timeout_s);
}
}  // namespace fr_traction
