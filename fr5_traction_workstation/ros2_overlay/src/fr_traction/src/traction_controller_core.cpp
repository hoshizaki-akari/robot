#include "fr_traction/traction_controller_core.hpp"

#include <cmath>

namespace fr_traction
{

TractionControllerCore::TractionControllerCore(
  double virtual_mass,
  double virtual_damping,
  double deadband_n,
  double max_speed_mps,
  double max_acceleration_mps2,
  double integral_gain_s_inv,
  double integral_limit_n)
: admittance_(
    virtual_mass, virtual_damping, deadband_n, max_speed_mps, max_acceleration_mps2,
    integral_gain_s_inv, integral_limit_n)
{
}

void TractionControllerCore::reset()
{
  admittance_.reset();
}

ControllerOutput TractionControllerCore::update(
  ControlMode mode,
  const Vec3 & direction,
  double target_force_n,
  const Vec3 & wrench,
  double dt_s,
  const Vec3 & lateral_velocity)
{
  ControllerOutput result;
  if (mode == ControlMode::DISABLED) {
    reset();
    result.valid = true;
    return result;
  }
  Vec3 unit;
  ForceMetrics metrics;
  if (!normalize(direction, unit) || !project_force(wrench, unit, metrics) ||
    !std::isfinite(target_force_n) || target_force_n < 0.0)
  {
    reset();
    return result;
  }
  // Once traction has started, the controlled value is the rope's total
  // tension. A direction change must not make a healthy 5 N rope look like a
  // smaller force merely because it is no longer parallel to the first
  // locked direction. PRETENSION keeps the original axial projection because
  // its direction has not yet been confirmed.
  const double measured_force_n = mode == ControlMode::TRACTION ?
    norm(wrench) : metrics.actual_force_n;
  if (!std::isfinite(measured_force_n)) {
    reset();
    return result;
  }
  const double scalar = admittance_.update(target_force_n, measured_force_n, dt_s);
  result.scalar_velocity_mps = scalar;
  result.linear_velocity = unit * scalar + lateral_velocity;
  result.valid = finite(result.linear_velocity);
  return result;
}

}  // namespace fr_traction
