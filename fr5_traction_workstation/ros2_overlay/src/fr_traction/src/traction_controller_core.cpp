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
  double dt_s)
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
  const double scalar = admittance_.update(target_force_n, metrics.actual_force_n, dt_s);
  result.scalar_velocity_mps = scalar;
  result.linear_velocity = unit * scalar;
  result.valid = finite(result.linear_velocity);
  return result;
}

}  // namespace fr_traction
