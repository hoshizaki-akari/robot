#ifndef FR_TRACTION__TRACTION_CONTROLLER_CORE_HPP_
#define FR_TRACTION__TRACTION_CONTROLLER_CORE_HPP_

#include "fr_traction/traction_math.hpp"

namespace fr_traction
{

enum class ControlMode : unsigned char
{
  DISABLED = 0,
  PRETENSION = 1,
  TRACTION = 2,
  RELEASING = 3
};

struct ControllerOutput
{
  Vec3 linear_velocity;
  double scalar_velocity_mps = 0.0;
  bool valid = false;
};

class TractionControllerCore
{
public:
  TractionControllerCore(
    double virtual_mass,
    double virtual_damping,
    double deadband_n,
    double max_speed_mps,
    double max_acceleration_mps2,
    double integral_gain_s_inv = 0.25,
    double integral_limit_n = 3.0);

  void reset();
  ControllerOutput update(
    ControlMode mode,
    const Vec3 & direction,
    double target_force_n,
    const Vec3 & wrench,
    double dt_s);

private:
  OneDimensionalAdmittance admittance_;
};

}  // namespace fr_traction

#endif  // FR_TRACTION__TRACTION_CONTROLLER_CORE_HPP_
