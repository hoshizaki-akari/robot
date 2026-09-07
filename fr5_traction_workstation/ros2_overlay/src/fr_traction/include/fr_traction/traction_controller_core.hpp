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
  RELEASING = 3,
  DRAGGING = 4
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
    double integral_limit_n = 3.0,
    double drag_start_force_n = 1.0,
    double drag_release_force_n = 0.6,
    double drag_release_confirm_s = 0.15,
    double drag_gain_mps_per_n = 0.00625,
    double drag_max_speed_mps = 0.050,
    double smoothing_max_acceleration_mps2 = 0.30,
    double smoothing_max_jerk_mps3 = 3.0);

  void reset();
  ControllerOutput update(
    ControlMode mode,
    const Vec3 & direction,
    double target_force_n,
    const Vec3 & wrench,
    double dt_s,
    const Vec3 & lateral_velocity = {});

private:
  Vec3 smooth_velocity(const Vec3 & desired_velocity, double dt_s);

  OneDimensionalAdmittance admittance_;
  double drag_start_force_n_;
  double drag_release_force_n_;
  double drag_release_confirm_s_;
  double drag_gain_mps_per_n_;
  double drag_max_speed_mps_;
  double smoothing_max_acceleration_mps2_;
  double smoothing_max_jerk_mps3_;
  bool drag_active_ = false;
  double drag_release_elapsed_s_ = 0.0;
  Vec3 smoothed_velocity_;
  Vec3 smoothed_acceleration_;
};

}  // namespace fr_traction

#endif  // FR_TRACTION__TRACTION_CONTROLLER_CORE_HPP_
