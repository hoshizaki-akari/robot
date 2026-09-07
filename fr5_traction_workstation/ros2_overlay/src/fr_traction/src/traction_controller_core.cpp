#include "fr_traction/traction_controller_core.hpp"

#include <cmath>
#include <algorithm>

namespace fr_traction
{

TractionControllerCore::TractionControllerCore(
  double virtual_mass,
  double virtual_damping,
  double deadband_n,
  double max_speed_mps,
  double max_acceleration_mps2,
  double integral_gain_s_inv,
  double integral_limit_n,
  double drag_start_force_n,
  double drag_release_force_n,
  double drag_release_confirm_s,
  double drag_gain_mps_per_n,
  double drag_max_speed_mps,
  double smoothing_max_acceleration_mps2,
  double smoothing_max_jerk_mps3)
: admittance_(
    virtual_mass, virtual_damping, deadband_n, max_speed_mps, max_acceleration_mps2,
    integral_gain_s_inv, integral_limit_n),
  drag_start_force_n_(drag_start_force_n),
  drag_release_force_n_(drag_release_force_n),
  drag_release_confirm_s_(drag_release_confirm_s),
  drag_gain_mps_per_n_(drag_gain_mps_per_n),
  drag_max_speed_mps_(drag_max_speed_mps),
  smoothing_max_acceleration_mps2_(smoothing_max_acceleration_mps2),
  smoothing_max_jerk_mps3_(smoothing_max_jerk_mps3)
{
  if (!std::isfinite(drag_start_force_n_) || drag_start_force_n_ <= 0.0) {
    drag_start_force_n_ = 1.0;
  }
  if (!std::isfinite(drag_release_force_n_) || drag_release_force_n_ < 0.0 ||
    drag_release_force_n_ >= drag_start_force_n_)
  {
    drag_release_force_n_ = 0.6;
  }
  if (!std::isfinite(drag_release_confirm_s_) || drag_release_confirm_s_ <= 0.0) {
    drag_release_confirm_s_ = 0.15;
  }
  if (!std::isfinite(drag_gain_mps_per_n_) || drag_gain_mps_per_n_ <= 0.0) {
    drag_gain_mps_per_n_ = 0.00625;
  }
  if (!std::isfinite(drag_max_speed_mps_) || drag_max_speed_mps_ <= 0.0) {
    drag_max_speed_mps_ = 0.050;
  }
  if (!std::isfinite(smoothing_max_acceleration_mps2_) ||
    smoothing_max_acceleration_mps2_ <= 0.0)
  {
    smoothing_max_acceleration_mps2_ = 0.30;
  }
  if (!std::isfinite(smoothing_max_jerk_mps3_) || smoothing_max_jerk_mps3_ <= 0.0) {
    smoothing_max_jerk_mps3_ = 3.0;
  }
}

void TractionControllerCore::reset()
{
  admittance_.reset();
  drag_active_ = false;
  drag_release_elapsed_s_ = 0.0;
  smoothed_velocity_ = {};
  smoothed_acceleration_ = {};
}

Vec3 TractionControllerCore::smooth_velocity(const Vec3 & desired_velocity, double dt_s)
{
  if (!finite(desired_velocity) || !std::isfinite(dt_s) || dt_s <= 0.0) {return {};}
  const Vec3 previous_velocity = smoothed_velocity_;
  Vec3 desired_acceleration = (desired_velocity - smoothed_velocity_) * (1.0 / dt_s);
  const double requested_acceleration = norm(desired_acceleration);
  if (requested_acceleration > smoothing_max_acceleration_mps2_) {
    desired_acceleration = desired_acceleration *
      (smoothing_max_acceleration_mps2_ / requested_acceleration);
  }
  Vec3 acceleration_delta = desired_acceleration - smoothed_acceleration_;
  const double requested_jerk_step = norm(acceleration_delta);
  const double maximum_jerk_step = smoothing_max_jerk_mps3_ * dt_s;
  if (requested_jerk_step > maximum_jerk_step) {
    acceleration_delta = acceleration_delta * (maximum_jerk_step / requested_jerk_step);
  }
  smoothed_acceleration_ = smoothed_acceleration_ + acceleration_delta;
  smoothed_velocity_ = smoothed_velocity_ + smoothed_acceleration_ * dt_s;

  // Stop exactly at the requested velocity after the limited trajectory
  // reaches it; this also prevents a deceleration step crossing through zero.
  if (dot(desired_velocity - previous_velocity, desired_velocity - smoothed_velocity_) <= 0.0 &&
    norm(desired_velocity - previous_velocity) > 1e-12)
  {
    smoothed_velocity_ = desired_velocity;
    smoothed_acceleration_ = {};
  }
  return smoothed_velocity_;
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
  if (mode == ControlMode::RELEASING) {
    // Direction correction is deliberately lateral-only. Axial force control
    // is reset and resumes after the new rope direction has settled. Preserve
    // the common velocity smoother across cycles so lateral following can
    // accelerate instead of restarting from zero on every 100 Hz update.
    admittance_.reset();
    drag_active_ = false;
    drag_release_elapsed_s_ = 0.0;
    result.linear_velocity = smooth_velocity(lateral_velocity, dt_s);
    result.valid = finite(result.linear_velocity);
    return result;
  }
  if (mode == ControlMode::DRAGGING) {
    if (!finite(wrench) || !std::isfinite(dt_s) || dt_s <= 0.0) {
      reset();
      return result;
    }
    const double force_n = norm(wrench);
    if (!drag_active_ && force_n >= drag_start_force_n_) {
      drag_active_ = true;
      drag_release_elapsed_s_ = 0.0;
    }
    if (drag_active_ && force_n <= drag_release_force_n_) {
      drag_release_elapsed_s_ += dt_s;
      if (drag_release_elapsed_s_ >= drag_release_confirm_s_) {drag_active_ = false;}
    } else {
      drag_release_elapsed_s_ = 0.0;
    }
    Vec3 desired_velocity;
    if (drag_active_ && force_n > 1e-12) {
      const double speed = std::min(
        drag_max_speed_mps_, drag_gain_mps_per_n_ * std::max(0.0, force_n - drag_release_force_n_));
      // Assisted-drag axis signs were calibrated on the actual tool: X/Y
      // retain the reaction-force inversion, while tool Z is inverted once
      // more because a measured Z+ load must produce a Z+ robot motion in
      // the calibrated drag convention.  This mapping belongs only to
      // assisted drag; traction-control signs remain unchanged.
      const Vec3 calibrated_drag_direction{-wrench.x, -wrench.y, wrench.z};
      desired_velocity = calibrated_drag_direction * (speed / force_n);
    }
    result.linear_velocity = smooth_velocity(desired_velocity, dt_s);
    result.scalar_velocity_mps = norm(result.linear_velocity);
    result.valid = finite(result.linear_velocity);
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
  result.linear_velocity = smooth_velocity(unit * scalar + lateral_velocity, dt_s);
  result.scalar_velocity_mps = dot(result.linear_velocity, unit);
  result.valid = finite(result.linear_velocity);
  return result;
}

}  // namespace fr_traction
