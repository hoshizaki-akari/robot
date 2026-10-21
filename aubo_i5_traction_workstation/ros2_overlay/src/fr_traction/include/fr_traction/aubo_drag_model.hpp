#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include "fr_traction/traction_math.hpp"

namespace fr_traction
{
struct AuboDragConfig
{
  double mobility_mps_per_n{0.010};
  double response_time_s{0.080};
  double deadband_n{0.35}, start_n{0.65}, release_n{0.35};
  double start_confirm_s{0.03};
  double maximum_speed_mps{0.200}, maximum_acceleration_mps2{1.2};
};

// AUBO-only Cartesian admittance: tau * dv/dt + v = mobility * F.
// One isotropic state in base coordinates, no position integral, axis-wise
// direction resets, force derivative, automatic tare, or target-force loop.
class AuboDragModel
{
public:
  explicit AuboDragModel(const AuboDragConfig & config = {})
  : config_(config)
  {
    const double values[]{config.mobility_mps_per_n, config.response_time_s,
      config.deadband_n, config.start_n, config.release_n, config.start_confirm_s,
      config.maximum_speed_mps, config.maximum_acceleration_mps2};
    for (double value : values) {
      if (!std::isfinite(value)) {throw std::invalid_argument("nonfinite AUBO drag config");}
    }
    if (config.mobility_mps_per_n <= 0.0 || config.response_time_s <= 0.0 ||
      config.deadband_n < 0.0 || config.release_n < 0.0 ||
      config.start_n <= config.release_n || config.start_confirm_s < 0.0 ||
      config.maximum_speed_mps <= 0.0 || config.maximum_acceleration_mps2 <= 0.0)
    {
      throw std::invalid_argument("invalid AUBO drag config");
    }
  }

  void reset() {velocity_ = {}; active_ = false; start_time_s_ = 0.0;}

  Vec3 update(const Vec3 & force_base, double dt)
  {
    if (!finite(force_base) || !std::isfinite(dt) || dt <= 0.0 || dt > 0.25) {
      reset(); return {};
    }
    const double force = norm(force_base);
    if (!std::isfinite(force)) {reset(); return {};}
    if (force <= config_.release_n) {reset(); return {};}
    if (!active_) {
      start_time_s_ = force >= config_.start_n ? start_time_s_ + dt : 0.0;
      if (force < config_.start_n || start_time_s_ + 1e-12 < config_.start_confirm_s) {
        return {};
      }
      active_ = true;
    }
    const double speed = std::min(
      config_.maximum_speed_mps,
      config_.mobility_mps_per_n * std::max(0.0, force - config_.deadband_n));
    const Vec3 desired = force_base * (speed / force);
    // Exact ZOH solution; coefficients do not depend on the number of ticks.
    const double alpha = -std::expm1(-dt / config_.response_time_s);
    Vec3 change = (desired - velocity_) * alpha;
    const double change_norm = norm(change);
    if (change_norm > config_.maximum_acceleration_mps2 * dt) {
      change = change * (config_.maximum_acceleration_mps2 * dt / change_norm);
    }
    velocity_ = velocity_ + change;
    // Remove an old opposing component immediately on reversal. This is an
    // orthogonal half-space projection, invariant under rotation of the axes.
    const double along = dot(velocity_, force_base) / force;
    if (along < 0.0) {velocity_ = velocity_ - force_base * (along / force);}
    return velocity_;
  }

private:
  AuboDragConfig config_;
  Vec3 velocity_;
  bool active_{false};
  double start_time_s_{0.0};
};
}  // namespace fr_traction
