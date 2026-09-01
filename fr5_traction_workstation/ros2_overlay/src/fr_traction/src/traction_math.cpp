#include "fr_traction/traction_math.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace fr_traction
{

constexpr double kPi = 3.14159265358979323846;

Vec3 operator+(const Vec3 & lhs, const Vec3 & rhs)
{
  return {lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
}

Vec3 operator-(const Vec3 & lhs, const Vec3 & rhs)
{
  return {lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
}

Vec3 operator*(const Vec3 & value, double scalar)
{
  return {value.x * scalar, value.y * scalar, value.z * scalar};
}

Vec3 operator*(double scalar, const Vec3 & value)
{
  return value * scalar;
}

double dot(const Vec3 & lhs, const Vec3 & rhs)
{
  return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

double norm(const Vec3 & value)
{
  return std::sqrt(dot(value, value));
}

bool finite(const Vec3 & value)
{
  return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

bool normalize(const Vec3 & value, Vec3 & unit, double minimum_norm)
{
  const double length = norm(value);
  if (!finite(value) || !std::isfinite(length) || length <= minimum_norm) {
    unit = {};
    return false;
  }
  unit = value * (1.0 / length);
  return finite(unit);
}

bool project_force(const Vec3 & force, const Vec3 & direction, ForceMetrics & result)
{
  Vec3 unit;
  if (!finite(force) || !normalize(direction, unit) || std::abs(norm(direction) - 1.0) > 1e-6) {
    result = {};
    return false;
  }
  // The KWR75D field reports the direction in which the flange load acts.
  // On this FR5 setup that is also the positive direction that increases
  // belt tension, so the axial projection is positive dot(force, unit).
  result.actual_force_n = std::max(0.0, dot(force, unit));
  result.lateral_force_vector = force - result.actual_force_n * unit;
  result.lateral_force_n = norm(result.lateral_force_vector);
  return finite(result.lateral_force_vector) && std::isfinite(result.lateral_force_n);
}

FirstOrderLowPass::FirstOrderLowPass(double cutoff_hz)
: cutoff_hz_(cutoff_hz)
{
  set_cutoff(cutoff_hz);
}

void FirstOrderLowPass::set_cutoff(double cutoff_hz)
{
  if (!std::isfinite(cutoff_hz) || cutoff_hz <= 0.0) {
    cutoff_hz_ = 5.0;
  } else {
    cutoff_hz_ = cutoff_hz;
  }
}

void FirstOrderLowPass::reset()
{
  initialized_ = false;
  value_ = {};
}

Vec3 FirstOrderLowPass::update(const Vec3 & sample, double dt_s)
{
  if (!finite(sample) || !std::isfinite(dt_s) || dt_s <= 0.0) {
    return value_;
  }
  if (!initialized_) {
    value_ = sample;
    initialized_ = true;
    return value_;
  }
  const double alpha = 1.0 - std::exp(-2.0 * kPi * cutoff_hz_ * dt_s);
  value_ = value_ + (sample - value_) * std::clamp(alpha, 0.0, 1.0);
  return value_;
}

OneDimensionalAdmittance::OneDimensionalAdmittance(
  double virtual_mass,
  double virtual_damping,
  double deadband_n,
  double max_speed_mps,
  double max_acceleration_mps2,
  double integral_gain_s_inv,
  double integral_limit_n)
: mass_(virtual_mass), damping_(virtual_damping), deadband_n_(deadband_n),
  max_speed_mps_(max_speed_mps), max_acceleration_mps2_(max_acceleration_mps2),
  integral_gain_s_inv_(integral_gain_s_inv), integral_limit_n_(integral_limit_n)
{
  if (!std::isfinite(mass_) || mass_ <= 0.0) {mass_ = 10.0;}
  if (!std::isfinite(damping_) || damping_ < 0.0) {damping_ = 80.0;}
  if (!std::isfinite(deadband_n_) || deadband_n_ < 0.0) {deadband_n_ = 0.5;}
  if (!std::isfinite(max_speed_mps_) || max_speed_mps_ <= 0.0) {max_speed_mps_ = 0.005;}
  if (!std::isfinite(max_acceleration_mps2_) || max_acceleration_mps2_ <= 0.0) {
    max_acceleration_mps2_ = 0.02;
  }
  if (!std::isfinite(integral_gain_s_inv_) || integral_gain_s_inv_ < 0.0) {
    integral_gain_s_inv_ = 0.25;
  }
  if (!std::isfinite(integral_limit_n_) || integral_limit_n_ < 0.0) {
    integral_limit_n_ = 3.0;
  }
}

void OneDimensionalAdmittance::reset()
{
  integral_state_n_s_ = 0.0;
  velocity_mps_ = 0.0;
}

double OneDimensionalAdmittance::update(double target_force_n, double actual_force_n, double dt_s)
{
  if (!std::isfinite(target_force_n) || !std::isfinite(actual_force_n) ||
    !std::isfinite(dt_s) || dt_s <= 0.0)
  {
    reset();
    return 0.0;
  }
  const double error = target_force_n - actual_force_n;
  // A compliant rope can change force much faster than the velocity ramp can
  // decelerate. Carrying the old velocity through the force deadband caused
  // the real FR5 to repeatedly cross a 15 N target and eventually overshoot
  // the old force fault. Hold immediately once the target band is reached.
  if (std::abs(error) <= deadband_n_) {
    integral_state_n_s_ = 0.0;
    velocity_mps_ = 0.0;
    return 0.0;
  }
  // If the force error has changed sign, the previous velocity is now moving
  // in the wrong direction. Drop that stale momentum before accelerating back
  // toward the target; this is a one-axis force controller, not a free mass.
  if (error * velocity_mps_ < 0.0) {
    integral_state_n_s_ = 0.0;
    velocity_mps_ = 0.0;
  }
  const bool saturated_positive = velocity_mps_ >= max_speed_mps_ && error > 0.0;
  const bool saturated_negative = velocity_mps_ <= -max_speed_mps_ && error < 0.0;
  if (!saturated_positive && !saturated_negative) {
    integral_state_n_s_ += error * dt_s;
  }
  const double integral_correction = std::clamp(
    integral_gain_s_inv_ * integral_state_n_s_, -integral_limit_n_, integral_limit_n_);
  double acceleration = (error + integral_correction - damping_ * velocity_mps_) / mass_;
  acceleration = std::clamp(acceleration, -max_acceleration_mps2_, max_acceleration_mps2_);
  velocity_mps_ = std::clamp(
    velocity_mps_ + acceleration * dt_s, -max_speed_mps_, max_speed_mps_);
  return velocity_mps_;
}

namespace
{

double median(std::vector<double> values)
{
  if (values.empty()) {return std::numeric_limits<double>::quiet_NaN();}
  const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
  std::nth_element(values.begin(), middle, values.end());
  double result = *middle;
  if (values.size() % 2 == 0) {
    const auto lower = std::max_element(values.begin(), middle);
    result = (*lower + *middle) * 0.5;
  }
  return result;
}

double percentile(std::vector<double> values, double fraction)
{
  if (values.empty()) {return std::numeric_limits<double>::quiet_NaN();}
  std::sort(values.begin(), values.end());
  const double index = std::clamp(fraction, 0.0, 1.0) * (values.size() - 1);
  const auto lower = static_cast<size_t>(std::floor(index));
  const auto upper = static_cast<size_t>(std::ceil(index));
  const double weight = index - lower;
  return values[lower] * (1.0 - weight) + values[upper] * weight;
}

}  // namespace

CalibrationResult robust_calibrate_direction(
  const std::vector<Vec3> & samples,
  std::size_t minimum_samples,
  double maximum_angle_p95_deg)
{
  CalibrationResult result;
  if (samples.size() < minimum_samples || minimum_samples == 0) {
    result.reason = "CALIBRATION_TOO_FEW_SAMPLES";
    return result;
  }
  std::vector<double> xs, ys, zs;
  xs.reserve(samples.size()); ys.reserve(samples.size()); zs.reserve(samples.size());
  for (const auto & sample : samples) {
    if (!finite(sample)) {
      result.reason = "CALIBRATION_NONFINITE_SAMPLE";
      return result;
    }
    xs.push_back(sample.x); ys.push_back(sample.y); zs.push_back(sample.z);
  }
  const Vec3 center{median(xs), median(ys), median(zs)};
  std::vector<double> residuals;
  residuals.reserve(samples.size());
  for (const auto & sample : samples) {
    residuals.push_back(norm(sample - center));
  }
  const double mad = median(residuals);
  const double threshold = std::max(0.5, 3.0 * mad);
  std::vector<Vec3> retained;
  retained.reserve(samples.size());
  for (size_t i = 0; i < samples.size(); ++i) {
    if (residuals[i] <= threshold) {retained.push_back(samples[i]);}
  }
  result.retained_fraction = static_cast<double>(retained.size()) / samples.size();
  if (retained.size() < minimum_samples || result.retained_fraction < 0.80) {
    result.reason = "CALIBRATION_RETAINED_FRACTION_LOW";
    return result;
  }
  Vec3 mean{};
  for (const auto & sample : retained) {
    mean = mean + sample;
  }
  mean = mean * (1.0 / retained.size());
  Vec3 mean_unit;
  if (!normalize(mean, mean_unit)) {
    result.reason = "CALIBRATION_MEAN_FORCE_INVALID";
    return result;
  }
  std::vector<double> angles;
  angles.reserve(retained.size());
  for (const auto & sample : retained) {
    Vec3 sample_unit;
    if (!normalize(sample, sample_unit)) {
      result.reason = "CALIBRATION_SAMPLE_FORCE_INVALID";
      return result;
    }
    const double cosine = std::clamp(dot(sample_unit, mean_unit), -1.0, 1.0);
    angles.push_back(std::acos(cosine) * 180.0 / kPi);
  }
  result.angle_p95_deg = percentile(angles, 0.95);
  if (!std::isfinite(result.angle_p95_deg) || result.angle_p95_deg > maximum_angle_p95_deg) {
    result.reason = "CALIBRATION_DIRECTION_VARIATION";
    return result;
  }
  result.direction = mean_unit;
  result.success = true;
  result.reason = "OK";
  return result;
}

}  // namespace fr_traction
