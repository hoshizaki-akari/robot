#include "fr_traction/direction_correction.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace fr_traction
{

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegree = kPi / 180.0;

Vec3 cross(const Vec3 & lhs, const Vec3 & rhs)
{
  return {
    lhs.y * rhs.z - lhs.z * rhs.y,
    lhs.z * rhs.x - lhs.x * rhs.z,
    lhs.x * rhs.y - lhs.y * rhs.x};
}

double angle_between(const Vec3 & lhs, const Vec3 & rhs)
{
  Vec3 lhs_unit;
  Vec3 rhs_unit;
  if (!normalize(lhs, lhs_unit) || !normalize(rhs, rhs_unit)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return std::acos(std::clamp(dot(lhs_unit, rhs_unit), -1.0, 1.0));
}

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

double safe_alpha(double cutoff_hz, double dt_s)
{
  if (!std::isfinite(cutoff_hz) || cutoff_hz <= 0.0 ||
    !std::isfinite(dt_s) || dt_s <= 0.0)
  {
    return 1.0;
  }
  return std::clamp(1.0 - std::exp(-2.0 * kPi * cutoff_hz * dt_s), 0.0, 1.0);
}

}  // namespace

bool make_tangent_basis(const Vec3 & direction, DirectionBasis & result)
{
  result = {};
  Vec3 normal;
  if (!normalize(direction, normal)) {return false;}

  // Choose the world axis least aligned with the locked direction. This keeps
  // the cross product well-conditioned even if the cable points near a world
  // axis, while making the basis deterministic for repeatable logs.
  const Vec3 axes[] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
  const Vec3 * reference = &axes[0];
  double smallest_alignment = std::abs(dot(normal, axes[0]));
  for (const auto & axis : axes) {
    const double alignment = std::abs(dot(normal, axis));
    if (alignment < smallest_alignment) {
      smallest_alignment = alignment;
      reference = &axis;
    }
  }
  Vec3 b1;
  if (!normalize(cross(normal, *reference), b1)) {return false;}
  const Vec3 b2 = cross(normal, b1);
  if (!finite(b2) || std::abs(norm(b2) - 1.0) > 1e-9) {return false;}
  result.valid = true;
  result.normal = normal;
  result.b1 = b1;
  result.b2 = b2;
  return true;
}

DirectionNoiseProfile estimate_direction_noise(
  const std::vector<Vec3> & samples,
  const Vec3 & locked_direction,
  double minimum_force_n,
  double minimum_entry_angle_rad,
  double maximum_entry_angle_rad)
{
  DirectionNoiseProfile result;
  Vec3 locked;
  if (!normalize(locked_direction, locked) ||
    !std::isfinite(minimum_force_n) || minimum_force_n <= 0.0 ||
    !std::isfinite(minimum_entry_angle_rad) || minimum_entry_angle_rad <= 0.0 ||
    !std::isfinite(maximum_entry_angle_rad) ||
    maximum_entry_angle_rad < minimum_entry_angle_rad)
  {
    return result;
  }

  std::vector<double> angles;
  angles.reserve(samples.size());
  for (const auto & sample : samples) {
    Vec3 unit;
    if (!normalize(sample, unit) || norm(sample) < minimum_force_n || dot(unit, locked) <= 0.0) {
      continue;
    }
    angles.push_back(std::acos(std::clamp(dot(unit, locked), -1.0, 1.0)));
  }
  if (angles.size() < 3) {return result;}

  const double center = median(angles);
  std::vector<double> deviations;
  deviations.reserve(angles.size());
  for (const double angle : angles) {
    deviations.push_back(std::abs(angle - center));
  }
  const double mad = median(deviations);
  if (!std::isfinite(center) || !std::isfinite(mad)) {return result;}

  // 1.4826 * MAD estimates the Gaussian standard deviation. Four sigma is
  // used for entering correction; the hard floor prevents sensor quantization
  // from making the controller chase sub-degree movement.
  const double robust_sigma = 1.4826 * mad;
  const double entry = std::clamp(
    std::max(minimum_entry_angle_rad, center + 4.0 * robust_sigma),
    minimum_entry_angle_rad, maximum_entry_angle_rad);
  result.valid = true;
  result.median_angle_rad = center;
  result.mad_angle_rad = mad;
  result.entry_angle_rad = entry;
  result.exit_angle_rad = std::max(1.5 * kDegree, 0.5 * entry);
  return result;
}

const char * direction_track_state_name(DirectionTrackState state)
{
  switch (state) {
    case DirectionTrackState::STABLE: return "STABLE";
    case DirectionTrackState::SUSPECT: return "SUSPECT";
    case DirectionTrackState::CORRECTING: return "CORRECTING";
    case DirectionTrackState::SETTLING: return "SETTLING";
    case DirectionTrackState::SENSOR_HOLD: return "SENSOR_HOLD";
  }
  return "UNKNOWN";
}

DirectionEstimator::DirectionEstimator(
  const Vec3 & locked_direction,
  const DirectionFilterConfig & config)
: config_(config)
{
  if (!std::isfinite(config_.fast_cutoff_hz) || config_.fast_cutoff_hz <= 0.0) {
    config_.fast_cutoff_hz = 4.0;
  }
  if (!std::isfinite(config_.slow_cutoff_hz) || config_.slow_cutoff_hz <= 0.0) {
    config_.slow_cutoff_hz = 0.6;
  }
  if (config_.slow_cutoff_hz > config_.fast_cutoff_hz) {
    config_.slow_cutoff_hz = config_.fast_cutoff_hz;
  }
  if (config_.robust_window_size < 3) {config_.robust_window_size = 3;}
  if (config_.robust_window_size % 2 == 0) {++config_.robust_window_size;}
  if (!std::isfinite(config_.minimum_force_n) || config_.minimum_force_n <= 0.0) {
    config_.minimum_force_n = 0.5;
  }
  if (!std::isfinite(config_.minimum_forward_cosine) ||
    config_.minimum_forward_cosine <= 0.0 || config_.minimum_forward_cosine >= 1.0)
  {
    config_.minimum_forward_cosine = 0.20;
  }
  if (!std::isfinite(config_.change_confirm_s) || config_.change_confirm_s <= 0.0) {
    config_.change_confirm_s = 0.15;
  }
  if (!std::isfinite(config_.settling_s) || config_.settling_s <= 0.0) {
    config_.settling_s = 0.25;
  }
  set_locked_direction(locked_direction);
}

bool DirectionEstimator::set_locked_direction(const Vec3 & locked_direction)
{
  DirectionBasis basis;
  if (!make_tangent_basis(locked_direction, basis)) {
    basis_ = {};
    return false;
  }
  locked_direction_ = basis.normal;
  basis_ = basis;
  reset();
  return true;
}

void DirectionEstimator::set_noise_profile(const DirectionNoiseProfile & profile)
{
  if (!profile.valid || !std::isfinite(profile.entry_angle_rad) ||
    !std::isfinite(profile.exit_angle_rad) || profile.entry_angle_rad <= 0.0 ||
    profile.exit_angle_rad <= 0.0 || profile.exit_angle_rad >= profile.entry_angle_rad)
  {
    noise_profile_ = {};
    return;
  }
  noise_profile_ = profile;
}

void DirectionEstimator::reset()
{
  direction_window_.clear();
  fast_direction_ = {};
  slow_direction_ = {};
  filter_initialized_ = false;
  change_candidate_s_ = 0.0;
  settling_elapsed_s_ = 0.0;
  state_ = DirectionTrackState::SENSOR_HOLD;
}

bool DirectionEstimator::valid_direction_sample(
  const Vec3 & force, Vec3 & unit, double & tension) const
{
  tension = norm(force);
  if (!normalize(force, unit) || !std::isfinite(tension) ||
    tension < config_.minimum_force_n ||
    dot(unit, locked_direction_) < config_.minimum_forward_cosine)
  {
    return false;
  }
  return true;
}

Vec3 DirectionEstimator::robust_center() const
{
  if (direction_window_.empty()) {return {};}
  std::size_t best_index = 0;
  double best_cost = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < direction_window_.size(); ++i) {
    double cost = 0.0;
    for (std::size_t j = 0; j < direction_window_.size(); ++j) {
      const double angle = angle_between(direction_window_[i], direction_window_[j]);
      if (!std::isfinite(angle)) {return {};}
      cost += angle;
    }
    if (cost < best_cost) {
      best_cost = cost;
      best_index = i;
    }
  }
  return direction_window_[best_index];
}

void DirectionEstimator::update_track_state(DirectionEstimate & estimate, double dt_s)
{
  estimate.entry_angle_rad = noise_profile_.valid ? noise_profile_.entry_angle_rad :
    3.0 * kDegree;
  estimate.exit_angle_rad = noise_profile_.valid ? noise_profile_.exit_angle_rad :
    1.5 * kDegree;
  if (!estimate.valid) {
    state_ = DirectionTrackState::SENSOR_HOLD;
    change_candidate_s_ = 0.0;
    settling_elapsed_s_ = 0.0;
    estimate.state = state_;
    return;
  }

  const bool evidence = estimate.fast_angle_rad > estimate.entry_angle_rad ||
    estimate.fast_slow_angle_rad > estimate.entry_angle_rad;
  const bool settled = estimate.fast_angle_rad <= estimate.exit_angle_rad &&
    estimate.fast_slow_angle_rad <= estimate.exit_angle_rad;
  const double safe_dt = std::isfinite(dt_s) && dt_s > 0.0 ? dt_s : 0.0;
  if (evidence) {
    change_candidate_s_ += safe_dt;
    settling_elapsed_s_ = 0.0;
    state_ = change_candidate_s_ >= config_.change_confirm_s ?
      DirectionTrackState::CORRECTING : DirectionTrackState::SUSPECT;
  } else if (settled) {
    change_candidate_s_ = 0.0;
    if (state_ == DirectionTrackState::CORRECTING || state_ == DirectionTrackState::SETTLING) {
      settling_elapsed_s_ += safe_dt;
      state_ = settling_elapsed_s_ >= config_.settling_s ?
        DirectionTrackState::STABLE : DirectionTrackState::SETTLING;
    } else {
      settling_elapsed_s_ = 0.0;
      state_ = DirectionTrackState::STABLE;
    }
  } else {
    change_candidate_s_ = 0.0;
    settling_elapsed_s_ = 0.0;
    state_ = DirectionTrackState::SUSPECT;
  }
  estimate.state = state_;
}

DirectionEstimate DirectionEstimator::update(const Vec3 & force, double dt_s)
{
  DirectionEstimate estimate;
  estimate.locked_direction = locked_direction_;
  estimate.entry_angle_rad = noise_profile_.valid ? noise_profile_.entry_angle_rad :
    3.0 * kDegree;
  estimate.exit_angle_rad = noise_profile_.valid ? noise_profile_.exit_angle_rad :
    1.5 * kDegree;
  estimate.tension_n = norm(force);
  if (finite(force) && std::isfinite(estimate.tension_n) && estimate.tension_n > 1e-12) {
    normalize(force, estimate.raw_direction);
    estimate.axial_force_n = dot(force, locked_direction_);
    estimate.lateral_force_vector = force - estimate.axial_force_n * locked_direction_;
    estimate.lateral_force_n = norm(estimate.lateral_force_vector);
    estimate.angle_to_locked_rad = angle_between(estimate.raw_direction, locked_direction_);
  }
  Vec3 raw_unit;
  double tension = 0.0;
  if (!valid_direction_sample(force, raw_unit, tension)) {
    estimate.valid = false;
    estimate.fast_direction = fast_direction_;
    estimate.slow_direction = slow_direction_;
    update_track_state(estimate, dt_s);
    return estimate;
  }

  const Vec3 previous_fast = fast_direction_;
  direction_window_.push_back(raw_unit);
  while (direction_window_.size() > config_.robust_window_size) {direction_window_.pop_front();}
  const Vec3 robust = robust_center();
  if (!finite(robust) || norm(robust) < 0.5) {
    estimate.valid = false;
    update_track_state(estimate, dt_s);
    return estimate;
  }
  estimate.robust_direction = robust;
  if (filter_initialized_) {
    const double raw_jump = angle_between(raw_unit, previous_fast);
    const double robust_jump = angle_between(robust, previous_fast);
    estimate.raw_outlier = std::isfinite(raw_jump) && std::isfinite(robust_jump) &&
      raw_jump > std::max(20.0 * kDegree, estimate.entry_angle_rad * 3.0) &&
      robust_jump < estimate.entry_angle_rad;
    const double fast_alpha = safe_alpha(config_.fast_cutoff_hz, dt_s);
    const double slow_alpha = safe_alpha(config_.slow_cutoff_hz, dt_s);
    fast_direction_ = fast_direction_ + (robust - fast_direction_) * fast_alpha;
    slow_direction_ = slow_direction_ + (robust - slow_direction_) * slow_alpha;
    Vec3 normalized_fast;
    Vec3 normalized_slow;
    if (!normalize(
        fast_direction_,
        normalized_fast) || !normalize(slow_direction_, normalized_slow))
    {
      estimate.valid = false;
      update_track_state(estimate, dt_s);
      return estimate;
    }
    fast_direction_ = normalized_fast;
    slow_direction_ = normalized_slow;
  } else {
    fast_direction_ = robust;
    slow_direction_ = robust;
    filter_initialized_ = true;
  }
  estimate.valid = true;
  estimate.fast_direction = fast_direction_;
  estimate.slow_direction = slow_direction_;
  estimate.fast_angle_rad = angle_between(fast_direction_, locked_direction_);
  estimate.slow_angle_rad = angle_between(slow_direction_, locked_direction_);
  estimate.fast_slow_angle_rad = angle_between(fast_direction_, slow_direction_);
  update_track_state(estimate, dt_s);
  return estimate;
}

DampedLateralController::DampedLateralController(
  const Vec3 & locked_direction,
  const LateralResponseJacobian & jacobian,
  const LateralCorrectionConfig & config)
: jacobian_(jacobian), config_(config)
{
  if (!std::isfinite(config_.position_gain_s_inv) || config_.position_gain_s_inv <= 0.0) {
    config_.position_gain_s_inv = 0.30;
  }
  if (!std::isfinite(config_.damping_n_per_m) || config_.damping_n_per_m <= 0.0) {
    config_.damping_n_per_m = 0.50;
  }
  if (!std::isfinite(config_.minimum_lateral_force_n) || config_.minimum_lateral_force_n < 0.0) {
    config_.minimum_lateral_force_n = 0.03;
  }
  if (!std::isfinite(config_.maximum_speed_mps) || config_.maximum_speed_mps <= 0.0) {
    config_.maximum_speed_mps = 0.0005;
  }
  if (!std::isfinite(config_.maximum_total_displacement_m) ||
    config_.maximum_total_displacement_m <= 0.0)
  {
    config_.maximum_total_displacement_m = 0.003;
  }
  make_tangent_basis(locked_direction, basis_);
}

void DampedLateralController::reset()
{
  accumulated_b1_m_ = 0.0;
  accumulated_b2_m_ = 0.0;
}

bool DampedLateralController::solve_damped(
  const double e1, const double e2, double & q1, double & q2) const
{
  if (!jacobian_.valid || !std::isfinite(e1) || !std::isfinite(e2)) {return false;}
  const double j11 = jacobian_.j11_n_per_m;
  const double j12 = jacobian_.j12_n_per_m;
  const double j21 = jacobian_.j21_n_per_m;
  const double j22 = jacobian_.j22_n_per_m;
  const double lambda2 = config_.damping_n_per_m * config_.damping_n_per_m;
  const double a11 = j11 * j11 + j12 * j12 + lambda2;
  const double a12 = j11 * j21 + j12 * j22;
  const double a22 = j21 * j21 + j22 * j22 + lambda2;
  const double determinant = a11 * a22 - a12 * a12;
  if (!std::isfinite(determinant) || determinant <= 1e-12) {return false;}
  const double z1 = (a22 * e1 - a12 * e2) / determinant;
  const double z2 = (-a12 * e1 + a11 * e2) / determinant;
  q1 = j11 * z1 + j21 * z2;
  q2 = j12 * z1 + j22 * z2;
  return std::isfinite(q1) && std::isfinite(q2);
}

LateralCorrectionResult DampedLateralController::update(
  const DirectionEstimate & estimate,
  bool active,
  double dt_s)
{
  LateralCorrectionResult result;
  result.accumulated_displacement_m =
    std::hypot(accumulated_b1_m_, accumulated_b2_m_);
  if (!basis_.valid || !estimate.valid || !std::isfinite(dt_s) || dt_s <= 0.0) {
    result.reason = !basis_.valid ? "INVALID_TANGENT_BASIS" :
      (!estimate.valid ? "SENSOR_HOLD" : "INVALID_DT");
    return result;
  }
  result.valid = true;
  result.error_b1_n = dot(estimate.lateral_force_vector, basis_.b1);
  result.error_b2_n = dot(estimate.lateral_force_vector, basis_.b2);
  const double error_norm = std::hypot(result.error_b1_n, result.error_b2_n);
  if (!std::isfinite(error_norm)) {
    result.valid = false;
    result.reason = "NONFINITE_LATERAL_ERROR";
    return result;
  }
  if (error_norm <= config_.minimum_lateral_force_n) {
    result.reason = "LATERAL_DEADBAND";
    return result;
  }
  if (estimate.state != DirectionTrackState::CORRECTING &&
    estimate.state != DirectionTrackState::SETTLING)
  {
    result.reason = "DIRECTION_NOT_CONFIRMED";
    return result;
  }
  double q1 = 0.0;
  double q2 = 0.0;
  if (!solve_damped(result.error_b1_n, result.error_b2_n, q1, q2)) {
    result.valid = false;
    result.reason = "INVALID_JACOBIAN";
    return result;
  }
  double v1 = -config_.position_gain_s_inv * q1;
  double v2 = -config_.position_gain_s_inv * q2;
  result.requested_speed_mps = std::hypot(v1, v2);
  if (!std::isfinite(result.requested_speed_mps)) {
    result.valid = false;
    result.reason = "NONFINITE_REQUEST";
    return result;
  }
  if (result.requested_speed_mps > config_.maximum_speed_mps) {
    const double scale = config_.maximum_speed_mps / result.requested_speed_mps;
    v1 *= scale;
    v2 *= scale;
    result.limited = true;
  }
  if (!active) {
    result.reason = "SHADOW_ONLY";
    return result;
  }

  const double proposed_b1 = accumulated_b1_m_ + v1 * dt_s;
  const double proposed_b2 = accumulated_b2_m_ + v2 * dt_s;
  const double proposed_norm = std::hypot(proposed_b1, proposed_b2);
  if (proposed_norm > config_.maximum_total_displacement_m) {
    if (proposed_norm > 1e-12) {
      const double scale = config_.maximum_total_displacement_m / proposed_norm;
      const double limited_b1 = scale * proposed_b1;
      const double limited_b2 = scale * proposed_b2;
      v1 = (limited_b1 - accumulated_b1_m_) / dt_s;
      v2 = (limited_b2 - accumulated_b2_m_) / dt_s;
      result.limited = true;
    } else {
      v1 = 0.0;
      v2 = 0.0;
    }
  }
  accumulated_b1_m_ += v1 * dt_s;
  accumulated_b2_m_ += v2 * dt_s;
  result.applied_speed_mps = std::hypot(v1, v2);
  result.accumulated_displacement_m =
    std::hypot(accumulated_b1_m_, accumulated_b2_m_);
  result.velocity_base = basis_.b1 * v1 + basis_.b2 * v2;
  result.reason = result.limited ? "ACTIVE_LIMITED" : "ACTIVE";
  return result;
}

}  // namespace fr_traction
