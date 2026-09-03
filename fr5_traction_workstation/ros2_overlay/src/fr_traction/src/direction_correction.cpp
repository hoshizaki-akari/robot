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

double percentile(std::vector<double> values, double fraction)
{
  if (values.empty()) {return std::numeric_limits<double>::quiet_NaN();}
  std::sort(values.begin(), values.end());
  const double index = std::clamp(fraction, 0.0, 1.0) * (values.size() - 1);
  const auto lower = static_cast<std::size_t>(std::floor(index));
  const auto upper = static_cast<std::size_t>(std::ceil(index));
  const double weight = index - static_cast<double>(lower);
  return values[lower] * (1.0 - weight) + values[upper] * weight;
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

Vec3 move_towards_on_sphere(
  const Vec3 & current, const Vec3 & target, double maximum_step_rad)
{
  Vec3 from;
  Vec3 to;
  if (!normalize(current, from) || !normalize(target, to) ||
    !std::isfinite(maximum_step_rad) || maximum_step_rad <= 0.0)
  {
    return from;
  }
  const double cosine = std::clamp(dot(from, to), -1.0, 1.0);
  const double angle = std::acos(cosine);
  if (!std::isfinite(angle) || angle <= maximum_step_rad) {return to;}

  Vec3 tangent = to - cosine * from;
  if (!normalize(tangent, tangent)) {
    DirectionBasis basis;
    if (!make_tangent_basis(from, basis)) {return from;}
    tangent = basis.b1;
  }
  Vec3 result = std::cos(maximum_step_rad) * from +
    std::sin(maximum_step_rad) * tangent;
  Vec3 unit;
  return normalize(result, unit) ? unit : from;
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
    case DirectionTrackState::AMBIGUOUS: return "AMBIGUOUS";
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
    config_.slow_cutoff_hz = 1.0;
  }
  if (config_.slow_cutoff_hz > config_.fast_cutoff_hz) {
    config_.slow_cutoff_hz = config_.fast_cutoff_hz;
  }
  if (config_.robust_window_size < 3) {config_.robust_window_size = 3;}
  if (config_.robust_window_size % 2 == 0) {++config_.robust_window_size;}
  if (!std::isfinite(config_.minimum_force_n) || config_.minimum_force_n <= 0.0) {
    config_.minimum_force_n = 0.5;
  }
  if (!std::isfinite(config_.recovery_confirm_s) || config_.recovery_confirm_s <= 0.0) {
    config_.recovery_confirm_s = 0.30;
  }
  if (!std::isfinite(config_.change_confirm_s) || config_.change_confirm_s <= 0.0) {
    config_.change_confirm_s = 0.20;
  }
  if (!std::isfinite(config_.settling_s) || config_.settling_s <= 0.0) {
    config_.settling_s = 0.50;
  }
  if (!std::isfinite(config_.candidate_max_dispersion_rad) ||
    config_.candidate_max_dispersion_rad <= 0.0)
  {
    config_.candidate_max_dispersion_rad = 8.0 * kDegree;
  }
  if (!std::isfinite(config_.ambiguity_timeout_s) || config_.ambiguity_timeout_s <= 0.0) {
    config_.ambiguity_timeout_s = 8.0;
  }
  if (!std::isfinite(config_.tracking_gain_s_inv) || config_.tracking_gain_s_inv <= 0.0) {
    config_.tracking_gain_s_inv = 5.0;
  }
  if (!std::isfinite(config_.tracking_max_rate_rad_s) ||
    config_.tracking_max_rate_rad_s <= 0.0)
  {
    config_.tracking_max_rate_rad_s = 180.0 * kDegree;
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
  candidate_direction_ = locked_direction_;
  tracked_direction_ = locked_direction_;
  filter_initialized_ = false;
  candidate_valid_ = false;
  candidate_confirmed_ = false;
  candidate_elapsed_s_ = 0.0;
  ambiguity_elapsed_s_ = 0.0;
  recovery_elapsed_s_ = 0.0;
  settling_elapsed_s_ = 0.0;
  recovering_from_hold_ = true;
  state_ = DirectionTrackState::SENSOR_HOLD;
}

bool DirectionEstimator::valid_direction_sample(
  const Vec3 & force, Vec3 & unit, double & tension) const
{
  tension = norm(force);
  if (!normalize(force, unit) || !std::isfinite(tension) ||
    tension < config_.minimum_force_n)
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

double DirectionEstimator::robust_dispersion(const Vec3 & center) const
{
  std::vector<double> distances;
  distances.reserve(direction_window_.size());
  for (const auto & sample : direction_window_) {
    const double angle = angle_between(sample, center);
    if (!std::isfinite(angle)) {return std::numeric_limits<double>::quiet_NaN();}
    distances.push_back(angle);
  }
  // The 90th percentile allows an occasional sensor spike while still
  // rejecting a window that alternates between incompatible directions.
  return percentile(distances, 0.90);
}

void DirectionEstimator::update_candidate_and_track(DirectionEstimate & estimate, double dt_s)
{
  estimate.entry_angle_rad = noise_profile_.valid ? noise_profile_.entry_angle_rad :
    3.0 * kDegree;
  estimate.exit_angle_rad = noise_profile_.valid ? noise_profile_.exit_angle_rad :
    1.5 * kDegree;
  if (!estimate.valid) {
    if (!recovering_from_hold_) {
      direction_window_.clear();
      fast_direction_ = {};
      slow_direction_ = {};
      filter_initialized_ = false;
    }
    recovering_from_hold_ = true;
    recovery_elapsed_s_ = 0.0;
    state_ = DirectionTrackState::SENSOR_HOLD;
    candidate_valid_ = false;
    candidate_confirmed_ = false;
    candidate_elapsed_s_ = 0.0;
    settling_elapsed_s_ = 0.0;
    estimate.tracked_direction = tracked_direction_;
    estimate.candidate_direction = candidate_direction_;
    estimate.ambiguity_elapsed_s = ambiguity_elapsed_s_;
    estimate.state = state_;
    return;
  }

  const double safe_dt = std::isfinite(dt_s) && dt_s > 0.0 ? dt_s : 0.0;
  const double consensus_limit = std::max(
    config_.candidate_max_dispersion_rad, 1.25 * estimate.entry_angle_rad);
  const bool consensus_stable =
    estimate.candidate_dispersion_rad <= consensus_limit;
  if (recovering_from_hold_) {
    if (consensus_stable) {
      recovery_elapsed_s_ += safe_dt;
      ambiguity_elapsed_s_ = std::max(0.0, ambiguity_elapsed_s_ - safe_dt);
    } else {
      recovery_elapsed_s_ = 0.0;
      ambiguity_elapsed_s_ += safe_dt;
    }
    if (ambiguity_elapsed_s_ >= config_.ambiguity_timeout_s) {
      state_ = DirectionTrackState::AMBIGUOUS;
      estimate.tracked_direction = tracked_direction_;
      estimate.candidate_direction = candidate_direction_;
      estimate.ambiguity_elapsed_s = ambiguity_elapsed_s_;
      estimate.ambiguity_timed_out = true;
      estimate.state = state_;
      return;
    }
    if (recovery_elapsed_s_ < config_.recovery_confirm_s) {
      state_ = DirectionTrackState::SENSOR_HOLD;
      estimate.tracked_direction = tracked_direction_;
      estimate.candidate_direction = candidate_direction_;
      estimate.ambiguity_elapsed_s = ambiguity_elapsed_s_;
      estimate.state = state_;
      return;
    }
    recovering_from_hold_ = false;
    recovery_elapsed_s_ = 0.0;
  }
  const double robust_error = angle_between(estimate.robust_direction, tracked_direction_);
  const bool change_evidence = std::isfinite(robust_error) &&
    robust_error > estimate.entry_angle_rad;
  bool rejected_confirmed_candidate = false;

  if (candidate_confirmed_) {
    const double candidate_jump = angle_between(
      estimate.robust_direction, candidate_direction_);
    if (!consensus_stable || !std::isfinite(candidate_jump) ||
      candidate_jump > 2.0 * consensus_limit)
    {
      candidate_valid_ = false;
      candidate_confirmed_ = false;
      candidate_elapsed_s_ = 0.0;
      settling_elapsed_s_ = 0.0;
      ambiguity_elapsed_s_ += safe_dt;
      state_ = ambiguity_elapsed_s_ >= config_.ambiguity_timeout_s ?
        DirectionTrackState::AMBIGUOUS : DirectionTrackState::SUSPECT;
      rejected_confirmed_candidate = true;
    } else {
      const double alpha = safe_alpha(config_.slow_cutoff_hz, safe_dt);
      candidate_direction_ = move_towards_on_sphere(
        candidate_direction_, estimate.robust_direction,
        std::max(1e-9, alpha * candidate_jump));
      const double remaining = angle_between(tracked_direction_, candidate_direction_);
      const double adaptive_rate = std::min(
        config_.tracking_max_rate_rad_s,
        config_.tracking_gain_s_inv * std::max(0.0, remaining));
      tracked_direction_ = move_towards_on_sphere(
        tracked_direction_, candidate_direction_, adaptive_rate * safe_dt);
      const double remaining_after_step = angle_between(
        tracked_direction_, candidate_direction_);
      if (remaining_after_step > estimate.exit_angle_rad) {
        settling_elapsed_s_ = 0.0;
        state_ = DirectionTrackState::CORRECTING;
      } else {
        settling_elapsed_s_ += safe_dt;
        state_ = DirectionTrackState::SETTLING;
        if (settling_elapsed_s_ >= config_.settling_s) {
          state_ = DirectionTrackState::STABLE;
          candidate_valid_ = false;
          candidate_confirmed_ = false;
          candidate_elapsed_s_ = 0.0;
          ambiguity_elapsed_s_ = 0.0;
        }
      }
    }
  }

  if (!candidate_confirmed_ && !rejected_confirmed_candidate) {
    if (change_evidence && consensus_stable) {
      const double candidate_jump = candidate_valid_ ?
        angle_between(estimate.robust_direction, candidate_direction_) : 0.0;
      if (!candidate_valid_ || !std::isfinite(candidate_jump) ||
        candidate_jump > 2.0 * consensus_limit)
      {
        if (candidate_valid_) {ambiguity_elapsed_s_ += safe_dt;}
        candidate_direction_ = estimate.robust_direction;
        candidate_elapsed_s_ = safe_dt;
      } else {
        const double alpha = safe_alpha(config_.slow_cutoff_hz, safe_dt);
        candidate_direction_ = move_towards_on_sphere(
          candidate_direction_, estimate.robust_direction,
          std::max(1e-9, alpha * candidate_jump));
        candidate_elapsed_s_ += safe_dt;
      }
      candidate_valid_ = true;
      ambiguity_elapsed_s_ = std::max(0.0, ambiguity_elapsed_s_ - safe_dt);
      candidate_confirmed_ = candidate_elapsed_s_ >= config_.change_confirm_s;
      state_ = candidate_confirmed_ ?
        DirectionTrackState::CORRECTING : DirectionTrackState::SUSPECT;
    } else if (change_evidence) {
      candidate_valid_ = false;
      candidate_elapsed_s_ = 0.0;
      settling_elapsed_s_ = 0.0;
      ambiguity_elapsed_s_ += safe_dt;
      state_ = ambiguity_elapsed_s_ >= config_.ambiguity_timeout_s ?
        DirectionTrackState::AMBIGUOUS : DirectionTrackState::SUSPECT;
    } else {
      candidate_valid_ = false;
      candidate_elapsed_s_ = 0.0;
      // The entry/exit thresholds are hysteresis, not a third no-motion
      // region. A robust direction below the entry threshold is ordinary
      // tolerated wander, even if the faster filter has not yet returned
      // inside the tighter exit threshold. The former SUSPECT assignment
      // could therefore pause force control forever in this band (seen for
      // 15.8 s in a real run). Exit is only relevant after a confirmed
      // correction while settling onto the accepted new direction.
      settling_elapsed_s_ = 0.0;
      ambiguity_elapsed_s_ = std::max(0.0, ambiguity_elapsed_s_ - safe_dt);
      state_ = DirectionTrackState::STABLE;
    }
  }
  estimate.tracked_direction = tracked_direction_;
  estimate.candidate_direction = candidate_direction_;
  estimate.candidate_elapsed_s = candidate_elapsed_s_;
  estimate.ambiguity_elapsed_s = ambiguity_elapsed_s_;
  estimate.candidate_confirmed = candidate_confirmed_;
  estimate.ambiguity_timed_out = state_ == DirectionTrackState::AMBIGUOUS;
  estimate.state = state_;
}

DirectionEstimate DirectionEstimator::update(const Vec3 & force, double dt_s)
{
  DirectionEstimate estimate;
  estimate.locked_direction = locked_direction_;
  estimate.tracked_direction = tracked_direction_;
  estimate.candidate_direction = candidate_direction_;
  estimate.entry_angle_rad = noise_profile_.valid ? noise_profile_.entry_angle_rad :
    3.0 * kDegree;
  estimate.exit_angle_rad = noise_profile_.valid ? noise_profile_.exit_angle_rad :
    1.5 * kDegree;
  estimate.tension_n = norm(force);
  if (finite(force) && std::isfinite(estimate.tension_n) && estimate.tension_n > 1e-12) {
    normalize(force, estimate.raw_direction);
    estimate.axial_force_n = dot(force, tracked_direction_);
    estimate.lateral_force_vector = force - estimate.axial_force_n * tracked_direction_;
    estimate.lateral_force_n = norm(estimate.lateral_force_vector);
    estimate.angle_to_locked_rad = angle_between(estimate.raw_direction, locked_direction_);
  }
  Vec3 raw_unit;
  double tension = 0.0;
  if (!valid_direction_sample(force, raw_unit, tension)) {
    estimate.valid = false;
    estimate.fast_direction = fast_direction_;
    estimate.slow_direction = slow_direction_;
    update_candidate_and_track(estimate, dt_s);
    return estimate;
  }

  const Vec3 previous_fast = fast_direction_;
  direction_window_.push_back(raw_unit);
  while (direction_window_.size() > config_.robust_window_size) {direction_window_.pop_front();}
  const Vec3 robust = robust_center();
  if (!finite(robust) || norm(robust) < 0.5) {
    estimate.valid = false;
    update_candidate_and_track(estimate, dt_s);
    return estimate;
  }
  estimate.robust_direction = robust;
  estimate.candidate_dispersion_rad = robust_dispersion(robust);
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
      update_candidate_and_track(estimate, dt_s);
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
  estimate.fast_angle_rad = angle_between(fast_direction_, tracked_direction_);
  estimate.slow_angle_rad = angle_between(slow_direction_, locked_direction_);
  estimate.fast_slow_angle_rad = angle_between(fast_direction_, slow_direction_);
  update_candidate_and_track(estimate, dt_s);
  estimate.fast_angle_rad = angle_between(fast_direction_, estimate.tracked_direction);
  estimate.axial_force_n = dot(force, estimate.tracked_direction);
  estimate.lateral_force_vector = force - estimate.axial_force_n * estimate.tracked_direction;
  estimate.lateral_force_n = norm(estimate.lateral_force_vector);
  return estimate;
}

AdaptiveDirectionFollower::AdaptiveDirectionFollower(const AdaptiveFollowConfig & config)
: config_(config)
{
  if (!std::isfinite(config_.speed_gain_mps_per_rad) ||
    config_.speed_gain_mps_per_rad <= 0.0)
  {
    config_.speed_gain_mps_per_rad = 0.020;
  }
  if (!std::isfinite(config_.maximum_speed_mps) || config_.maximum_speed_mps <= 0.0) {
    config_.maximum_speed_mps = 0.020;
  }
  if (!std::isfinite(config_.maximum_acceleration_mps2) ||
    config_.maximum_acceleration_mps2 <= 0.0)
  {
    config_.maximum_acceleration_mps2 = 0.10;
  }
}

void AdaptiveDirectionFollower::reset()
{
  speed_mps_ = 0.0;
  accumulated_displacement_m_ = 0.0;
}

AdaptiveFollowResult AdaptiveDirectionFollower::update(
  const DirectionEstimate & estimate, bool active, double dt_s)
{
  AdaptiveFollowResult result;
  result.accumulated_displacement_m = accumulated_displacement_m_;
  if (!estimate.valid || !finite(estimate.tracked_direction) ||
    !finite(estimate.candidate_direction) || !std::isfinite(dt_s) || dt_s <= 0.0)
  {
    speed_mps_ = 0.0;
    result.reason = !estimate.valid ? "SENSOR_HOLD" : "INVALID_INPUT";
    return result;
  }
  result.valid = true;
  if (!estimate.candidate_confirmed ||
    (estimate.state != DirectionTrackState::CORRECTING &&
    estimate.state != DirectionTrackState::SETTLING))
  {
    speed_mps_ = 0.0;
    result.reason = estimate.state == DirectionTrackState::AMBIGUOUS ?
      "DIRECTION_AMBIGUOUS" : "DIRECTION_STABLE_OR_UNCONFIRMED";
    return result;
  }

  result.angle_error_rad = angle_between(
    estimate.tracked_direction, estimate.candidate_direction);
  if (!std::isfinite(result.angle_error_rad) ||
    result.angle_error_rad <= estimate.exit_angle_rad)
  {
    speed_mps_ = 0.0;
    result.reason = "ANGULAR_DEADBAND";
    return result;
  }

  Vec3 tangent = estimate.candidate_direction -
    dot(estimate.candidate_direction, estimate.tracked_direction) *
    estimate.tracked_direction;
  Vec3 tangent_unit;
  if (!normalize(tangent, tangent_unit)) {
    speed_mps_ = 0.0;
    result.reason = "TANGENT_UNDEFINED";
    return result;
  }

  result.requested_speed_mps = config_.speed_gain_mps_per_rad *
    std::max(0.0, result.angle_error_rad - estimate.exit_angle_rad);
  const double bounded_request = std::min(
    result.requested_speed_mps, config_.maximum_speed_mps);
  result.limited = result.requested_speed_mps > config_.maximum_speed_mps;
  const double maximum_delta = config_.maximum_acceleration_mps2 * dt_s;
  speed_mps_ += std::clamp(bounded_request - speed_mps_, -maximum_delta, maximum_delta);
  speed_mps_ = std::clamp(speed_mps_, 0.0, config_.maximum_speed_mps);
  if (!active) {
    result.reason = "SHADOW_ONLY";
    return result;
  }

  result.active = true;
  result.applied_speed_mps = speed_mps_;
  result.velocity_base = tangent_unit * speed_mps_;
  accumulated_displacement_m_ += speed_mps_ * dt_s;
  result.accumulated_displacement_m = accumulated_displacement_m_;
  result.reason = result.limited ? "ACTIVE_ADAPTIVE_LIMITED" : "ACTIVE_ADAPTIVE";
  return result;
}

}  // namespace fr_traction
