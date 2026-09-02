#ifndef FR_TRACTION__DIRECTION_CORRECTION_HPP_
#define FR_TRACTION__DIRECTION_CORRECTION_HPP_

#include <cstddef>
#include <deque>
#include <string>
#include <vector>

#include "fr_traction/traction_math.hpp"

namespace fr_traction
{

// The tangent frame is attached to the direction locked during manual setup.
// b1 and b2 are orthonormal and both perpendicular to normal.
struct DirectionBasis
{
  bool valid = false;
  Vec3 normal;
  Vec3 b1;
  Vec3 b2;
};

bool make_tangent_basis(const Vec3 & direction, DirectionBasis & result);

struct DirectionNoiseProfile
{
  bool valid = false;
  double median_angle_rad = 0.0;
  double mad_angle_rad = 0.0;
  double entry_angle_rad = 0.05235987755982989;  // 3 degrees
  double exit_angle_rad = 0.026179938779914945;  // 1.5 degrees
};

// Estimate a noise band from the stationary force samples collected while
// the operator confirms the direction. This is deliberately based on angles,
// not force magnitude, so a small tension fluctuation does not look like a
// direction change.
DirectionNoiseProfile estimate_direction_noise(
  const std::vector<Vec3> & samples,
  const Vec3 & locked_direction,
  double minimum_force_n = 0.5,
  double minimum_entry_angle_rad = 0.05235987755982989,
  double maximum_entry_angle_rad = 0.3490658503988659);

enum class DirectionTrackState : unsigned char
{
  STABLE = 0,
  SUSPECT = 1,
  CORRECTING = 2,
  SETTLING = 3,
  SENSOR_HOLD = 4,
  AMBIGUOUS = 5
};

const char * direction_track_state_name(DirectionTrackState state);

struct DirectionFilterConfig
{
  double fast_cutoff_hz = 4.0;
  double slow_cutoff_hz = 1.0;
  std::size_t robust_window_size = 11;
  double minimum_force_n = 0.5;
  double recovery_confirm_s = 0.30;
  double change_confirm_s = 0.20;
  double settling_s = 0.50;
  double candidate_max_dispersion_rad = 0.13962634015954636;  // 8 degrees
  double ambiguity_timeout_s = 8.0;
  double tracking_gain_s_inv = 5.0;
  double tracking_max_rate_rad_s = 3.14159265358979323846;  // 180 deg/s guard
};

struct DirectionEstimate
{
  bool valid = false;
  bool raw_outlier = false;
  Vec3 locked_direction;
  Vec3 raw_direction;
  Vec3 robust_direction;
  Vec3 fast_direction;
  Vec3 slow_direction;
  Vec3 candidate_direction;
  Vec3 tracked_direction;
  Vec3 lateral_force_vector;
  double tension_n = 0.0;
  double axial_force_n = 0.0;
  double lateral_force_n = 0.0;
  double angle_to_locked_rad = 0.0;
  double fast_angle_rad = 0.0;
  double slow_angle_rad = 0.0;
  double fast_slow_angle_rad = 0.0;
  double candidate_dispersion_rad = 0.0;
  double candidate_elapsed_s = 0.0;
  double ambiguity_elapsed_s = 0.0;
  double entry_angle_rad = 0.05235987755982989;
  double exit_angle_rad = 0.026179938779914945;
  bool candidate_confirmed = false;
  bool ambiguity_timed_out = false;
  DirectionTrackState state = DirectionTrackState::SENSOR_HOLD;
};

class DirectionEstimator
{
public:
  DirectionEstimator(
    const Vec3 & locked_direction,
    const DirectionFilterConfig & config = DirectionFilterConfig{});

  bool set_locked_direction(const Vec3 & locked_direction);
  void set_noise_profile(const DirectionNoiseProfile & profile);
  void reset();
  DirectionEstimate update(const Vec3 & force, double dt_s);

  const DirectionBasis & basis() const {return basis_;}
  const DirectionNoiseProfile & noise_profile() const {return noise_profile_;}

private:
  Vec3 robust_center() const;
  double robust_dispersion(const Vec3 & center) const;
  bool valid_direction_sample(const Vec3 & force, Vec3 & unit, double & tension) const;
  void update_candidate_and_track(DirectionEstimate & estimate, double dt_s);

  DirectionFilterConfig config_;
  DirectionBasis basis_;
  DirectionNoiseProfile noise_profile_;
  std::deque<Vec3> direction_window_;
  Vec3 locked_direction_;
  Vec3 fast_direction_;
  Vec3 slow_direction_;
  Vec3 candidate_direction_;
  Vec3 tracked_direction_;
  bool filter_initialized_ = false;
  bool candidate_valid_ = false;
  bool candidate_confirmed_ = false;
  double candidate_elapsed_s_ = 0.0;
  double ambiguity_elapsed_s_ = 0.0;
  double recovery_elapsed_s_ = 0.0;
  double settling_elapsed_s_ = 0.0;
  bool recovering_from_hold_ = true;
  DirectionTrackState state_ = DirectionTrackState::SENSOR_HOLD;
};

// Converts an angular difference into a Cartesian following velocity. The
// speed is proportional to the remaining angle, so a large change is chased
// quickly and the motion naturally slows as the new direction is reached.
// The accumulated distance is diagnostic only; it is deliberately not a
// motion limit because the accepted traction direction may cover 360 degrees.
struct AdaptiveFollowConfig
{
  double speed_gain_mps_per_rad = 0.020;
  double maximum_speed_mps = 0.020;
  double maximum_acceleration_mps2 = 0.10;
};

struct AdaptiveFollowResult
{
  bool valid = false;
  bool active = false;
  bool limited = false;
  double angle_error_rad = 0.0;
  double requested_speed_mps = 0.0;
  double applied_speed_mps = 0.0;
  double accumulated_displacement_m = 0.0;
  Vec3 velocity_base;
  std::string reason;
};

class AdaptiveDirectionFollower
{
public:
  explicit AdaptiveDirectionFollower(
    const AdaptiveFollowConfig & config = AdaptiveFollowConfig{});

  void reset();
  AdaptiveFollowResult update(
    const DirectionEstimate & estimate,
    bool active,
    double dt_s);

private:
  AdaptiveFollowConfig config_;
  double speed_mps_ = 0.0;
  double accumulated_displacement_m_ = 0.0;
};

}  // namespace fr_traction

#endif  // FR_TRACTION__DIRECTION_CORRECTION_HPP_
