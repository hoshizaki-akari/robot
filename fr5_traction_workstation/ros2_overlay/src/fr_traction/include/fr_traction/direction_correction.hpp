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
  SENSOR_HOLD = 4
};

const char * direction_track_state_name(DirectionTrackState state);

struct DirectionFilterConfig
{
  double fast_cutoff_hz = 4.0;
  double slow_cutoff_hz = 0.6;
  std::size_t robust_window_size = 7;
  double minimum_force_n = 0.5;
  double minimum_forward_cosine = 0.20;
  double change_confirm_s = 0.15;
  double settling_s = 0.25;
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
  Vec3 lateral_force_vector;
  double tension_n = 0.0;
  double axial_force_n = 0.0;
  double lateral_force_n = 0.0;
  double angle_to_locked_rad = 0.0;
  double fast_angle_rad = 0.0;
  double slow_angle_rad = 0.0;
  double fast_slow_angle_rad = 0.0;
  double entry_angle_rad = 0.05235987755982989;
  double exit_angle_rad = 0.026179938779914945;
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
  bool valid_direction_sample(const Vec3 & force, Vec3 & unit, double & tension) const;
  void update_track_state(DirectionEstimate & estimate, double dt_s);

  DirectionFilterConfig config_;
  DirectionBasis basis_;
  DirectionNoiseProfile noise_profile_;
  std::deque<Vec3> direction_window_;
  Vec3 locked_direction_;
  Vec3 fast_direction_;
  Vec3 slow_direction_;
  bool filter_initialized_ = false;
  double change_candidate_s_ = 0.0;
  double settling_elapsed_s_ = 0.0;
  DirectionTrackState state_ = DirectionTrackState::SENSOR_HOLD;
};

// J maps a displacement in the tangent plane (m) to the measured lateral
// force components (N). Its sign is physical: a positive displacement is a
// positive motion in the corresponding tangent basis direction.
struct LateralResponseJacobian
{
  bool valid = false;
  double j11_n_per_m = 0.0;
  double j12_n_per_m = 0.0;
  double j21_n_per_m = 0.0;
  double j22_n_per_m = 0.0;
};

struct LateralCorrectionConfig
{
  double position_gain_s_inv = 0.30;
  double damping_n_per_m = 0.50;
  double minimum_lateral_force_n = 0.03;
  double maximum_speed_mps = 0.0005;
  double maximum_total_displacement_m = 0.003;
};

struct LateralCorrectionResult
{
  bool valid = false;
  bool limited = false;
  double error_b1_n = 0.0;
  double error_b2_n = 0.0;
  double requested_speed_mps = 0.0;
  double applied_speed_mps = 0.0;
  double accumulated_displacement_m = 0.0;
  Vec3 velocity_base;
  std::string reason;
};

class DampedLateralController
{
public:
  DampedLateralController(
    const Vec3 & locked_direction,
    const LateralResponseJacobian & jacobian,
    const LateralCorrectionConfig & config = LateralCorrectionConfig{});

  void reset();
  LateralCorrectionResult update(
    const DirectionEstimate & estimate,
    bool active,
    double dt_s);

  const LateralResponseJacobian & jacobian() const {return jacobian_;}
  const LateralCorrectionConfig & config() const {return config_;}

private:
  bool solve_damped(const double e1, const double e2, double & q1, double & q2) const;

  LateralResponseJacobian jacobian_;
  LateralCorrectionConfig config_;
  DirectionBasis basis_;
  double accumulated_b1_m_ = 0.0;
  double accumulated_b2_m_ = 0.0;
};

}  // namespace fr_traction

#endif  // FR_TRACTION__DIRECTION_CORRECTION_HPP_
