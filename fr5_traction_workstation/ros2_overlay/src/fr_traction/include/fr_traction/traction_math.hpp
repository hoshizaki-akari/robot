#ifndef FR_TRACTION__TRACTION_MATH_HPP_
#define FR_TRACTION__TRACTION_MATH_HPP_

#include <cstddef>
#include <string>
#include <vector>

namespace fr_traction
{

struct Vec3
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

Vec3 operator+(const Vec3 & lhs, const Vec3 & rhs);
Vec3 operator-(const Vec3 & lhs, const Vec3 & rhs);
Vec3 operator*(const Vec3 & value, double scalar);
Vec3 operator*(double scalar, const Vec3 & value);
double dot(const Vec3 & lhs, const Vec3 & rhs);
double norm(const Vec3 & value);
bool finite(const Vec3 & value);
bool normalize(const Vec3 & value, Vec3 & unit, double minimum_norm = 1e-12);

struct ForceMetrics
{
  double actual_force_n = 0.0;
  Vec3 lateral_force_vector;
  double lateral_force_n = 0.0;
};

bool project_force(const Vec3 & force, const Vec3 & direction, ForceMetrics & result);

class FirstOrderLowPass
{
public:
  explicit FirstOrderLowPass(double cutoff_hz = 5.0);
  void set_cutoff(double cutoff_hz);
  void reset();
  Vec3 update(const Vec3 & sample, double dt_s);
  bool initialized() const {return initialized_;}

private:
  double cutoff_hz_;
  bool initialized_ = false;
  Vec3 value_;
};

class OneDimensionalAdmittance
{
public:
  OneDimensionalAdmittance(
    double virtual_mass,
    double virtual_damping,
    double deadband_n,
    double max_speed_mps,
    double max_acceleration_mps2,
    double integral_gain_s_inv = 0.25,
    double integral_limit_n = 3.0);

  void reset();
  double update(double target_force_n, double actual_force_n, double dt_s);
  double velocity() const {return velocity_mps_;}

private:
  double mass_;
  double damping_;
  double deadband_n_;
  double max_speed_mps_;
  double max_acceleration_mps2_;
  double integral_gain_s_inv_;
  double integral_limit_n_;
  double integral_state_n_s_ = 0.0;
  double velocity_mps_ = 0.0;
};

struct CalibrationResult
{
  bool success = false;
  Vec3 direction;
  double retained_fraction = 0.0;
  double angle_p95_deg = 0.0;
  std::string reason;
};

CalibrationResult robust_calibrate_direction(
  const std::vector<Vec3> & samples,
  std::size_t minimum_samples,
  double maximum_angle_p95_deg);

}  // namespace fr_traction

#endif  // FR_TRACTION__TRACTION_MATH_HPP_
