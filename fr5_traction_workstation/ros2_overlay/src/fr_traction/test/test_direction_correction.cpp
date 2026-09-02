#include "fr_traction/direction_correction.hpp"

#include <cmath>
#include <vector>

#include "gtest/gtest.h"

namespace fr_traction
{

namespace
{

Vec3 direction_from_degrees(double degrees)
{
  const double angle = degrees * 3.14159265358979323846 / 180.0;
  return {std::sin(angle), 0.0, std::cos(angle)};
}

}  // namespace

TEST(DirectionCorrection, TangentBasisIsOrthonormalAndDeterministic)
{
  DirectionBasis basis;
  ASSERT_TRUE(make_tangent_basis({0.2, -0.3, 0.9327379}, basis));
  EXPECT_NEAR(norm(basis.normal), 1.0, 1e-12);
  EXPECT_NEAR(norm(basis.b1), 1.0, 1e-12);
  EXPECT_NEAR(norm(basis.b2), 1.0, 1e-12);
  EXPECT_NEAR(dot(basis.normal, basis.b1), 0.0, 1e-12);
  EXPECT_NEAR(dot(basis.normal, basis.b2), 0.0, 1e-12);
  EXPECT_NEAR(dot(basis.b1, basis.b2), 0.0, 1e-12);

  DirectionBasis repeated;
  ASSERT_TRUE(make_tangent_basis(basis.normal, repeated));
  EXPECT_NEAR(dot(basis.b1, repeated.b1), 1.0, 1e-12);
  EXPECT_NEAR(dot(basis.b2, repeated.b2), 1.0, 1e-12);
}

TEST(DirectionCorrection, NoiseProfileUsesAnAdaptiveFloor)
{
  std::vector<Vec3> samples;
  for (int index = 0; index < 100; ++index) {
    const double angle = (index % 5 - 2) * 0.001;
    samples.push_back({std::sin(angle) * 5.0, 0.0, std::cos(angle) * 5.0});
  }
  const auto profile = estimate_direction_noise(samples, {0.0, 0.0, 1.0});
  ASSERT_TRUE(profile.valid);
  EXPECT_GE(profile.entry_angle_rad, 3.0 * 3.14159265358979323846 / 180.0);
  EXPECT_LT(profile.exit_angle_rad, profile.entry_angle_rad);
}

TEST(DirectionCorrection, OneSampleSpikeIsRejectedButPersistentChangeIsDetected)
{
  DirectionFilterConfig config;
  config.fast_cutoff_hz = 8.0;
  config.slow_cutoff_hz = 0.5;
  config.robust_window_size = 7;
  config.change_confirm_s = 0.10;
  DirectionEstimator estimator({0.0, 0.0, 1.0}, config);
  DirectionNoiseProfile profile;
  profile.valid = true;
  profile.entry_angle_rad = 5.0 * 3.14159265358979323846 / 180.0;
  profile.exit_angle_rad = 2.0 * 3.14159265358979323846 / 180.0;
  estimator.set_noise_profile(profile);

  for (int index = 0; index < 20; ++index) {
    const auto estimate = estimator.update({0.0, 0.0, 5.0}, 0.01);
    ASSERT_TRUE(estimate.valid);
    EXPECT_EQ(estimate.state, DirectionTrackState::STABLE);
  }
  const auto spike = estimator.update(direction_from_degrees(30.0) * 5.0, 0.01);
  EXPECT_TRUE(spike.valid);
  EXPECT_TRUE(spike.raw_outlier);
  EXPECT_NE(spike.state, DirectionTrackState::CORRECTING);

  bool correcting_seen = false;
  for (int index = 0; index < 45; ++index) {
    const auto estimate = estimator.update(direction_from_degrees(15.0) * 5.0, 0.01);
    ASSERT_TRUE(estimate.valid);
    correcting_seen = correcting_seen || estimate.state == DirectionTrackState::CORRECTING;
  }
  EXPECT_TRUE(correcting_seen);
}

TEST(DirectionCorrection, LowForceAndReverseForceHoldTheCorrection)
{
  DirectionEstimator estimator({0.0, 0.0, 1.0});
  auto low = estimator.update({0.0, 0.0, 0.1}, 0.01);
  EXPECT_FALSE(low.valid);
  EXPECT_EQ(low.state, DirectionTrackState::SENSOR_HOLD);
  auto reverse = estimator.update({0.0, 0.0, -5.0}, 0.01);
  EXPECT_FALSE(reverse.valid);
  EXPECT_EQ(reverse.state, DirectionTrackState::SENSOR_HOLD);
}

TEST(DirectionCorrection, DampedControllerUsesNegativeFeedbackAndLimitsMotion)
{
  // Use a positive diagonal response here so the sign assertion isolates the
  // controller feedback sign. The measured FR5 matrix is exercised by the
  // shadow-mode integration configuration, not by this basic algebra test.
  const LateralResponseJacobian jacobian{true, 5.0, 0.0, 0.0, 5.0};
  LateralCorrectionConfig config;
  config.position_gain_s_inv = 0.30;
  config.damping_n_per_m = 0.50;
  config.minimum_lateral_force_n = 0.0;
  config.maximum_speed_mps = 0.0005;
  config.maximum_total_displacement_m = 0.003;
  DampedLateralController controller({0.0, 0.0, 1.0}, jacobian, config);
  DirectionEstimate estimate;
  estimate.valid = true;
  estimate.locked_direction = {0.0, 0.0, 1.0};
  estimate.lateral_force_vector = {1.0, 0.0, 0.0};
  estimate.state = DirectionTrackState::CORRECTING;
  auto result = controller.update(estimate, true, 0.01);
  ASSERT_TRUE(result.valid);
  EXPECT_LT(result.velocity_base.x, 0.0);
  EXPECT_LE(result.applied_speed_mps, config.maximum_speed_mps + 1e-12);

  for (int index = 0; index < 2000; ++index) {
    result = controller.update(estimate, true, 0.01);
  }
  EXPECT_LE(
    result.accumulated_displacement_m,
    config.maximum_total_displacement_m + 1e-12);
}

TEST(DirectionCorrection, ShadowModeCalculatesButDoesNotMove)
{
  const LateralResponseJacobian jacobian{true, 5.0, 0.0, 0.0, 5.0};
  DampedLateralController controller({0.0, 0.0, 1.0}, jacobian);
  DirectionEstimate estimate;
  estimate.valid = true;
  estimate.lateral_force_vector = {0.2, 0.0, 0.0};
  estimate.state = DirectionTrackState::CORRECTING;
  const auto result = controller.update(estimate, false, 0.01);
  ASSERT_TRUE(result.valid);
  EXPECT_GT(result.requested_speed_mps, 0.0);
  EXPECT_DOUBLE_EQ(result.applied_speed_mps, 0.0);
  EXPECT_DOUBLE_EQ(norm(result.velocity_base), 0.0);
  EXPECT_EQ(result.reason, "SHADOW_ONLY");
}

}  // namespace fr_traction
