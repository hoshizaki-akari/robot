#include "fr_traction/direction_correction.hpp"

#include <algorithm>
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

  for (int index = 0; index < 40; ++index) {
    const auto estimate = estimator.update({0.0, 0.0, 5.0}, 0.01);
    ASSERT_TRUE(estimate.valid);
    if (index >= 30) {EXPECT_EQ(estimate.state, DirectionTrackState::STABLE);}
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

TEST(DirectionCorrection, LowForceHoldsButLargeDirectionChangesAreAccepted)
{
  DirectionEstimator estimator({0.0, 0.0, 1.0});
  auto low = estimator.update({0.0, 0.0, 0.1}, 0.01);
  EXPECT_FALSE(low.valid);
  EXPECT_EQ(low.state, DirectionTrackState::SENSOR_HOLD);

  DirectionEstimate estimate;
  for (int index = 0; index < 150; ++index) {
    estimate = estimator.update(direction_from_degrees(120.0) * 5.0, 0.01);
  }
  EXPECT_TRUE(estimate.valid);
  EXPECT_NE(estimate.state, DirectionTrackState::AMBIGUOUS);
  EXPECT_GT(dot(estimate.tracked_direction, direction_from_degrees(120.0)), 0.98);
}

TEST(DirectionCorrection, DirectionResumesOnlyAfterStableTensionRecovery)
{
  DirectionFilterConfig config;
  config.recovery_confirm_s = 0.30;
  DirectionEstimator estimator({0.0, 0.0, 1.0}, config);
  EXPECT_EQ(
    estimator.update({0.0, 0.0, 0.1}, 0.01).state,
    DirectionTrackState::SENSOR_HOLD);
  DirectionEstimate estimate;
  for (int index = 0; index < 20; ++index) {
    estimate = estimator.update({0.0, 0.0, 5.0}, 0.01);
  }
  EXPECT_EQ(estimate.state, DirectionTrackState::SENSOR_HOLD);
  for (int index = 0; index < 15; ++index) {
    estimate = estimator.update({0.0, 0.0, 5.0}, 0.01);
  }
  EXPECT_EQ(estimate.state, DirectionTrackState::STABLE);
}

TEST(DirectionCorrection, NearbyWanderFormsOneEquivalentDirectionWithoutFlapping)
{
  DirectionFilterConfig config;
  config.change_confirm_s = 0.20;
  config.settling_s = 0.40;
  DirectionEstimator estimator({0.0, 0.0, 1.0}, config);
  DirectionEstimate estimate;
  for (int index = 0; index < 250; ++index) {
    const double degrees = 25.0 + static_cast<double>((index % 5) - 2);
    estimate = estimator.update(direction_from_degrees(degrees) * 5.0, 0.01);
  }
  EXPECT_TRUE(estimate.valid);
  EXPECT_FALSE(estimate.ambiguity_timed_out);
  EXPECT_LT(
    std::acos(
      std::clamp(
        dot(estimate.tracked_direction, direction_from_degrees(25.0)), -1.0, 1.0)),
    4.0 * 3.14159265358979323846 / 180.0);
}

TEST(DirectionCorrection, AlternatingIncompatibleDirectionsEventuallyBecomeAmbiguous)
{
  DirectionFilterConfig config;
  config.ambiguity_timeout_s = 1.0;
  DirectionEstimator estimator({0.0, 0.0, 1.0}, config);
  DirectionEstimate estimate;
  for (int index = 0; index < 180; ++index) {
    estimate = estimator.update(
      direction_from_degrees(index % 2 == 0 ? 40.0 : -40.0) * 5.0, 0.01);
  }
  EXPECT_EQ(estimate.state, DirectionTrackState::AMBIGUOUS);
  EXPECT_TRUE(estimate.ambiguity_timed_out);
}

TEST(DirectionCorrection, AdaptiveFollowerIsFastForLargeAnglesAndSlowsNearTarget)
{
  AdaptiveFollowConfig config;
  config.speed_gain_mps_per_rad = 0.020;
  config.maximum_speed_mps = 0.020;
  config.maximum_acceleration_mps2 = 1.0;
  AdaptiveDirectionFollower follower(config);
  DirectionEstimate estimate;
  estimate.valid = true;
  estimate.candidate_confirmed = true;
  estimate.state = DirectionTrackState::CORRECTING;
  estimate.exit_angle_rad = 2.0 * 3.14159265358979323846 / 180.0;
  estimate.tracked_direction = direction_from_degrees(0.0);
  estimate.candidate_direction = direction_from_degrees(60.0);
  const auto large = follower.update(estimate, true, 0.01);
  ASSERT_TRUE(large.valid);
  EXPECT_TRUE(large.active);
  EXPECT_GT(large.applied_speed_mps, 0.0);

  follower.reset();
  estimate.candidate_direction = direction_from_degrees(8.0);
  const auto small = follower.update(estimate, true, 0.01);
  ASSERT_TRUE(small.valid);
  EXPECT_LT(small.requested_speed_mps, large.requested_speed_mps);
  EXPECT_GT(dot(small.velocity_base, Vec3{1.0, 0.0, 0.0}), 0.0);
}

TEST(DirectionCorrection, ShadowModeCalculatesButDoesNotMove)
{
  AdaptiveDirectionFollower controller;
  DirectionEstimate estimate;
  estimate.valid = true;
  estimate.candidate_confirmed = true;
  estimate.exit_angle_rad = 2.0 * 3.14159265358979323846 / 180.0;
  estimate.tracked_direction = direction_from_degrees(0.0);
  estimate.candidate_direction = direction_from_degrees(20.0);
  estimate.state = DirectionTrackState::CORRECTING;
  const auto result = controller.update(estimate, false, 0.01);
  ASSERT_TRUE(result.valid);
  EXPECT_GT(result.requested_speed_mps, 0.0);
  EXPECT_DOUBLE_EQ(result.applied_speed_mps, 0.0);
  EXPECT_DOUBLE_EQ(norm(result.velocity_base), 0.0);
  EXPECT_EQ(result.reason, "SHADOW_ONLY");
}

}  // namespace fr_traction
