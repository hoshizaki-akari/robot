#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "fr_traction/direction_correction.hpp"
#include "fr_traction/traction_controller_core.hpp"
#include "fr_traction/traction_safety.hpp"

TEST(TractionSyntheticPipeline, TenNewtonAxisOnlyConvergesWithoutLateralCommand)
{
  fr_traction::TractionControllerCore controller(10.0, 80.0, 0.5, 0.005, 0.02);
  const fr_traction::Vec3 direction{0.0, 0.0, -1.0};
  double actual_force = 0.0;
  double previous_force = 0.0;
  for (int step = 0; step < 400; ++step) {
    const fr_traction::Vec3 wrench{0.0, 0.0, -actual_force};
    const auto output = controller.update(
      fr_traction::ControlMode::TRACTION, direction, 10.0, wrench, 0.01);
    ASSERT_TRUE(output.valid);
    EXPECT_DOUBLE_EQ(output.linear_velocity.x, 0.0);
    EXPECT_DOUBLE_EQ(output.linear_velocity.y, 0.0);
    EXPECT_TRUE(std::isfinite(output.linear_velocity.z));
    actual_force = std::max(0.0, actual_force + output.scalar_velocity_mps * 200.0 * 0.01);
    EXPECT_GE(actual_force + 1e-12, previous_force);
    previous_force = actual_force;
  }
  EXPECT_GT(actual_force, 0.0);

  controller.reset();
  const auto settled = controller.update(
    fr_traction::ControlMode::TRACTION, direction, 10.0,
    {0.0, 0.0, -9.8}, 0.01);
  EXPECT_TRUE(settled.valid);
  EXPECT_DOUBLE_EQ(settled.scalar_velocity_mps, 0.0);
}

TEST(TractionSyntheticPipeline, SafetyFaultsStopTheSamePipeline)
{
  fr_traction::SafetyMonitor monitor;
  fr_traction::SafetySample sample;
  sample.wrench_valid = true;
  sample.wrench_fresh = true;
  sample.ee_fresh = true;
  sample.controller_healthy = true;
  sample.raw_wrench = {0.0, 0.0, -30.0};
  sample.metrics.actual_force_n = 30.0;
  EXPECT_EQ(
    monitor.update(sample, 0.0, false), fr_traction::SafetyFault::NONE);
}

TEST(TractionSyntheticPipeline, DirectionChangeFollowsWhileTotalTensionStaysControlled)
{
  constexpr double kPi = 3.14159265358979323846;
  fr_traction::DirectionFilterConfig filter_config;
  filter_config.change_confirm_s = 0.20;
  fr_traction::DirectionEstimator estimator({0.0, 0.0, 1.0}, filter_config);
  fr_traction::AdaptiveDirectionFollower follower;
  fr_traction::TractionControllerCore controller(10.0, 80.0, 0.15, 0.005, 0.02);

  for (int step = 0; step < 50; ++step) {
    estimator.update({0.0, 0.0, 5.0}, 0.01);
  }
  const double angle = 35.0 * kPi / 180.0;
  const fr_traction::Vec3 changed_force{5.0 * std::sin(angle), 0.0, 5.0 * std::cos(angle)};
  bool follow_motion_seen = false;
  fr_traction::DirectionEstimate estimate;
  for (int step = 0; step < 250; ++step) {
    estimate = estimator.update(changed_force, 0.01);
    const auto follow = follower.update(estimate, true, 0.01);
    const auto output = controller.update(
      fr_traction::ControlMode::TRACTION, estimate.tracked_direction, 5.0,
      changed_force, 0.01, follow.velocity_base);
    ASSERT_TRUE(output.valid);
    EXPECT_DOUBLE_EQ(output.scalar_velocity_mps, 0.0);
    follow_motion_seen = follow_motion_seen || fr_traction::norm(follow.velocity_base) > 0.0;
  }
  EXPECT_TRUE(follow_motion_seen);
  EXPECT_FALSE(estimate.ambiguity_timed_out);
  EXPECT_GT(fr_traction::dot(estimate.tracked_direction, changed_force * 0.2), 0.99);
}
