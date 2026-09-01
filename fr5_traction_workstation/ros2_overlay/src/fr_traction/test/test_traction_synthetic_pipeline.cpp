#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

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
  sample.metrics.actual_force_n = 10.0;
  EXPECT_EQ(
    monitor.update(sample, 0.0, false), fr_traction::SafetyFault::HARD_OVERFORCE);
}
