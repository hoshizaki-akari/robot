#include "fr_traction/traction_math.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "gtest/gtest.h"

namespace fr_traction
{

TEST(TractionMath, ProjectsAxialAndLateralForce)
{
  ForceMetrics metrics;
  ASSERT_TRUE(project_force({10.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, metrics));
  EXPECT_NEAR(metrics.actual_force_n, 10.0, 1e-12);
  EXPECT_NEAR(metrics.lateral_force_n, 0.0, 1e-12);

  ASSERT_TRUE(project_force({10.0, 3.0, 4.0}, {1.0, 0.0, 0.0}, metrics));
  EXPECT_NEAR(metrics.actual_force_n, 10.0, 1e-12);
  EXPECT_NEAR(metrics.lateral_force_n, 5.0, 1e-12);

  ASSERT_TRUE(project_force({0.0, 3.0, 4.0}, {1.0, 0.0, 0.0}, metrics));
  EXPECT_NEAR(metrics.actual_force_n, 0.0, 1e-12);
  EXPECT_NEAR(metrics.lateral_force_n, 5.0, 1e-12);
}

TEST(TractionMath, RejectsInvalidDirectionAndWrench)
{
  ForceMetrics metrics;
  EXPECT_FALSE(project_force({1.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, metrics));
  EXPECT_FALSE(project_force({1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, metrics));
  EXPECT_FALSE(project_force({NAN, 0.0, 0.0}, {1.0, 0.0, 0.0}, metrics));
  Vec3 unit;
  EXPECT_FALSE(normalize({INFINITY, 0.0, 0.0}, unit));
}

TEST(TractionMath, LowPassUsesActualDt)
{
  FirstOrderLowPass filter(5.0);
  filter.update({0.0, 0.0, 0.0}, 0.001);
  Vec3 output{};
  for (int i = 0; i < 100; ++i) {
    output = filter.update({1.0, 0.0, 0.0}, 0.001);
  }
  const double expected = 1.0 - std::exp(-2.0 * 3.14159265358979323846 * 5.0 * 0.1);
  EXPECT_NEAR(output.x, expected, 0.05);
}

TEST(TractionMath, RobustCalibrationRejectsOutliersAndWobble)
{
  std::vector<Vec3> samples;
  for (int i = 0; i < 90; ++i) {
    samples.push_back({3.0, 0.0, 0.0});
  }
  for (int i = 0; i < 10; ++i) {
    samples.push_back({-30.0, 20.0, -15.0});
  }
  const auto result = robust_calibrate_direction(samples, 80, 15.0);
  ASSERT_TRUE(result.success);
  EXPECT_NEAR(result.direction.x, 1.0, 1e-12);
  EXPECT_NEAR(result.direction.y, 0.0, 1e-12);
  EXPECT_LT(result.angle_p95_deg, 2.0);
  EXPECT_NEAR(result.retained_fraction, 0.90, 1e-12);

  samples.clear();
  for (int i = 0; i < 100; ++i) {
    samples.push_back(i % 2 == 0 ? Vec3{3.0, 0.0, 0.0} : Vec3{2.0, 2.0, 0.0});
  }
  EXPECT_FALSE(robust_calibrate_direction(samples, 80, 15.0).success);
}

TEST(TractionMath, DirectionCalibrationIgnoresChangingTensionMagnitude)
{
  std::vector<Vec3> samples;
  for (int i = 0; i < 100; ++i) {
    const double magnitude = 9.0 - 3.0 * static_cast<double>(i) / 99.0;
    const double angle = static_cast<double>((i % 7) - 3) * 0.4 *
      3.14159265358979323846 / 180.0;
    samples.push_back({magnitude * std::cos(angle), magnitude * std::sin(angle), 0.0});
  }
  const auto result = robust_calibrate_direction(samples, 80, 15.0);
  ASSERT_TRUE(result.success);
  EXPECT_GT(result.retained_fraction, 0.95);
  EXPECT_NEAR(result.direction.x, 1.0, 1e-3);
  EXPECT_NEAR(result.direction.y, 0.0, 1e-3);
  EXPECT_LT(result.angle_p95_deg, 2.0);
}

TEST(TractionMath, PositionControllerApproachesAndSettlesWithoutRepeatedOvershoot)
{
  PositionTractionController controller;
  constexpr double dt = 0.01;
  constexpr double target_force = 10.0;
  constexpr double stiffness = 330.0;
  double position_m = 0.0;
  double actuator_velocity_mps = 0.0;
  double force_n = 3.0;
  double maximum_force_n = force_n;
  double settling_elapsed_s = 0.0;
  int velocity_reversals = 0;
  int previous_sign = 0;

  for (int step = 0; step < 1200; ++step) {
    const auto result = controller.update(target_force, force_n, dt);
    ASSERT_TRUE(result.valid);
    actuator_velocity_mps += (result.velocity_mps - actuator_velocity_mps) * dt / 0.06;
    position_m += actuator_velocity_mps * dt;
    force_n = 3.0 + stiffness * position_m;
    maximum_force_n = std::max(maximum_force_n, force_n);
    const int sign = result.velocity_mps > 0.0005 ? 1 :
      (result.velocity_mps < -0.0005 ? -1 : 0);
    if (sign != 0 && previous_sign != 0 && sign != previous_sign) {++velocity_reversals;}
    if (sign != 0) {previous_sign = sign;}
    if (result.phase == PositionControlPhase::SETTLING &&
      std::abs(force_n - target_force) <= 0.20)
    {
      settling_elapsed_s += dt;
    } else {
      settling_elapsed_s = 0.0;
    }
  }

  EXPECT_LE(maximum_force_n, 10.5);
  EXPECT_LE(velocity_reversals, 1);
  EXPECT_NEAR(force_n, target_force, 0.20);
  EXPECT_GE(settling_elapsed_s, 0.50);
}

TEST(TractionMath, PositionControllerReacquiresMovingTargetThenHoldsStable)
{
  PositionTractionController controller;
  constexpr double dt = 0.01;
  constexpr double target_force = 10.0;
  constexpr double stiffness = 300.0;
  double robot_position_m = 0.0;
  double target_motion_m = 0.0;
  double actuator_velocity_mps = 0.0;
  double force_n = 3.0;
  double settling_elapsed_s = 0.0;

  for (int step = 0; step < 1500; ++step) {
    const double time_s = step * dt;
    if (time_s > 2.0 && time_s < 4.0) {target_motion_m += 0.0015 * dt;}
    if (time_s > 5.0 && time_s < 6.0) {target_motion_m -= 0.0010 * dt;}
    force_n = 3.0 + stiffness * (robot_position_m - target_motion_m);
    const auto result = controller.update(target_force, std::max(0.0, force_n), dt);
    ASSERT_TRUE(result.valid);
    actuator_velocity_mps += (result.velocity_mps - actuator_velocity_mps) * dt / 0.06;
    robot_position_m += actuator_velocity_mps * dt;
    if (time_s >= 6.0 && result.phase == PositionControlPhase::SETTLING &&
      std::abs(force_n - target_force) <= 0.20)
    {
      settling_elapsed_s += dt;
    } else if (time_s >= 6.0) {
      settling_elapsed_s = 0.0;
    }
  }

  EXPECT_NEAR(force_n, target_force, 0.20);
  EXPECT_GE(settling_elapsed_s, 0.50);
}

TEST(TractionMath, PositionControllerDoesNotSettleWithResidualVelocity)
{
  PositionTractionController controller;
  constexpr double dt = 0.01;
  PositionControlResult result;
  for (int step = 0; step < 50; ++step) {
    result = controller.update(10.0, 5.0, dt);
  }
  ASSERT_GT(result.velocity_mps, 0.0005);

  result = controller.update(10.0, 10.0, dt);
  EXPECT_NE(result.phase, PositionControlPhase::SETTLING);
  int settling_wait_steps = 0;
  while (result.valid && result.phase != PositionControlPhase::SETTLING &&
    settling_wait_steps < 200)
  {
    result = controller.update(10.0, 10.0, dt);
    ++settling_wait_steps;
  }
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.phase, PositionControlPhase::SETTLING);
  EXPECT_LT(settling_wait_steps, 200);
}

TEST(TractionMath, PositionControllerSettlesFifteenNewtonsWithinFiveSeconds)
{
  PositionTractionController controller;
  constexpr double dt = 0.01;
  constexpr double target_force = 15.0;
  double position_m = 0.0;
  double actuator_velocity_mps = 0.0;
  double stable_elapsed_s = 0.0;
  double reached_at_s = 100.0;
  double maximum_force_n = 0.0;

  for (int step = 0; step < 600; ++step) {
    const double time_s = step * dt;
    const double plant_force_n = 2.2 + 250.0 * position_m +
      5000.0 * position_m * position_m;
    const double measured_force_n = plant_force_n + 0.03 * std::sin(7.0 * time_s);
    const auto result = controller.update(target_force, measured_force_n, dt);
    ASSERT_TRUE(result.valid);
    actuator_velocity_mps += (result.velocity_mps - actuator_velocity_mps) * dt / 0.06;
    position_m += actuator_velocity_mps * dt;
    maximum_force_n = std::max(maximum_force_n, measured_force_n);
    if (result.phase == PositionControlPhase::SETTLING &&
      std::abs(measured_force_n - target_force) <= 0.20)
    {
      stable_elapsed_s += dt;
      if (stable_elapsed_s >= 0.50 && reached_at_s > 99.0) {reached_at_s = time_s;}
    } else {
      stable_elapsed_s = 0.0;
    }
  }

  EXPECT_LE(reached_at_s, 5.0);
  EXPECT_LE(maximum_force_n, 15.5);
}

}  // namespace fr_traction
