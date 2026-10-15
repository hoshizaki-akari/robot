#include "fr_traction/traction_controller_core.hpp"

#include <cmath>

#include "gtest/gtest.h"

namespace fr_traction
{

TEST(TractionControllerCore, ProducesOnlyLockedAxisVelocity)
{
  TractionControllerCore core(10.0, 80.0, 0.5, 0.005, 0.02);
  const Vec3 direction{1.0, 0.0, 0.0};
  auto output = core.update(ControlMode::TRACTION, direction, 10.0, {0.0, 0.0, 0.0}, 0.01);
  EXPECT_TRUE(output.valid);
  output = core.update(ControlMode::TRACTION, direction, 10.0, {0.0, 0.0, 0.0}, 0.01);
  EXPECT_TRUE(output.valid);
  EXPECT_GT(output.scalar_velocity_mps, 0.0);
  EXPECT_NEAR(output.linear_velocity.y, 0.0, 1e-12);
  EXPECT_NEAR(output.linear_velocity.z, 0.0, 1e-12);

  const double cross_norm = std::sqrt(
    std::pow(output.linear_velocity.y * direction.z - output.linear_velocity.z * direction.y, 2) +
    std::pow(output.linear_velocity.z * direction.x - output.linear_velocity.x * direction.z, 2) +
    std::pow(output.linear_velocity.x * direction.y - output.linear_velocity.y * direction.x, 2));
  EXPECT_LT(cross_norm, 1e-9);
  EXPECT_DOUBLE_EQ(output.linear_velocity.y, 0.0);
  EXPECT_DOUBLE_EQ(output.linear_velocity.z, 0.0);

  core.reset();
  output = core.update(ControlMode::TRACTION, direction, 10.0, {12.0, 0.0, 0.0}, 0.01);
  EXPECT_TRUE(output.valid);
  output = core.update(ControlMode::TRACTION, direction, 10.0, {12.0, 0.0, 0.0}, 0.01);
  EXPECT_LT(output.scalar_velocity_mps, 0.0);
}

TEST(TractionControllerCore, DeadbandAndDisableStopTheController)
{
  TractionControllerCore core(10.0, 80.0, 0.5, 0.005, 0.02);
  auto output = core.update(ControlMode::TRACTION, {1.0, 0.0, 0.0}, 10.0, {9.8, 0.0, 0.0}, 0.01);
  EXPECT_TRUE(output.valid);
  EXPECT_NEAR(output.scalar_velocity_mps, 0.0, 1e-12);
  output = core.update(ControlMode::DISABLED, {1.0, 0.0, 0.0}, 0.0, {9.8, 0.0, 0.0}, 0.01);
  EXPECT_TRUE(output.valid);
  EXPECT_DOUBLE_EQ(output.scalar_velocity_mps, 0.0);
  EXPECT_DOUBLE_EQ(output.linear_velocity.x, 0.0);
}

TEST(TractionControllerCore, NarrowDeadbandContinuesTowardFiveNewtonTarget)
{
  TractionControllerCore core(10.0, 80.0, 0.15, 0.005, 0.02);
  const Vec3 direction{1.0, 0.0, 0.0};

  auto output = core.update(ControlMode::TRACTION, direction, 5.0, {4.86, 0.0, 0.0}, 0.01);
  EXPECT_DOUBLE_EQ(output.scalar_velocity_mps, 0.0);

  output = core.update(ControlMode::TRACTION, direction, 5.0, {4.80, 0.0, 0.0}, 0.01);
  EXPECT_GT(output.scalar_velocity_mps, 0.0);
}

TEST(TractionControllerCore, TargetBandBrakesResidualVelocitySmoothly)
{
  TractionControllerCore core(10.0, 80.0, 0.5, 0.005, 0.02);
  const Vec3 direction{1.0, 0.0, 0.0};
  ControllerOutput output;
  for (int step = 0; step < 100; ++step) {
    output = core.update(ControlMode::TRACTION, direction, 15.0, {10.0, 0.0, 0.0}, 0.01);
  }
  ASSERT_GT(output.scalar_velocity_mps, 0.0);

  const double velocity_before_band = output.scalar_velocity_mps;
  output = core.update(ControlMode::TRACTION, direction, 15.0, {14.7, 0.0, 0.0}, 0.01);
  EXPECT_GE(output.scalar_velocity_mps, 0.0);
  EXPECT_LE(output.scalar_velocity_mps, velocity_before_band);

  output = core.update(ControlMode::TRACTION, direction, 15.0, {16.0, 0.0, 0.0}, 0.01);
  EXPECT_LE(output.scalar_velocity_mps, velocity_before_band);

  core.reset();
  output = core.update(ControlMode::TRACTION, direction, 5.0, {30.0, 0.0, 0.0}, 0.01);
  EXPECT_TRUE(output.valid);
  output = core.update(ControlMode::TRACTION, direction, 5.0, {30.0, 0.0, 0.0}, 0.01);
  EXPECT_LT(output.scalar_velocity_mps, 0.0);
  EXPECT_LT(output.linear_velocity.x, 0.0);
}

TEST(TractionControllerCore, RejectsInvalidDirection)
{
  TractionControllerCore core(10.0, 80.0, 0.5, 0.005, 0.02);
  EXPECT_FALSE(
    core.update(
      ControlMode::TRACTION, {0.0, 0.0, 0.0}, 10.0, {-1.0, 0.0, 0.0}, 0.01).valid);
}

TEST(TractionControllerCore, TractionControlsTotalRopeTensionAfterDirectionChange)
{
  TractionControllerCore core(10.0, 80.0, 0.15, 0.005, 0.02);
  // The current control direction has not yet caught up with the measured
  // rope direction. The rope already carries 5 N in total, so no extra axial
  // motion should be requested.
  const auto output = core.update(
    ControlMode::TRACTION, {0.0, 0.0, 1.0}, 5.0, {3.0, 4.0, 0.0}, 0.01);
  ASSERT_TRUE(output.valid);
  EXPECT_DOUBLE_EQ(output.scalar_velocity_mps, 0.0);
}

TEST(TractionControllerCore, PositioningUsesDedicatedPredictiveControllerOnLockedAxis)
{
  PositionControlConfig position_config;
  position_config.tolerance_n = 0.20;
  TractionControllerCore core(
    10.0, 80.0, 0.15, 0.005, 0.02, 0.25, 3.0,
    1.0, 0.6, 0.15, 0.00625, 0.050, 0.30, 3.0,
    -1.0, -1.0, 1.0, position_config);
  const Vec3 locked_direction{0.0, 0.0, 1.0};

  auto output = core.update(
    ControlMode::POSITIONING, locked_direction, 5.0, {3.0, 4.0, 0.0}, 0.01);
  ASSERT_TRUE(output.valid);
  EXPECT_TRUE(output.position_control.valid);
  EXPECT_DOUBLE_EQ(output.linear_velocity.x, 0.0);
  EXPECT_DOUBLE_EQ(output.linear_velocity.y, 0.0);

  output = core.update(
    ControlMode::POSITIONING, locked_direction, 5.0, {3.0, 4.0, 0.0}, 0.01);
  ASSERT_TRUE(output.valid);
  EXPECT_EQ(output.position_control.phase, PositionControlPhase::SETTLING);
  EXPECT_DOUBLE_EQ(output.scalar_velocity_mps, 0.0);
  EXPECT_DOUBLE_EQ(output.linear_velocity.z, 0.0);
}

TEST(TractionControllerCore, DirectionCorrectionIsLateralOnly)
{
  TractionControllerCore core(10.0, 80.0, 0.15, 0.020, 0.02);
  const Vec3 lateral{0.0, 0.004, -0.003};
  auto output = core.update(
    ControlMode::RELEASING, {1.0, 0.0, 0.0}, 10.0, {2.0, 0.0, 0.0}, 0.01, lateral);
  ASSERT_TRUE(output.valid);
  EXPECT_DOUBLE_EQ(output.scalar_velocity_mps, 0.0);
  EXPECT_DOUBLE_EQ(output.linear_velocity.x, 0.0);
  EXPECT_GT(output.linear_velocity.y, 0.0);
  EXPECT_LT(output.linear_velocity.z, 0.0);
  EXPECT_LE(norm(output.linear_velocity), norm(lateral));
  const double first_speed = norm(output.linear_velocity);
  for (int step = 0; step < 20; ++step) {
    output = core.update(
      ControlMode::RELEASING, {1.0, 0.0, 0.0}, 10.0, {2.0, 0.0, 0.0}, 0.01, lateral);
  }
  EXPECT_GT(norm(output.linear_velocity), first_speed);
}

TEST(TractionControllerCore, AssistedDragUsesThreeAxisForceAndReleaseHysteresis)
{
  TractionControllerCore core(
    10.0, 80.0, 0.15, 0.020, 0.02, 0.25, 3.0,
    0.5, 0.2, 0.08, 0.015, 0.050, 0.30, 3.0,
    -1.0, -1.0, 1.0, PositionControlConfig{}, 0.60, 12.0);
  auto output = core.update(ControlMode::DRAGGING, {}, 0.0, {0.35, 0.0, 0.0}, 0.01);
  EXPECT_TRUE(output.valid);
  EXPECT_DOUBLE_EQ(norm(output.linear_velocity), 0.0);

  // A light 0.7 N hand force must already create a useful continuous move;
  // the former 1 N threshold made the operator repeatedly break away from rest.
  for (int step = 0; step < 20; ++step) {
    output = core.update(ControlMode::DRAGGING, {}, 0.0, {0.7, 0.0, 0.0}, 0.01);
  }
  EXPECT_LT(output.linear_velocity.x, -0.004);

  for (int step = 0; step < 50; ++step) {
    output = core.update(ControlMode::DRAGGING, {}, 0.0, {2.0, -1.0, 0.5}, 0.01);
  }
  EXPECT_LT(output.linear_velocity.x, 0.0);
  EXPECT_GT(output.linear_velocity.y, 0.0);
  EXPECT_GT(output.linear_velocity.z, 0.0);
  EXPECT_GT(dot(output.linear_velocity, Vec3{-2.0, 1.0, 0.5}), 0.0);
  EXPECT_LE(norm(output.linear_velocity), 0.050);

  for (int step = 0; step < 100; ++step) {
    output = core.update(ControlMode::DRAGGING, {}, 0.0, {0.0, 0.0, 0.0}, 0.01);
  }
  EXPECT_NEAR(norm(output.linear_velocity), 0.0, 1e-9);
}

TEST(TractionControllerCore, ForceReversalRespectsAccelerationAndJerkLimits)
{
  constexpr double dt = 0.01;
  constexpr double maximum_acceleration = 0.30;
  constexpr double maximum_jerk = 3.0;
  TractionControllerCore core(
    10.0, 80.0, 0.15, 0.020, 0.02, 0.25, 3.0,
    1.0, 0.6, 0.15, 0.00625, 0.050, maximum_acceleration, maximum_jerk);
  double previous_velocity = 0.0;
  double previous_acceleration = 0.0;
  for (int step = 0; step < 500; ++step) {
    const Vec3 wrench = step < 250 ? Vec3{0.0, 0.0, 0.0} : Vec3{30.0, 0.0, 0.0};
    const auto output = core.update(
      ControlMode::TRACTION, {1.0, 0.0, 0.0}, 10.0, wrench, dt);
    ASSERT_TRUE(output.valid);
    const double acceleration = (output.scalar_velocity_mps - previous_velocity) / dt;
    EXPECT_LE(std::abs(acceleration), maximum_acceleration + 1e-9);
    EXPECT_LE(
      std::abs(acceleration - previous_acceleration), maximum_jerk * dt + 1e-8);
    previous_velocity = output.scalar_velocity_mps;
    previous_acceleration = acceleration;
  }
}

TEST(TractionControllerCore, ConstantForceDownStepStopsPullingPromptly)
{
  constexpr double dt = 0.01;
  TractionControllerCore core(10.0, 80.0, 0.15, 0.020, 0.02);
  const Vec3 direction{1.0, 0.0, 0.0};

  ControllerOutput output;
  for (int step = 0; step < 100; ++step) {
    output = core.update(
      ControlMode::TRACTION, direction, 65.0, {60.0, 0.0, 0.0}, dt);
    ASSERT_TRUE(output.valid);
  }
  ASSERT_GT(output.scalar_velocity_mps, 0.0);

  int first_non_positive_step = -1;
  for (int step = 0; step < 50; ++step) {
    output = core.update(
      ControlMode::TRACTION, direction, 45.0, {60.0, 0.0, 0.0}, dt);
    ASSERT_TRUE(output.valid);
    if (output.scalar_velocity_mps <= 0.0) {
      first_non_positive_step = step;
      break;
    }
  }

  ASSERT_GE(first_non_positive_step, 0);
  EXPECT_LT(first_non_positive_step * dt, 0.30);
}

TEST(TractionControllerCore, ConstantForceRejectsHighStiffnessOscillation)
{
  constexpr double dt = 0.01;
  TractionControllerCore core(10.0, 80.0, 0.15, 0.020, 0.02);
  const Vec3 direction{1.0, 0.0, 0.0};
  double position_m = 0.0;
  double actuator_velocity_mps = 0.0;
  double command_target_n = 4.0;
  double maximum_force_n = 0.0;
  std::vector<double> final_forces;
  int velocity_reversals = 0;
  int previous_sign = 0;

  for (int step = 0; step < 1200; ++step) {
    const double time_s = step * dt;
    command_target_n = std::min(15.0, command_target_n + 3.0 * dt);
    const double force_n = 4.0 + 250.0 * position_m +
      5000.0 * position_m * position_m + 0.03 * std::sin(7.0 * time_s);
    const auto output = core.update(
      ControlMode::TRACTION, direction, command_target_n, {force_n, 0.0, 0.0}, dt);
    ASSERT_TRUE(output.valid);
    actuator_velocity_mps +=
      (output.scalar_velocity_mps - actuator_velocity_mps) * dt / 0.06;
    position_m += actuator_velocity_mps * dt;
    maximum_force_n = std::max(maximum_force_n, force_n);
    const int sign = output.scalar_velocity_mps > 0.0005 ? 1 :
      (output.scalar_velocity_mps < -0.0005 ? -1 : 0);
    if (sign != 0 && previous_sign != 0 && sign != previous_sign) {++velocity_reversals;}
    if (sign != 0) {previous_sign = sign;}
    if (time_s >= 9.0) {final_forces.push_back(force_n);}
  }

  const auto final_range = std::minmax_element(final_forces.begin(), final_forces.end());
  EXPECT_LE(maximum_force_n, 15.5);
  EXPECT_LE(velocity_reversals, 2);
  EXPECT_LE(*final_range.second - *final_range.first, 0.40);
  EXPECT_NEAR(final_forces.back(), 15.0, 0.20);
}

}  // namespace fr_traction
