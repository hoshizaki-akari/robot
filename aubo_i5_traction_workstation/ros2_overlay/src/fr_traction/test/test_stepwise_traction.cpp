#include <gtest/gtest.h>
#include <limits>
#include "fr_traction/stepwise_traction.hpp"

namespace ft = fr_traction;
namespace
{
ft::StepwiseTractionConfig verified()
{
  ft::StepwiseTractionConfig config;
  config.force_polarity_verified = true;
  config.drag_axes_verified = true;
  config.axis_sign = 1.0;
  return config;
}
}

TEST(StepwiseTraction, UnverifiedForceCannotMove)
{
  for (auto mode :
    {ft::ControlMode::PRETENSION, ft::ControlMode::TRACTION, ft::ControlMode::POSITIONING})
  {
    EXPECT_FALSE(ft::stepwise_traction_output({}, mode, {1, 0, 0}, 5, {1, 0, 0}).valid);
  }
}

TEST(StepwiseResponse, ReleasedLoadMustNotSeekTargetAgainUntilNewRun)
{
  ft::LoadReleaseDetector detector;
  for (unsigned i = 0; i < 100; ++i) {
    EXPECT_FALSE(detector.update(0.7, 14.0, 0.01, 1.2, 0.25));
  }
  EXPECT_FALSE(detector.update(12.0, 14.0, 0.01, 1.2, 0.25));
  for (unsigned i = 0; i < 15; ++i) {
    EXPECT_FALSE(detector.update(0.8, 14.0, 0.01, 1.2, 0.25));
  }
  EXPECT_FALSE(detector.update(3.0, 14.0, 0.01, 1.2, 0.25));
  for (unsigned i = 0; i < 30; ++i) {
    detector.update(0.8, 14.0, 0.01, 1.2, 0.25);
  }
  EXPECT_TRUE(detector.update(0.8, 14.0, 0.01, 1.2, 0.25));
  EXPECT_TRUE(detector.update(10.0, 14.0, 0.01, 1.2, 0.25));
  detector.reset();
  EXPECT_FALSE(detector.update(0.8, 14.0, 0.01, 1.2, 0.25));
}

TEST(StepwiseResponse, ForceApproachIsFasterButContinuousAtHoldBand)
{
  auto config = verified();
  config.maximum_speed_mps = 0.020;
  config.minimum_command_speed_mps = 0.0;
  config.force_gain_mps_per_n = 0.010;
  config.force_deadband_n = 0.15;
  config.force_soft_transition_n = 0.25;
  const auto out = [&](double error) {
      return ft::stepwise_traction_output(
        config, ft::ControlMode::TRACTION,
        {1, 0, 0}, 10.0, {10.0 - error, 0, 0});
    };
  EXPECT_GT(out(2.0).linear_velocity.x, 0.015);
  EXPECT_NEAR(out(5.0).linear_velocity.x, 0.020, 1e-12);
  EXPECT_NEAR(out(0.15).linear_velocity.x, 0.0, 1e-12);
  EXPECT_LT(ft::norm(out(0.1501).linear_velocity), 1e-8);
  EXPECT_LT(out(-2.0).linear_velocity.x, -0.015);
}

TEST(StepwiseResponse, VelocityRampDoesNotRetainOldDirectionOrDelayRelease)
{
  ft::Vec3 previous;
  EXPECT_NEAR(ft::ramp_stepwise_velocity(previous, {0.15, 0, 0}, 0.01, 0.6).x, 0.006, 1e-12);
  for (unsigned i = 0; i < 30; ++i) {
    ft::ramp_stepwise_velocity(previous, {0.15, 0, 0}, 0.01, 0.6);
  }
  EXPECT_NEAR(previous.x, 0.15, 1e-12);
  EXPECT_LT(ft::ramp_stepwise_velocity(previous, {-0.15, 0, 0}, 0.01, 0.6).x, 0.0);
  const auto y = ft::ramp_stepwise_velocity(previous, {0, 0.15, 0}, 0.01, 0.6);
  EXPECT_DOUBLE_EQ(y.x, 0.0);
  EXPECT_GT(y.y, 0.0);
  EXPECT_DOUBLE_EQ(ft::norm(ft::ramp_stepwise_velocity(previous, {}, 0.01, 0.6)), 0.0);
}

TEST(StepwiseResponse, DirectionChangeHasNoOldAxisFilterTail)
{
  ft::FirstOrderLowPass filter(12.0);
  ft::Vec3 previous;
  const auto x = ft::responsive_drag_force(filter, previous, {5, 0, 0}, 0.01, 0.15);
  EXPECT_GT(x.x, 0);
  const auto y = ft::responsive_drag_force(filter, previous, {0, 5, 0}, 0.01, 0.15);
  EXPECT_DOUBLE_EQ(y.x, 0);
  EXPECT_GT(y.y, 0);
  const auto opposite = ft::responsive_drag_force(filter, previous, {0, -5, 0}, 0.01, 0.15);
  EXPECT_LT(opposite.y, 0);
  const auto released = ft::responsive_drag_force(filter, previous, {}, 0.01, 0.15);
  EXPECT_DOUBLE_EQ(ft::norm(released), 0);
}

TEST(StepwiseResponse, ReleaseGateRejectsRecordedNoiseAndDoesNotRearmInHysteresisBand)
{
  ft::DragReleaseGate gate;
  for (double f : {0.0, 0.14, 0.16, 0.22, 0.5, 0.64}) {
    EXPECT_FALSE(gate.update(f, 0.65, 0.35));
  }
  EXPECT_TRUE(gate.update(5.5, 0.65, 0.35));
  EXPECT_TRUE(gate.update(0.5, 0.65, 0.35));
  EXPECT_FALSE(gate.update(0.35, 0.65, 0.35));
  EXPECT_FALSE(gate.update(0.6, 0.65, 0.35));
  EXPECT_TRUE(gate.update(0.65, 0.65, 0.35));
  EXPECT_FALSE(gate.update(std::numeric_limits<double>::quiet_NaN(), 0.65, 0.35));
}

TEST(StepwiseResponse, ZeroFloorIsContinuousAndStillLightAtFiveNewtons)
{
  auto config = verified();
  config.drag_deadband_n = 0.35;
  config.drag_gain_mps_per_n = 0.008;
  config.drag_response_scale = 4.0;
  config.drag_minimum_speed_mps = 0.0;
  config.drag_maximum_speed_mps = 0.15;
  for (auto axis : {ft::Vec3{1, 0, 0}, ft::Vec3{0, -1, 0}, ft::Vec3{0, 0, 1}}) {
    auto near = ft::stepwise_traction_output(
      config, ft::ControlMode::DRAGGING, {}, 0,
      axis * 0.35001);
    ASSERT_TRUE(near.valid);
    EXPECT_LT(ft::norm(near.linear_velocity), 0.000001);
    auto strong =
      ft::stepwise_traction_output(config, ft::ControlMode::DRAGGING, {}, 0, axis * 5.5);
    EXPECT_NEAR(ft::norm(strong.linear_velocity), 0.15, 1e-12);
  }
}

TEST(StepwiseResponse, FourfoldLowForceGainAndForceModesUsePracticalSpeeds)
{
  auto config = verified();
  config.drag_deadband_n = 0.15;
  config.drag_gain_mps_per_n = 0.080;
  config.drag_maximum_speed_mps = 0.050;
  const auto before = ft::stepwise_traction_output(
    config, ft::ControlMode::DRAGGING, {}, 0, {0.25, 0, 0});
  config.drag_response_scale = 4.0;
  const auto after = ft::stepwise_traction_output(
    config, ft::ControlMode::DRAGGING, {}, 0, {0.25, 0, 0});
  EXPECT_NEAR(ft::norm(after.linear_velocity), 4 * ft::norm(before.linear_velocity), 1e-12);
  config.force_gain_mps_per_n = 0.004;
  config.maximum_speed_mps = 0.020;
  config.force_deadband_n = 0.15;
  for (auto mode : {ft::ControlMode::TRACTION, ft::ControlMode::POSITIONING}) {
    const auto out = ft::stepwise_traction_output(config, mode, {1, 0, 0}, 10, {4, 0, 0});
    ASSERT_TRUE(out.valid);
    EXPECT_NEAR(ft::norm(out.linear_velocity), 0.020, 1e-12);
  }
}

TEST(StepwiseTraction, ResponsiveDragDoesNotAccelerateForceModes)
{
  auto config = verified();
  config.drag_deadband_n = 0.5;
  config.drag_gain_mps_per_n = 0.003;
  config.drag_maximum_speed_mps = 0.005;
  for (const auto & force : {ft::Vec3{2, 0, 0}, ft::Vec3{0, 2, 0}, ft::Vec3{0, 0, 2}}) {
    const auto out = ft::stepwise_traction_output(config, ft::ControlMode::DRAGGING, {}, 0, force);
    ASSERT_TRUE(out.valid);
    EXPECT_NEAR(ft::norm(out.linear_velocity), 0.0045, 1e-12);
    EXPECT_GT(ft::dot(out.linear_velocity, force), 0.0);
  }
  EXPECT_DOUBLE_EQ(
    ft::norm(
      ft::stepwise_traction_output(
        config, ft::ControlMode::DRAGGING, {}, 0, {0.5, 0, 0}).linear_velocity), 0.0);
  EXPECT_NEAR(
    ft::norm(
      ft::stepwise_traction_output(
        config, ft::ControlMode::DRAGGING, {}, 0, {0.6, 0, 0}).linear_velocity), 0.0012, 1e-12);
  EXPECT_NEAR(
    ft::norm(
      ft::stepwise_traction_output(
        config, ft::ControlMode::DRAGGING, {}, 0, {3, 4, 0}).linear_velocity), 0.005, 1e-12);
  for (auto mode : {ft::ControlMode::TRACTION, ft::ControlMode::POSITIONING}) {
    EXPECT_LE(
      ft::norm(
        ft::stepwise_traction_output(
          config, mode, {1, 0, 0}, 20, {}).linear_velocity), 0.002);
  }
  config.drag_maximum_speed_mps = 0.201;
  EXPECT_FALSE(
    ft::stepwise_traction_output(
      config, ft::ControlMode::DRAGGING, {}, 0, {2, 0, 0}).valid);
}

TEST(StepwiseTraction, PracticalDragHasResolvableFirstStepAndPreservesForceMode)
{
  auto config = verified();
  config.drag_deadband_n = 0.5;
  config.drag_gain_mps_per_n = 0.010;
  config.drag_minimum_speed_mps = 0.005;
  config.drag_maximum_speed_mps = 0.020;
  for (const auto & force : {ft::Vec3{0.8, 0, 0}, ft::Vec3{0, -0.8, 0}, ft::Vec3{0, 0, 0.8}}) {
    const auto output = ft::stepwise_traction_output(
      config, ft::ControlMode::DRAGGING, {}, 0, force);
    ASSERT_TRUE(output.valid);
    EXPECT_NEAR(ft::norm(output.linear_velocity), 0.005, 1e-12);
    EXPECT_GT(ft::dot(output.linear_velocity, force), 0);
    EXPECT_GE(ft::norm(output.linear_velocity) * 0.5, 0.0025 - 1e-12);
  }
  const auto held = ft::stepwise_traction_output(
    config, ft::ControlMode::DRAGGING, {}, 0, {0, 3, 4});
  EXPECT_NEAR(ft::norm(held.linear_velocity), 0.020, 1e-12);
  EXPECT_DOUBLE_EQ(
    ft::norm(
      ft::stepwise_traction_output(
        config, ft::ControlMode::DRAGGING, {}, 0, {}).linear_velocity), 0);
  EXPECT_LE(
    ft::norm(
      ft::stepwise_traction_output(
        config, ft::ControlMode::TRACTION, {1, 0, 0}, 20, {}).linear_velocity), 0.002);
}
TEST(StepwiseTraction, LightDragImprovesLowForceOnAllAxesWithinRaisedCeiling)
{
  auto config = verified();
  config.drag_deadband_n = 0.15;
  config.drag_gain_mps_per_n = 0.080;
  config.drag_minimum_speed_mps = 0.005;
  config.drag_maximum_speed_mps = 0.050;
  for (const auto & axis : {
    ft::Vec3{1, 0, 0}, ft::Vec3{-1, 0, 0}, ft::Vec3{0, 1, 0},
    ft::Vec3{0, -1, 0}, ft::Vec3{0, 0, 1}, ft::Vec3{0, 0, -1}})
  {
    for (const double force : {0.0, 0.15, 0.20, 0.25, 0.5, 1.0, 10.0}) {
      const auto output = ft::stepwise_traction_output(
        config, ft::ControlMode::DRAGGING, {}, 0, axis * force);
      ASSERT_TRUE(output.valid);
      const double expected = force <= 0.15 ? 0.0 :
        std::clamp(0.080 * (force - 0.15), 0.005, 0.050);
      EXPECT_NEAR(ft::norm(output.linear_velocity), expected, 1e-12);
      if (expected > 0.0) {EXPECT_GT(ft::dot(output.linear_velocity, axis), 0.0);}
    }
  }
  const auto light_push = ft::stepwise_traction_output(
    config, ft::ControlMode::DRAGGING, {}, 0, {0.25, 0.0, 0.0});
  EXPECT_NEAR(ft::norm(light_push.linear_velocity), 0.008, 1e-12);
}

TEST(StepwiseTraction, UnverifiedDragCannotMove)
{
  EXPECT_FALSE(ft::stepwise_traction_output({}, ft::ControlMode::DRAGGING, {}, 0, {5, 0, 0}).valid);
}
TEST(StepwiseTraction, DisabledNeverRequiresCalibration)
{
  const auto out = ft::stepwise_traction_output({}, ft::ControlMode::DISABLED, {}, 0, {});
  EXPECT_TRUE(out.valid);
  EXPECT_DOUBLE_EQ(ft::norm(out.linear_velocity), 0);
}
TEST(StepwiseTraction, ForceErrorAndPolarityDetermineDirection)
{
  auto config = verified();
  EXPECT_GT(
    ft::stepwise_traction_output(
      config, ft::ControlMode::TRACTION, {1, 0, 0}, 5, {1, 0,
        0}).linear_velocity.x, 0);
  EXPECT_LT(
    ft::stepwise_traction_output(
      config, ft::ControlMode::TRACTION, {1, 0, 0}, 1, {5, 0,
        0}).linear_velocity.x, 0);
  config.axis_sign = -1;
  EXPECT_LT(
    ft::stepwise_traction_output(
      config, ft::ControlMode::TRACTION, {1, 0, 0}, 5, {1, 0,
        0}).linear_velocity.x, 0);
}
TEST(StepwiseTraction, NoMomentumOrIntegrationAfterRelease)
{
  auto config = verified();
  ft::stepwise_traction_output(config, ft::ControlMode::TRACTION, {1, 0, 0}, 5, {1, 0, 0});
  const auto out = ft::stepwise_traction_output(
    config, ft::ControlMode::TRACTION, {1, 0, 0}, 5, {5,
      0, 0});
  EXPECT_DOUBLE_EQ(ft::norm(out.linear_velocity), 0);
}
TEST(StepwiseTraction, PositionUsesForceTargetAndExposesSettling)
{
  const auto out = ft::stepwise_traction_output(
    verified(), ft::ControlMode::POSITIONING, {1, 0, 0}, 5, {3, 4, 0});
  EXPECT_TRUE(out.valid);
  EXPECT_EQ(out.position_control.phase, ft::PositionControlPhase::SETTLING);
  EXPECT_DOUBLE_EQ(ft::norm(out.linear_velocity), 0);
}
TEST(StepwiseTraction, ConstantForceRespondsAgainOutsideDeadband)
{
  const auto config = verified();
  EXPECT_DOUBLE_EQ(
    ft::norm(
      ft::stepwise_traction_output(
        config, ft::ControlMode::TRACTION,
        {1, 0, 0}, 5, {5, 0, 0}).linear_velocity), 0);
  EXPECT_GT(
    ft::norm(
      ft::stepwise_traction_output(
        config, ft::ControlMode::TRACTION, {1, 0, 0}, 5,
        {3, 0, 0}).linear_velocity), 0);
}
TEST(StepwiseTraction, DragAxesAndLetGo)
{
  auto config = verified();
  for (auto force : {ft::Vec3{2, 0, 0}, ft::Vec3{0, 2, 0}, ft::Vec3{0, 0, 2}, ft::Vec3{-2, 0, 0}}) {
    const auto out = ft::stepwise_traction_output(config, ft::ControlMode::DRAGGING, {}, 0, force);
    EXPECT_GT(ft::dot(force, out.linear_velocity), 0);
  }
  EXPECT_DOUBLE_EQ(
    ft::norm(
      ft::stepwise_traction_output(
        config, ft::ControlMode::DRAGGING, {}, 0,
        {}).linear_velocity), 0);
}
TEST(StepwiseTraction, DefaultDragMaskRetainsAllAxes)
{
  const auto out = ft::stepwise_traction_output(
    verified(), ft::ControlMode::DRAGGING, {}, 0, {0, 0, 2});
  EXPECT_TRUE(out.valid);
  EXPECT_GT(out.linear_velocity.z, 0);
}
TEST(StepwiseTraction, AllSixDirectionsAndDiagonalPreserveFullForceDirection)
{
  auto config = verified();
  config.drag_axis_mask = {1, 1, 1};
  for (const auto force : {
    ft::Vec3{3, 0, 0}, ft::Vec3{-3, 0, 0}, ft::Vec3{0, 3, 0},
    ft::Vec3{0, -3, 0}, ft::Vec3{0, 0, 3}, ft::Vec3{0, 0, -3},
    ft::Vec3{2, -3, 4}, ft::Vec3{-2, 3, -4}})
  {
    const auto output = ft::stepwise_traction_output(
      config, ft::ControlMode::DRAGGING, {}, 0, force);
    ASSERT_TRUE(output.valid);
    const double speed = ft::norm(output.linear_velocity);
    ASSERT_GT(speed, 0);
    EXPECT_LE(speed, config.maximum_speed_mps + 1e-12);
    const auto expected = force * (speed / ft::norm(force));
    EXPECT_NEAR(output.linear_velocity.x, expected.x, 1e-12);
    EXPECT_NEAR(output.linear_velocity.y, expected.y, 1e-12);
    EXPECT_NEAR(output.linear_velocity.z, expected.z, 1e-12);
    const auto released = ft::stepwise_traction_output(
      config, ft::ControlMode::DRAGGING, {}, 0, {});
    EXPECT_DOUBLE_EQ(ft::norm(released.linear_velocity), 0);
  }
}
TEST(StepwiseTraction, ZDominantRecordedForceIsNotProjectedIntoXY)
{
  // Offline numerical regression from the reviewed session's approximate
  // tool-frame decomposition, NOT replay data in an operational controller.
  const ft::Vec3 force{-2.73147, -1.28915, -8.92812};
  const auto output = ft::stepwise_traction_output(
    verified(), ft::ControlMode::DRAGGING, {}, 0, force);
  ASSERT_TRUE(output.valid);
  EXPECT_LT(output.linear_velocity.z, 0);
  EXPECT_GT(std::abs(output.linear_velocity.z) / ft::norm(output.linear_velocity), 0.94);
}
TEST(StepwiseTraction, TractionAndPositionRetainArbitraryThreeDimensionalDirections)
{
  auto config = verified();
  for (const auto mode : {ft::ControlMode::TRACTION, ft::ControlMode::POSITIONING}) {
    for (const auto direction : {
      ft::Vec3{1, 0, 0}, ft::Vec3{-1, 0, 0}, ft::Vec3{0, 1, 0},
      ft::Vec3{0, -1, 0}, ft::Vec3{0, 0, 1}, ft::Vec3{0, 0, -1},
      ft::Vec3{1, -2, 3}})
    {
      const auto unit = direction * (1.0 / ft::norm(direction));
      for (const double sign : {-1.0, 1.0}) {
        config.axis_sign = sign;
        const auto output = ft::stepwise_traction_output(config, mode, direction, 2, unit);
        ASSERT_TRUE(output.valid);
        const double speed = ft::norm(output.linear_velocity);
        ASSERT_GT(speed, 0);
        const auto expected = unit * (sign * speed);
        EXPECT_NEAR(output.linear_velocity.x, expected.x, 1e-12);
        EXPECT_NEAR(output.linear_velocity.y, expected.y, 1e-12);
        EXPECT_NEAR(output.linear_velocity.z, expected.z, 1e-12);
      }
    }
  }
}
TEST(StepwiseTraction, DragRejectsAllAxesMasked)
{
  auto config = verified();
  config.drag_axis_mask = {0, 0, 0};
  EXPECT_FALSE(
    ft::stepwise_traction_output(config, ft::ControlMode::DRAGGING, {}, 0, {2, 0, 0}).valid);
}
TEST(StepwiseTraction, DragRejectsNonBinaryMask)
{
  auto config = verified();
  config.drag_axis_mask = {1, 0.5, 0};
  EXPECT_FALSE(
    ft::stepwise_traction_output(config, ft::ControlMode::DRAGGING, {}, 0, {2, 0, 0}).valid);
}
TEST(StepwiseTraction, DragMaskExcludesZBeforeMagnitudeAndVelocity)
{
  auto config = verified();
  config.drag_axis_mask = {1, 1, 0};
  const auto xy_only = ft::stepwise_traction_output(
    config, ft::ControlMode::DRAGGING, {}, 0, {2, -3, 0});
  const auto with_masked_z = ft::stepwise_traction_output(
    config, ft::ControlMode::DRAGGING, {}, 0, {2, -3, 100});
  EXPECT_TRUE(with_masked_z.valid);
  EXPECT_DOUBLE_EQ(with_masked_z.linear_velocity.x, xy_only.linear_velocity.x);
  EXPECT_DOUBLE_EQ(with_masked_z.linear_velocity.y, xy_only.linear_velocity.y);
  EXPECT_DOUBLE_EQ(with_masked_z.linear_velocity.z, 0.0);
}
TEST(StepwiseTraction, SmallErrorProducesResolvableStepNotSilentStall)
{
  const auto out = ft::stepwise_traction_output(
    verified(), ft::ControlMode::TRACTION,
    {1, 0, 0}, 5, {4.499, 0, 0});
  EXPECT_TRUE(out.valid);
  EXPECT_GT(ft::norm(out.linear_velocity) * 0.1, 2.0 * 0.00005);
  EXPECT_LE(ft::norm(out.linear_velocity) * 0.1, 0.0005);
}
TEST(StepwiseTraction, CombinedSpeedIsNormBounded)
{
  for (const double ceiling : {0.002, 0.020}) {
    auto config = verified();
    config.maximum_speed_mps = ceiling;
    const auto out = ft::stepwise_traction_output(
      config, ft::ControlMode::TRACTION, {1, 0, 0}, 100, {}, {0, 1, 1});
    EXPECT_TRUE(out.valid);
    EXPECT_LE(ft::norm(out.linear_velocity), ceiling + 1e-12);
  }
}
TEST(StepwiseTraction, ReleasingIsLateralOrReturnOnly)
{
  const auto out = ft::stepwise_traction_output(
    verified(), ft::ControlMode::RELEASING, {}, 100, {5, 0, 0}, {0, -0.001, 0});
  EXPECT_DOUBLE_EQ(out.linear_velocity.x, 0);
  EXPECT_LT(out.linear_velocity.y, 0);
}
TEST(StepwiseTraction, InvalidInputsFailClosed)
{
  EXPECT_FALSE(
    ft::stepwise_traction_output(
      verified(), ft::ControlMode::TRACTION, {}, 5, {1, 0, 0}).valid);
  EXPECT_FALSE(
    ft::stepwise_traction_output(
      verified(), ft::ControlMode::TRACTION, {1, 0, 0}, -1, {}).valid);
  EXPECT_FALSE(
    ft::stepwise_traction_output(
      verified(), ft::ControlMode::DRAGGING, {}, 0,
      {std::numeric_limits<double>::infinity(), 0, 0}).valid);
}
TEST(StepwiseTraction, LowTargetCannotAcceptUnloadedForce)
{
  const auto out = ft::stepwise_traction_output(
    verified(), ft::ControlMode::POSITIONING, {1, 0, 0}, 1, {});
  EXPECT_TRUE(out.valid);
  EXPECT_GT(ft::norm(out.linear_velocity), 0);
}
TEST(StepwiseTraction, GeometricReturnDoesNotRequireForcePolarity)
{
  const auto out = ft::stepwise_traction_output(
    ft::StepwiseTractionConfig{}, ft::ControlMode::RELEASING,
    {}, 0, {}, {0.0012, 0, 0});
  EXPECT_TRUE(out.valid);
  EXPECT_GT(out.linear_velocity.x, 0);
}
TEST(StepwiseTraction, MissingStaleOrFalseIdleCannotComplete)
{
  EXPECT_FALSE(ft::measured_idle_allows_completion(true, false, 0, .25));
  EXPECT_FALSE(ft::measured_idle_allows_completion(true, true, 1, .25));
  EXPECT_FALSE(ft::measured_idle_allows_completion(true, true, -1, .25));
  EXPECT_FALSE(
    ft::measured_idle_allows_completion(
      true, true,
      std::numeric_limits<double>::infinity(), .25));
  EXPECT_TRUE(ft::measured_idle_allows_completion(true, true, .1, .25));
  EXPECT_TRUE(ft::measured_idle_allows_completion(false, false, 1, .25));
}
