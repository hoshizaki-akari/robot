#include <gtest/gtest.h>
#include <cmath>
#include "aubo_v4_driver/aubo_drag_reference.hpp"
#include "aubo_v4_driver/stepwise_scheduler.hpp"

namespace av = aubo_v4_driver;

TEST(AuboDragReference, BurstCallbackTimingDoesNotCreateReleaseVelocityImpulse)
{
  av::CartesianVelocityWindow window;
  window.push({0, 0, 0}, 0.0);
  window.push({0.001, 0, 0}, 0.04);
  window.push({0.002, 0, 0}, 0.0401);
  const auto velocity = window.velocity();
  EXPECT_LT(velocity[0], 0.051);  // Raw adjacent difference would be 10 m/s.
  EXPECT_DOUBLE_EQ(velocity[1], 0.0);
  window.push({0.002, 0, 0}, 0.1);
  window.push({0.002, 0, 0}, 0.2);
  EXPECT_DOUBLE_EQ(window.velocity()[0], 0.0);
  window.push({1, 0, 0}, 0.6);
  EXPECT_DOUBLE_EQ(window.velocity()[0], 0.0);  // Gap invalidates extrapolation.
}

TEST(AuboDragReference, UsesFreshPoseWithoutAccumulatingOrRotatingOldPositionError)
{
  av::StepwiseConfig config;
  config.maximum_speed_mps = 0.2;
  config.follow_update_interval_s = 0.03;
  config.follow_horizon_s = 0.1;
  config.horizon_s = 0.1;
  config.drag_follow_horizon_s = 0.06;
  config.drag_maximum_step_m = 0.02;
  config.follow_hold_enabled = true;
  config.follow_reference_enabled = true;
  config.independent_drag_reference = true;
  av::StepwiseScheduler scheduler(config);
  av::StepwiseFeedback feedback;
  feedback.healthy = feedback.stopped = true;
  feedback.sequence = 1;
  ASSERT_TRUE(scheduler.set_feedback(feedback, 0));
  ASSERT_TRUE(scheduler.set_enabled(true, 0));
  ASSERT_TRUE(scheduler.set_follow_mode(true, 0));
  scheduler.set_drag_mode(true);
  for (unsigned i = 1; i <= 30; ++i) {
    const double time = 0.04 * i;
    feedback.sequence++;
    feedback.received_at = time;
    ASSERT_TRUE(scheduler.set_feedback(feedback, time));
    const std::array<double, 3> velocity = i < 15 ?
      std::array<double, 3>{0.05, 0, 0.05} : std::array<double, 3>{0, -0.05, 0};
    ASSERT_TRUE(scheduler.set_velocity(velocity, time));
    const auto command = scheduler.take_command(time);
    ASSERT_TRUE(command);
    EXPECT_TRUE(command->cartesian_drag_reference);
    for (unsigned axis = 0; axis < 3; ++axis) {
      EXPECT_NEAR(command->target[axis] - command->start[axis], velocity[axis] * 0.06, 1e-12);
    }
    scheduler.acknowledge(command->id, command->epoch, true, time + 0.001);
  }
}

TEST(AuboDragReference, OffsetUsesOneCommonScaleAndZeroInputNeverMoves)
{
  const auto offset = av::aubo_drag_offset({0.1, 0.2, -0.3}, 0.1, 0.02);
  EXPECT_NEAR(offset[1], 2.0 * offset[0], 1e-12);
  EXPECT_NEAR(offset[2], -3.0 * offset[0], 1e-12);
  EXPECT_NEAR(std::hypot(offset[0], offset[1], offset[2]), 0.02, 1e-12);
  EXPECT_EQ(av::aubo_drag_offset({0, 0, 0}, 0.1, 0.02), (std::array<double, 3>{}));
}

TEST(AuboDragReference, SupersededUnsentCommandDoesNotFaultOrReplayAnOldDirection)
{
  av::StepwiseConfig config;
  config.independent_drag_reference = true;
  config.follow_update_interval_s = 0.03;
  config.follow_hold_enabled = true;
  av::StepwiseScheduler scheduler(config);
  av::StepwiseFeedback feedback;
  feedback.healthy = feedback.stopped = true;
  feedback.sequence = 1;
  ASSERT_TRUE(scheduler.set_feedback(feedback, 0.0));
  ASSERT_TRUE(scheduler.set_enabled(true, 0.0));
  ASSERT_TRUE(scheduler.set_follow_mode(true, 0.0));
  scheduler.set_drag_mode(true);
  ASSERT_TRUE(scheduler.set_velocity({0.002, 0, 0}, 0.01));
  const auto old = scheduler.take_command(0.01);
  ASSERT_TRUE(old);
  ASSERT_TRUE(scheduler.discard_drag_command(old->id, old->epoch, 0.02));
  EXPECT_FALSE(scheduler.fault_pending());
  EXPECT_FALSE(scheduler.take_stop_request());
  feedback.sequence = 2;
  feedback.received_at = 0.06;
  ASSERT_TRUE(scheduler.set_feedback(feedback, 0.06));
  ASSERT_TRUE(scheduler.set_velocity({0, -0.002, 0}, 0.06));
  const auto next = scheduler.take_command(0.06);
  ASSERT_TRUE(next);
  EXPECT_DOUBLE_EQ(next->target[0], next->start[0]);
  EXPECT_LT(next->target[1], next->start[1]);
  EXPECT_EQ(next->epoch, old->epoch);
}
