#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "aubo_v4_driver/stepwise_scheduler.hpp"
#include "aubo_v4_driver/stop_handoff.hpp"

namespace
{
using aubo_v4_driver::StepwiseCommand;
using aubo_v4_driver::StepwiseConfig;
using aubo_v4_driver::StepwiseFeedback;
using aubo_v4_driver::StepwiseScheduler;
using aubo_v4_driver::StepwiseState;

StepwiseFeedback feedback(
  std::uint64_t sequence, double timestamp, bool stopped = true, double speed = 0.0,
  std::array<double, 3> position = {})
{
  StepwiseFeedback result;
  result.sequence = sequence;
  result.received_at = timestamp;
  result.stopped = stopped;
  result.speed_mps = speed;
  result.position = position;
  result.healthy = true;
  return result;
}

void arm(StepwiseScheduler & scheduler, double now = 0.0)
{
  ASSERT_TRUE(scheduler.set_feedback(feedback(1U, now), now));
  ASSERT_TRUE(scheduler.set_enabled(true, now));
}

StepwiseCommand send(StepwiseScheduler & scheduler, const std::array<double, 3> & velocity)
{
  EXPECT_TRUE(scheduler.set_velocity(velocity, 0.01));
  const auto command = scheduler.take_command(0.01);
  EXPECT_TRUE(command.has_value());
  return command.value_or(StepwiseCommand{});
}

void acknowledge(StepwiseScheduler & scheduler, const StepwiseCommand & command)
{
  scheduler.acknowledge(command.id, command.epoch, true, 0.02);
  ASSERT_EQ(scheduler.state(), StepwiseState::MOVING);
}
}  // namespace

TEST(StepwiseRecovery, ThreeModesResetOnlyAfterFreshStopAndNeverReplayOldIntent)
{
  for (unsigned mode = 0; mode < 3; ++mode) {
    StepwiseScheduler scheduler;
    arm(scheduler);
    ASSERT_TRUE(scheduler.set_follow_mode(true, 0.001));
    scheduler.set_drag_mode(mode == 2);
    const auto old = send(scheduler, {0.002, 0, 0});
    scheduler.acknowledge(old.id, old.epoch, false, 0.02, "SDK timeout");
    EXPECT_FALSE(scheduler.reset_fault(0.021));
    const auto stop = scheduler.take_stop_request();
    ASSERT_TRUE(stop);
    scheduler.acknowledge_stop(*stop, true, 0.03);
    for (unsigned i = 2; i <= 4; ++i) {
      ASSERT_TRUE(scheduler.set_feedback(feedback(i, 0.02 + i * 0.01), 0.02 + i * 0.01));
    }
    ASSERT_EQ(scheduler.state(), StepwiseState::FAULT);
    EXPECT_FALSE(scheduler.reset_fault(0.7));  // Retained data is now stale.
    ASSERT_TRUE(scheduler.set_feedback(feedback(5, 0.71, false, 0.001), 0.71));
    EXPECT_FALSE(scheduler.reset_fault(0.711));
    ASSERT_TRUE(scheduler.set_feedback(feedback(6, 0.72), 0.72));
    ASSERT_TRUE(scheduler.reset_fault(0.721));
    EXPECT_EQ(scheduler.state(), StepwiseState::DISABLED);
    EXPECT_TRUE(scheduler.fault_reason().empty());
    EXPECT_GT(scheduler.epoch(), old.epoch);
    EXPECT_FALSE(scheduler.take_command(0.722));
    scheduler.acknowledge(old.id, old.epoch, true, 0.723);
    EXPECT_EQ(scheduler.state(), StepwiseState::DISABLED);
    ASSERT_TRUE(scheduler.set_enabled(true, 0.724));
    EXPECT_FALSE(scheduler.take_command(0.725));
    ASSERT_TRUE(scheduler.set_velocity({0.002, 0, 0}, 0.726));
    const auto next = scheduler.take_command(0.727);
    ASSERT_TRUE(next);
    EXPECT_GT(next->id, old.id);
    EXPECT_GT(next->epoch, old.epoch);
  }
}

TEST(StepwiseRecovery, FailedStopCannotBeResetUsingStationarySamplesAlone)
{
  StepwiseScheduler scheduler;
  arm(scheduler);
  const auto command = send(scheduler, {0.002, 0, 0});
  scheduler.acknowledge(command.id, command.epoch, false, 0.02);
  const auto stop = scheduler.take_stop_request();
  ASSERT_TRUE(stop);
  scheduler.acknowledge_stop(*stop, false, 0.03);
  for (unsigned i = 2; i < 7; ++i) {
    ASSERT_TRUE(scheduler.set_feedback(feedback(i, 0.02 + i * 0.01), 0.02 + i * 0.01));
  }
  EXPECT_FALSE(scheduler.reset_fault(0.09));
  EXPECT_EQ(scheduler.state(), StepwiseState::FAULT);
}

TEST(StepwiseCommunicationRecovery, VerifiedPauseKeepsTaskButDiscardsOldGoal)
{
  StepwiseScheduler scheduler;
  arm(scheduler);
  ASSERT_TRUE(scheduler.set_follow_mode(true, 0.001));
  const auto old = send(scheduler, {0.002, 0, 0});
  scheduler.acknowledge(old.id, old.epoch, false, 0.02, "SDK 10023", true);
  EXPECT_EQ(scheduler.state(), StepwiseState::STOPPING);
  EXPECT_FALSE(scheduler.fault_pending());
  const auto stop = scheduler.take_stop_request();
  ASSERT_TRUE(stop);
  EXPECT_FALSE(scheduler.take_command(0.021));
  scheduler.acknowledge_stop(*stop, true, 0.03);
  for (unsigned i = 2; i <= 4; ++i) {
    ASSERT_TRUE(scheduler.set_feedback(feedback(i, 0.02 + i * 0.01), 0.02 + i * 0.01));
  }
  EXPECT_EQ(scheduler.state(), StepwiseState::DISABLED);
  ASSERT_TRUE(scheduler.set_enabled(true, 0.07));
  EXPECT_FALSE(scheduler.take_command(0.071));
  ASSERT_TRUE(scheduler.set_velocity({-0.002, 0, 0}, 0.08));
  const auto next = scheduler.take_command(0.081);
  ASSERT_TRUE(next);
  EXPECT_GT(next->epoch, old.epoch);
  EXPECT_LT(next->target[0], next->start[0]);
}

TEST(StepwiseCommunicationRecovery, PersistentRejectionsCannotCreateUnlimitedAutomaticMotionTrials)
{
  StepwiseScheduler scheduler;
  arm(scheduler);
  ASSERT_TRUE(scheduler.set_follow_mode(true, 0.001));
  double time = 0.01;
  unsigned sequence = 1;
  for (unsigned attempt = 1; attempt <= 4; ++attempt) {
    ASSERT_TRUE(scheduler.set_velocity({0.002, 0, 0}, time));
    const auto target = scheduler.take_command(time);
    ASSERT_TRUE(target);
    time += 0.01;
    scheduler.acknowledge(target->id, target->epoch, false, time, "SDK 10023", true);
    EXPECT_EQ(scheduler.fault_pending(), attempt == 4);
    if (attempt == 4) {break;}
    const auto stop = scheduler.take_stop_request();
    ASSERT_TRUE(stop);
    time += 0.01;
    scheduler.acknowledge_stop(*stop, true, time);
    for (unsigned frame = 0; frame < 3; ++frame) {
      time += 0.01;
      ASSERT_TRUE(scheduler.set_feedback(feedback(++sequence, time), time));
    }
    time += 0.01;
    ASSERT_TRUE(scheduler.set_enabled(true, time));
  }
}

TEST(StepwiseSchedulerConfig, RejectsNonFiniteAndUnsafeValues)
{
  StepwiseConfig config;
  config.maximum_step_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(static_cast<void>(StepwiseScheduler{config}), std::invalid_argument);
  config = {};
  config.arrival_tolerance_m = config.maximum_step_m;
  EXPECT_THROW(static_cast<void>(StepwiseScheduler{config}), std::invalid_argument);
  config = {};
  config.maximum_speed_mps = 2.0;
  EXPECT_THROW(static_cast<void>(StepwiseScheduler{config}), std::invalid_argument);
  config = {};
  config.confirmation_frames = 0U;
  EXPECT_THROW(static_cast<void>(StepwiseScheduler{config}), std::invalid_argument);
}

TEST(StepwiseOrientation, UsesShortestQuaternionDistanceAndRejectsInvalidSamples)
{
  const auto rotated = [](double angle) {
      return std::array<double, 4>{0.0, 0.0, std::sin(angle / 2.0), std::cos(angle / 2.0)};
    };
  const std::array<double, 4> identity{0.0, 0.0, 0.0, 1.0};
  EXPECT_NEAR(aubo_v4_driver::quaternion_distance_rad(identity, rotated(0.02)), 0.02, 1e-12);
  EXPECT_LT(aubo_v4_driver::quaternion_distance_rad(identity, rotated(0.02)), 0.03);
  EXPECT_GT(aubo_v4_driver::quaternion_distance_rad(identity, rotated(0.04)), 0.03);
  const std::array<double, 4> opposite_sign{0.0, 0.0, 0.0, -1.0};
  EXPECT_NEAR(aubo_v4_driver::quaternion_distance_rad(identity, opposite_sign), 0.0, 1e-12);
  const std::array<double, 4> invalid{0.0, 0.0, 0.0, 0.0};
  EXPECT_FALSE(std::isfinite(aubo_v4_driver::quaternion_distance_rad(identity, invalid)));
}

TEST(StepwiseSchedulerResponse, CarriesClampedSpeedIndependentlyOfEndpointClipping)
{
  StepwiseConfig config;
  config.maximum_step_m = 0.001;
  config.maximum_speed_mps = 0.005;
  config.horizon_s = 0.5;
  StepwiseScheduler scheduler(config);
  arm(scheduler);
  const auto command = send(scheduler, {0.006, 0.008, 0.0});
  EXPECT_NEAR(command.speed_mps, 0.005, 1e-12);
  EXPECT_NEAR(command.target[0], 0.0006, 1e-12);
  EXPECT_NEAR(command.target[1], 0.0008, 1e-12);
  StepwiseScheduler slower(config);
  arm(slower);
  EXPECT_NEAR(send(slower, {0.0012, 0.0, 0.0}).speed_mps, 0.0012, 1e-12);
}

TEST(StepwiseSchedulerResponse, PracticalDragMovesFiveMmButSlowForceStillMovesOneMm)
{
  StepwiseConfig config;
  config.maximum_step_m = 0.005;
  config.maximum_speed_mps = 0.020;
  config.horizon_s = 0.5;
  config.arrival_tolerance_m = 0.0002;
  StepwiseScheduler fast(config);
  arm(fast);
  const auto drag = send(fast, {0, 0.012, 0.016});
  EXPECT_NEAR(std::hypot(drag.target[1], drag.target[2]), 0.005, 1e-12);
  EXPECT_DOUBLE_EQ(drag.speed_mps, 0.020);
  StepwiseScheduler slow(config);
  arm(slow);
  EXPECT_NEAR(send(slow, {0.002, 0, 0}).target[0], 0.001, 1e-12);
}

TEST(StepwiseFollow, UpdatesFreshLatestEndpointWithoutWaitingForStandstill)
{
  StepwiseConfig config;
  config.maximum_step_m = 0.005;
  config.maximum_speed_mps = 0.020;
  config.horizon_s = 0.5;
  StepwiseScheduler scheduler(config);
  arm(scheduler);
  ASSERT_TRUE(scheduler.set_follow_mode(true, 0));
  auto first = send(scheduler, {0.020, 0, 0});
  EXPECT_TRUE(first.follow_joint_mode);
  EXPECT_NEAR(first.target[0], 0.003, 1e-12);
  EXPECT_FALSE(scheduler.take_command(0.015));
  acknowledge(scheduler, first);
  ASSERT_TRUE(scheduler.set_feedback(feedback(2, 0.11, false, 0.01, {0.0005, 0, 0}), 0.11));
  ASSERT_TRUE(scheduler.set_velocity({0, 0.020, 0}, 0.11));
  const auto replacement = scheduler.take_command(0.111);
  ASSERT_TRUE(replacement);
  EXPECT_EQ(replacement->start[0], 0.0005);
  EXPECT_NEAR(replacement->target[1], 0.003, 1e-12);
  EXPECT_FALSE(scheduler.set_follow_mode(false, 0.112));
  scheduler.acknowledge(replacement->id, replacement->epoch, true, 0.12);
  ASSERT_TRUE(scheduler.set_velocity({}, 0.13));
  EXPECT_EQ(scheduler.state(), StepwiseState::STOPPING);
  EXPECT_FALSE(scheduler.take_command(0.14));
}

TEST(StepwiseFollow, AckWithoutPhysicalProgressDoesNotAllowEndlessTargets)
{
  StepwiseConfig config;
  config.maximum_step_m = 0.005;
  config.maximum_speed_mps = 0.020;
  config.horizon_s = 0.5;
  StepwiseScheduler scheduler(config);
  arm(scheduler);
  ASSERT_TRUE(scheduler.set_follow_mode(true, 0));
  auto current = send(scheduler, {0.020, 0, 0});
  acknowledge(scheduler, current);
  for (unsigned i = 1; i < 22 && !scheduler.fault_pending(); ++i) {
    const double now = i * 0.1 + 0.03;
    scheduler.set_feedback(feedback(i + 1, now, false), now);
    scheduler.set_velocity({0.020, 0, 0}, now);
    if (auto command = scheduler.take_command(now)) {
      scheduler.acknowledge(command->id, command->epoch, true, now);
    }
  }
  EXPECT_TRUE(scheduler.fault_pending());
  EXPECT_NE(scheduler.fault_reason().find("no measured progress"), std::string::npos);
}

TEST(StepwiseFollow, FineForceCorrectionDoesNotFaultBeforeMeasurableTravelIsRequested)
{
  StepwiseScheduler scheduler;
  arm(scheduler);
  ASSERT_TRUE(scheduler.set_follow_mode(true, 0));
  auto current = send(scheduler, {0.00001, 0, 0});
  acknowledge(scheduler, current);
  for (unsigned i = 1; i < 120 && !scheduler.fault_pending(); ++i) {
    const double now = i * 0.1 + 0.03;
    scheduler.set_feedback(feedback(i + 1, now, false), now);
    scheduler.set_velocity({0.00001, 0, 0}, now);
    if (auto next = scheduler.take_command(now)) {
      scheduler.acknowledge(next->id, next->epoch, true, now);
    }
    if (i == 30) {EXPECT_FALSE(scheduler.fault_pending());}
  }
  // Persistent lack of progress still faults once commanded net travel is
  // measurable. The change must not turn a stalled actuator into healthy idle.
  EXPECT_TRUE(scheduler.fault_pending());
  EXPECT_NE(scheduler.fault_reason().find("no measured progress"), std::string::npos);
}

TEST(StepwiseFollow, FailedAckRetainsSdkReasonAndOldEpochCannotOverwriteIt)
{
  StepwiseScheduler scheduler;
  arm(scheduler);
  auto current = send(scheduler, {0.002, 0, 0});
  scheduler.acknowledge(
    current.id, current.epoch, false, 0.02,
    "FollowModeJointMove SDK result 10007");
  EXPECT_NE(scheduler.fault_reason().find("10007"), std::string::npos);
  scheduler.acknowledge(current.id, current.epoch, false, 0.03, "stale cancellation");
  EXPECT_NE(scheduler.fault_reason().find("10007"), std::string::npos);
  EXPECT_EQ(scheduler.fault_reason().find("stale cancellation"), std::string::npos);
}

TEST(StepwiseFollow, ReleaseStillRequiresThreeDistinctPostStopFrames)
{
  StepwiseScheduler scheduler;
  arm(scheduler);
  ASSERT_TRUE(scheduler.set_follow_mode(true, 0));
  const auto command = send(scheduler, {0.002, 0, 0});
  acknowledge(scheduler, command);
  ASSERT_TRUE(scheduler.set_velocity({}, 0.03));
  const auto epoch = scheduler.take_stop_request();
  ASSERT_TRUE(epoch);
  scheduler.acknowledge_stop(*epoch, true, 0.04);
  scheduler.set_feedback(feedback(2, 0.05), 0.05);
  scheduler.set_feedback(feedback(2, 0.05), 0.06);
  EXPECT_EQ(scheduler.state(), StepwiseState::STOPPING);
  scheduler.set_feedback(feedback(3, 0.07), 0.07);
  EXPECT_EQ(scheduler.state(), StepwiseState::STOPPING);
  scheduler.set_feedback(feedback(4, 0.08), 0.08);
  EXPECT_EQ(scheduler.state(), StepwiseState::DISABLED);
}

TEST(StepwiseFollow, IdleBeforeFirstForceDoesNotConsumeProgressTimeout)
{
  StepwiseScheduler scheduler;
  arm(scheduler);
  ASSERT_TRUE(scheduler.set_follow_mode(true, 0));
  for (unsigned i = 1; i <= 30; ++i) {
    const double now = i * 0.1;
    ASSERT_TRUE(scheduler.set_feedback(feedback(i + 1, now), now));
    ASSERT_TRUE(scheduler.set_velocity({}, now));
    EXPECT_EQ(scheduler.state(), StepwiseState::READY);
  }
  ASSERT_TRUE(scheduler.set_velocity({0.002, 0, 0}, 3.01));
  const auto first = scheduler.take_command(3.02);
  ASSERT_TRUE(first);
  scheduler.acknowledge(first->id, first->epoch, true, 3.03);
  EXPECT_FALSE(scheduler.fault_pending());
  for (unsigned i = 31; i <= 50; ++i) {
    const double now = i * 0.1;
    scheduler.set_feedback(feedback(i + 1, now, false), now);
    scheduler.set_velocity({0.002, 0, 0}, now);
    if (auto next = scheduler.take_command(now)) {
      scheduler.acknowledge(next->id, next->epoch, true, now);
    }
    EXPECT_FALSE(scheduler.fault_pending());
  }
  scheduler.set_feedback(feedback(52, 5.1, false), 5.1);
  EXPECT_TRUE(scheduler.fault_pending());
}

TEST(StepwiseFollowResponse, ReversalViaZeroUsesMeasuredHoldWithoutFastStop)
{
  StepwiseConfig config;
  config.maximum_step_m = 0.005;
  config.maximum_speed_mps = 0.050;
  config.follow_update_interval_s = 0.03;
  config.follow_horizon_s = 0.08;
  config.follow_hold_enabled = true;
  StepwiseScheduler scheduler(config);
  arm(scheduler);
  ASSERT_TRUE(scheduler.set_follow_mode(true, 0));
  const auto first = send(scheduler, {0.05, 0, 0});
  acknowledge(scheduler, first);
  scheduler.set_feedback(feedback(2, 0.05, false, 0.02, {0.001, 0, 0}), 0.05);
  scheduler.set_velocity({}, 0.05);
  const auto hold = scheduler.take_command(0.051);
  ASSERT_TRUE(hold);
  EXPECT_TRUE(hold->hold_position);
  EXPECT_EQ(hold->target, hold->start);
  EXPECT_FALSE(scheduler.take_stop_request());
  scheduler.acknowledge(hold->id, hold->epoch, true, 0.06);
  scheduler.set_feedback(feedback(3, 0.09, false, 0.01, {0.0011, 0, 0}), 0.09);
  scheduler.set_velocity({-0.05, 0, 0}, 0.09);
  const auto reverse = scheduler.take_command(0.091);
  ASSERT_TRUE(reverse);
  EXPECT_LT(reverse->target[0], reverse->start[0]);
  EXPECT_FALSE(reverse->hold_position);
  EXPECT_FALSE(scheduler.take_stop_request());
}

TEST(StepwiseFollowResponse, HoldSettlesOnceAndCanRestartOnAnotherAxis)
{
  StepwiseConfig config;
  config.follow_hold_enabled = true;
  config.follow_update_interval_s = 0.03;
  StepwiseScheduler scheduler(config);
  arm(scheduler);
  scheduler.set_follow_mode(true, 0);
  auto command = send(scheduler, {0.002, 0, 0});
  acknowledge(scheduler, command);
  scheduler.set_feedback(feedback(2, 0.05), 0.05);
  scheduler.set_velocity({}, 0.05);
  auto hold = scheduler.take_command(0.051);
  ASSERT_TRUE(hold);
  scheduler.acknowledge(hold->id, hold->epoch, true, 0.06);
  for (unsigned i = 3; i <= 9; ++i) {
    const double t = 0.06 + (i - 2) * 0.03;
    scheduler.set_feedback(feedback(i, t), t);
    scheduler.set_velocity({}, t);
    EXPECT_FALSE(scheduler.take_command(t));
  }
  EXPECT_TRUE(scheduler.motion_idle(0.27));
  scheduler.set_velocity({0, 0.002, 0}, 0.28);
  const auto y = scheduler.take_command(0.28);
  ASSERT_TRUE(y);
  EXPECT_GT(y->target[1], y->start[1]);
}

TEST(StepwiseFollowResponse, SubArrivalToleranceContinuousForceStepIsNotDiscarded)
{
  StepwiseConfig config;
  config.maximum_step_m = 0.005;
  config.arrival_tolerance_m = 0.0002;
  config.follow_horizon_s = 0.08;
  config.follow_update_interval_s = 0.03;
  StepwiseScheduler scheduler(config);
  arm(scheduler);
  scheduler.set_follow_mode(true, 0);
  const auto command = send(scheduler, {0.0005, 0, 0});
  EXPECT_GT(command.target[0], 0);
  EXPECT_NEAR(command.target[0], 0.00004, 1e-12);
}

TEST(StepwiseFollowResponse, FixedProfileReferenceAdvancesByTimeAndRebasesOnReversal)
{
  StepwiseConfig config;
  config.maximum_step_m = 0.005;
  config.maximum_speed_mps = 0.15;
  config.follow_update_interval_s = 0.03;
  config.follow_horizon_s = 0.08;
  config.follow_reference_enabled = true;
  StepwiseScheduler scheduler(config);
  arm(scheduler);
  scheduler.set_follow_mode(true, 0);
  auto first = send(scheduler, {0.05, 0, 0});
  EXPECT_NEAR(first.target[0], 0.0015, 1e-12);
  acknowledge(scheduler, first);
  scheduler.set_feedback(feedback(2, 0.05, false, 0.05, {0.0015, 0, 0}), 0.05);
  scheduler.set_velocity({0.05, 0, 0}, 0.05);
  auto next = scheduler.take_command(0.05);
  ASSERT_TRUE(next);
  EXPECT_NEAR(next->target[0], 0.0035, 1e-12);
  scheduler.acknowledge(next->id, next->epoch, true, 0.06);
  scheduler.set_feedback(feedback(3, 0.09, false, 0.05, {0.0035, 0, 0}), 0.09);
  scheduler.set_velocity({-0.05, 0, 0}, 0.09);
  auto reverse = scheduler.take_command(0.09);
  ASSERT_TRUE(reverse);
  EXPECT_NEAR(reverse->target[0], 0.0020, 1e-12);
}

TEST(StepwiseFollowResponse, OvertakenReferenceCannotCommandMotionAgainstCurrentForce)
{
  StepwiseConfig config;
  config.maximum_step_m = 0.005;
  config.maximum_speed_mps = 0.15;
  config.follow_update_interval_s = 0.03;
  config.follow_horizon_s = 0.08;
  config.follow_reference_enabled = true;
  StepwiseScheduler scheduler(config);
  arm(scheduler);
  scheduler.set_follow_mode(true, 0);
  const auto first = send(scheduler, {0.05, 0, 0});
  acknowledge(scheduler, first);
  scheduler.set_feedback(feedback(2, 0.05, false, 0.05, {0.005, 0, 0}), 0.05);
  scheduler.set_velocity({0.05, 0, 0}, 0.05);
  auto next = scheduler.take_command(0.05);
  ASSERT_TRUE(next);
  EXPECT_GT(next->target[0], next->start[0]);
}

TEST(StepwiseFollowResponse, DragForcesKeepDifferentLeadDistancesInsteadOfSameFiveMillimetres)
{
  const auto lead = [](double velocity, bool drag) {
      StepwiseConfig config;
      config.maximum_step_m = 0.005;
      config.maximum_speed_mps = 0.20;
      config.horizon_s = 0.5;
      config.follow_update_interval_s = 0.03;
      config.follow_horizon_s = 0.16;
      config.follow_reference_enabled = true;
      config.drag_maximum_step_m = 0.020;
      config.drag_follow_horizon_s = 0.10;
      StepwiseScheduler scheduler(config);
      arm(scheduler);
      scheduler.set_drag_mode(drag);
      scheduler.set_follow_mode(true, 0);
      const auto first = send(scheduler, {velocity, 0, 0});
      acknowledge(scheduler, first);
      double result = first.target[0];
      for (unsigned i = 1; i <= 15; ++i) {
        const double now = i * 0.04;
        scheduler.set_feedback(feedback(i + 2, now, false, 0.001), now);
        scheduler.set_velocity({velocity, 0, 0}, now);
        const auto next = scheduler.take_command(now);
        if (next) {
          result = next->target[0];
          EXPECT_EQ(next->assisted_drag, drag);
          scheduler.acknowledge(next->id, next->epoch, true, now);
        }
      }
      return result;
    };
  EXPECT_NEAR(lead(0.0465, true), 0.00465, 1e-12);
  EXPECT_NEAR(lead(0.0965, true), 0.00965, 1e-12);
  EXPECT_NEAR(lead(0.1965, true), 0.01965, 1e-12);
  EXPECT_LE(lead(0.1965, false), 0.0050000001);
}

TEST(StepwiseFollowResponse, DelayedSdkReplyDoesNotAccumulateACatchupJump)
{
  StepwiseConfig config;
  config.maximum_step_m = 0.005;
  config.maximum_speed_mps = 0.15;
  config.follow_update_interval_s = 0.03;
  config.follow_horizon_s = 0.16;
  config.horizon_s = 0.5;
  config.follow_reference_enabled = true;
  StepwiseScheduler scheduler(config);
  arm(scheduler);
  scheduler.set_follow_mode(true, 0);
  const auto first = send(scheduler, {0.02, 0, 0});
  scheduler.acknowledge(first.id, first.epoch, true, 0.18);
  scheduler.set_feedback(feedback(2, 0.20, false, 0.02, first.target), 0.20);
  scheduler.set_velocity({0.02, 0, 0}, 0.20);
  const auto next = scheduler.take_command(0.20);
  ASSERT_TRUE(next);
  EXPECT_NEAR(next->target[0] - first.target[0], 0.0012, 1e-12);
}

TEST(SessionStopHandoff, RejectedAndMissingRepliesCannotBecomeNormalCompletion)
{
  using aubo_v4_driver::StopHandoff;
  using aubo_v4_driver::stop_handoff_status;
  EXPECT_EQ(stop_handoff_status(false, false, 1, 2), StopHandoff::PENDING);
  EXPECT_EQ(stop_handoff_status(true, false, 1, 2), StopHandoff::REJECTED);
  EXPECT_EQ(stop_handoff_status(true, true, 1, 2), StopHandoff::ACCEPTED);
  EXPECT_EQ(stop_handoff_status(false, false, 2, 2), StopHandoff::TIMED_OUT);
  EXPECT_EQ(stop_handoff_status(true, true, 2.01, 2), StopHandoff::TIMED_OUT);
}

TEST(StepwiseSchedulerActivation, RequiresFreshHealthyQuietStoppedFeedback)
{
  StepwiseScheduler scheduler;
  EXPECT_FALSE(scheduler.set_enabled(true, 0.0));
  EXPECT_TRUE(scheduler.set_feedback(feedback(1U, 0.0, false), 0.0));
  EXPECT_FALSE(scheduler.set_enabled(true, 0.0));
  EXPECT_TRUE(scheduler.set_feedback(feedback(2U, 0.1, true, 0.001), 0.1));
  EXPECT_FALSE(scheduler.set_enabled(true, 0.1));
  EXPECT_TRUE(scheduler.set_feedback(feedback(3U, 0.2), 0.2));
  EXPECT_TRUE(scheduler.set_enabled(true, 0.2));
}

TEST(StepwiseSchedulerActivation, RequiresVelocityNewerThanActivation)
{
  StepwiseScheduler scheduler;
  ASSERT_TRUE(scheduler.set_velocity({0.002, 0.0, 0.0}, 0.0));
  ASSERT_TRUE(scheduler.set_feedback(feedback(1U, 0.0), 0.0));
  ASSERT_TRUE(scheduler.set_enabled(true, 0.0));
  EXPECT_FALSE(scheduler.take_command(0.01).has_value());
  ASSERT_TRUE(scheduler.set_velocity({0.002, 0.0, 0.0}, 0.01));
  EXPECT_TRUE(scheduler.take_command(0.01).has_value());
}

TEST(StepwiseSchedulerActivation, ZeroOrStaleNewVelocityCannotStartMotion)
{
  StepwiseConfig config;
  config.command_timeout_s = 0.04;
  StepwiseScheduler scheduler(config);
  arm(scheduler);
  ASSERT_TRUE(scheduler.set_velocity({0.0, 0.0, 0.0}, 0.01));
  EXPECT_FALSE(scheduler.take_command(0.01).has_value());
  ASSERT_TRUE(scheduler.set_velocity({0.002, 0.0, 0.0}, 0.02));
  scheduler.poll(0.07);
  EXPECT_EQ(scheduler.state(), StepwiseState::READY);
  EXPECT_FALSE(scheduler.take_command(0.07).has_value());
}

TEST(StepwiseSchedulerCommand, ClipsNormWithFixedHorizon)
{
  StepwiseScheduler scheduler;
  arm(scheduler);
  const StepwiseCommand command = send(scheduler, {10.0, 0.0, 0.0});
  EXPECT_NEAR(command.target[0] - command.start[0], 0.0002, 1e-12);
  EXPECT_NEAR(command.target[1] - command.start[1], 0.0, 1e-12);
}

TEST(StepwiseSchedulerCommand, CapsStepNormAndPreservesSigns)
{
  StepwiseConfig config;
  config.maximum_speed_mps = 1.0;
  StepwiseScheduler scheduler(config);
  arm(scheduler);
  const StepwiseCommand command = send(scheduler, {-1.0, 1.0, -1.0});
  const double norm = std::sqrt(
    std::pow(command.target[0] - command.start[0], 2) +
    std::pow(command.target[1] - command.start[1], 2) +
    std::pow(command.target[2] - command.start[2], 2));
  EXPECT_LE(norm, config.maximum_step_m + 1e-12);
  EXPECT_LT(command.target[0], command.start[0]);
  EXPECT_GT(command.target[1], command.start[1]);
  EXPECT_LT(command.target[2], command.start[2]);
}

TEST(StepwiseSchedulerCommand, DoesNotQueueASecondCommand)
{
  StepwiseScheduler scheduler;
  arm(scheduler);
  static_cast<void>(send(scheduler, {0.002, 0.0, 0.0}));
  ASSERT_TRUE(scheduler.set_velocity({0.0, 0.001, 0.0}, 0.02));
  EXPECT_FALSE(scheduler.take_command(0.02).has_value());
}

TEST(StepwiseSchedulerCommand, DoesNotDispatchSubToleranceStep)
{
  StepwiseScheduler scheduler;
  arm(scheduler);
  ASSERT_TRUE(scheduler.set_velocity({0.0005, 0.0, 0.0}, 0.01));
  EXPECT_FALSE(scheduler.take_command(0.01).has_value());
  EXPECT_EQ(scheduler.state(), StepwiseState::READY);
  EXPECT_EQ(scheduler.reason(), "belowminimumstep");
}

TEST(StepwiseSchedulerCommand, DoesNotDispatchNumericallyUnchangedLargePosition)
{
  StepwiseScheduler scheduler;
  ASSERT_TRUE(scheduler.set_feedback(feedback(1U, 0.0, true, 0.0, {1.0e16, 0.0, 0.0}), 0.0));
  ASSERT_TRUE(scheduler.set_enabled(true, 0.0));
  ASSERT_TRUE(scheduler.set_velocity({0.002, 0.0, 0.0}, 0.01));
  EXPECT_FALSE(scheduler.take_command(0.01).has_value());
  EXPECT_EQ(scheduler.state(), StepwiseState::READY);
  EXPECT_EQ(scheduler.reason(), "belowminimumstep");
}

TEST(StepwiseSchedulerFeedback, RejectsInvalidQuaternion)
{
  StepwiseScheduler scheduler;
  StepwiseFeedback sample = feedback(1U, 0.0);
  sample.orientation = {0.0, 0.0, 0.0, 0.0};
  EXPECT_FALSE(scheduler.set_feedback(sample, 0.0));
  EXPECT_EQ(scheduler.state(), StepwiseState::FAULT);
}

TEST(StepwiseSchedulerFeedback, DuplicateFramesDoNotCountForCompletion)
{
  StepwiseScheduler scheduler;
  arm(scheduler);
  const auto command = send(scheduler, {0.002, 0.0, 0.0});
  acknowledge(scheduler, command);
  const auto at_target = command.target;
  ASSERT_TRUE(scheduler.set_feedback(feedback(2U, 0.03, true, 0.0, at_target), 0.03));
  ASSERT_TRUE(scheduler.set_feedback(feedback(2U, 0.04, true, 0.0, at_target), 0.04));
  EXPECT_EQ(scheduler.state(), StepwiseState::MOVING);
}

TEST(StepwiseSchedulerFeedback, DuplicateUnhealthyFeedbackRequestsFaultStop)
{
  StepwiseScheduler scheduler;
  arm(scheduler);
  StepwiseFeedback duplicate = feedback(1U, 0.01);
  duplicate.healthy = false;
  ASSERT_TRUE(scheduler.set_feedback(duplicate, 0.01));
  EXPECT_EQ(scheduler.state(), StepwiseState::STOPPING);
  EXPECT_TRUE(scheduler.fault_pending());
}

TEST(StepwiseSchedulerFeedback, DuplicateMovingFlagBlocksCommandWithoutRefreshingPose)
{
  StepwiseScheduler scheduler;
  arm(scheduler);
  StepwiseFeedback duplicate = feedback(1U, 0.01, false);
  ASSERT_TRUE(scheduler.set_feedback(duplicate, 0.01));
  ASSERT_TRUE(scheduler.set_velocity({0.002, 0.0, 0.0}, 0.02));
  EXPECT_EQ(scheduler.state(), StepwiseState::READY);
  EXPECT_FALSE(scheduler.take_command(0.02).has_value());
  EXPECT_FALSE(scheduler.motion_idle(0.02));
}

TEST(StepwiseSchedulerFeedback, StaleSequenceFailsClosedDuringMotion)
{
  StepwiseScheduler scheduler;
  arm(scheduler);
  const auto command = send(scheduler, {0.002, 0.0, 0.0});
  acknowledge(scheduler, command);
  ASSERT_TRUE(scheduler.set_feedback(feedback(3U, 0.03, false), 0.03));
  EXPECT_FALSE(scheduler.set_feedback(feedback(2U, 0.04, false), 0.04));
  EXPECT_EQ(scheduler.state(), StepwiseState::STOPPING);
}

TEST(StepwiseSchedulerAcknowledgement, LateAckCannotBeatTimeout)
{
  StepwiseConfig config;
  config.ack_timeout_s = 0.05;
  StepwiseScheduler scheduler(config);
  arm(scheduler);
  const auto command = send(scheduler, {0.002, 0.0, 0.0});
  scheduler.acknowledge(command.id, command.epoch, true, 0.07);
  EXPECT_EQ(scheduler.state(), StepwiseState::STOPPING);
  EXPECT_NE(scheduler.reason().find("timeout"), std::string::npos);
}

TEST(StepwiseSchedulerAcknowledgement, WrongAckDoesNotAdvanceState)
{
  StepwiseScheduler scheduler;
  arm(scheduler);
  const auto command = send(scheduler, {0.002, 0.0, 0.0});
  scheduler.acknowledge(command.id + 1U, command.epoch, true, 0.02);
  EXPECT_EQ(scheduler.state(), StepwiseState::SENDING);
}

TEST(StepwiseSchedulerAcknowledgement, MatchingIdWithWrongEpochDoesNotAdvanceState)
{
  StepwiseScheduler scheduler;
  arm(scheduler);
  const auto command = send(scheduler, {0.002, 0.0, 0.0});
  scheduler.acknowledge(command.id, command.epoch + 1U, true, 0.02);
  EXPECT_EQ(scheduler.state(), StepwiseState::SENDING);
}

TEST(StepwiseSchedulerAcknowledgement, FailedAckRequestsFaultStop)
{
  StepwiseScheduler scheduler;
  arm(scheduler);
  const auto command = send(scheduler, {0.002, 0.0, 0.0});
  scheduler.acknowledge(command.id, command.epoch, false, 0.02);
  EXPECT_EQ(scheduler.state(), StepwiseState::STOPPING);
  EXPECT_TRUE(scheduler.take_stop_request().has_value());
}

TEST(StepwiseSchedulerMotion, ZeroVelocityDuringMotionStops)
{
  StepwiseScheduler scheduler;
  arm(scheduler);
  const auto command = send(scheduler, {0.002, 0.0, 0.0});
  acknowledge(scheduler, command);
  ASSERT_TRUE(scheduler.set_velocity({0.0, 0.0, 0.0}, 0.03));
  EXPECT_EQ(scheduler.state(), StepwiseState::STOPPING);
}

TEST(StepwiseSchedulerMotion, HugeFiniteVelocityNormFailsClosed)
{
  StepwiseScheduler scheduler;
  arm(scheduler);
  EXPECT_FALSE(
    scheduler.set_velocity(
  {
    std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), 0.0}, 0.01));
  EXPECT_EQ(scheduler.state(), StepwiseState::STOPPING);
  EXPECT_TRUE(scheduler.fault_pending());
}

TEST(StepwiseSchedulerMotion, StaleVelocityDuringMotionStops)
{
  StepwiseConfig config;
  config.command_timeout_s = 0.04;
  StepwiseScheduler scheduler(config);
  arm(scheduler);
  const auto command = send(scheduler, {0.002, 0.0, 0.0});
  acknowledge(scheduler, command);
  scheduler.poll(0.06);
  EXPECT_EQ(scheduler.state(), StepwiseState::STOPPING);
}

TEST(StepwiseSchedulerMotion, StaleVelocityInReadyOnlyClearsIt)
{
  StepwiseConfig config;
  config.command_timeout_s = 0.04;
  StepwiseScheduler scheduler(config);
  arm(scheduler);
  ASSERT_TRUE(scheduler.set_velocity({0.002, 0.0, 0.0}, 0.01));
  scheduler.poll(0.06);
  EXPECT_EQ(scheduler.state(), StepwiseState::READY);
  EXPECT_FALSE(scheduler.take_command(0.06).has_value());
}

TEST(StepwiseSchedulerCompletion, WrongTargetCannotConfirmArrival)
{
  StepwiseScheduler scheduler;
  arm(scheduler);
  const auto command = send(scheduler, {0.002, 0.0, 0.0});
  acknowledge(scheduler, command);
  ASSERT_TRUE(scheduler.set_feedback(feedback(2U, 0.03, true, 0.0, {0.1, 0.0, 0.0}), 0.03));
  ASSERT_TRUE(scheduler.set_feedback(feedback(3U, 0.04, true, 0.0, {0.1, 0.0, 0.0}), 0.04));
  ASSERT_TRUE(scheduler.set_feedback(feedback(4U, 0.05, true, 0.0, {0.1, 0.0, 0.0}), 0.05));
  EXPECT_EQ(scheduler.state(), StepwiseState::MOVING);
}

TEST(StepwiseSchedulerCompletion, RequiresDistinctStoppedTargetFramesAndSettlement)
{
  StepwiseConfig config;
  config.settling_time_s = 0.05;
  StepwiseScheduler scheduler(config);
  arm(scheduler);
  const auto command = send(scheduler, {0.002, 0.0, 0.0});
  acknowledge(scheduler, command);
  ASSERT_TRUE(scheduler.set_feedback(feedback(2U, 0.03, true, 0.0, command.target), 0.03));
  ASSERT_TRUE(scheduler.set_feedback(feedback(3U, 0.04, true, 0.0, command.target), 0.04));
  ASSERT_TRUE(scheduler.set_feedback(feedback(4U, 0.05, true, 0.0, command.target), 0.05));
  EXPECT_EQ(scheduler.state(), StepwiseState::SETTLING);
  scheduler.poll(0.079);
  EXPECT_EQ(scheduler.state(), StepwiseState::SETTLING);
  scheduler.poll(0.081);
  EXPECT_EQ(scheduler.state(), StepwiseState::READY);
}

TEST(StepwiseSchedulerCompletion, DeadlineIncludesSettlement)
{
  StepwiseConfig config;
  config.completion_timeout_s = 0.08;
  config.settling_time_s = 0.1;
  StepwiseScheduler scheduler(config);
  arm(scheduler);
  const auto command = send(scheduler, {0.002, 0.0, 0.0});
  acknowledge(scheduler, command);
  ASSERT_TRUE(scheduler.set_feedback(feedback(2U, 0.03, true, 0.0, command.target), 0.03));
  ASSERT_TRUE(scheduler.set_feedback(feedback(3U, 0.04, true, 0.0, command.target), 0.04));
  ASSERT_TRUE(scheduler.set_feedback(feedback(4U, 0.05, true, 0.0, command.target), 0.05));
  ASSERT_EQ(scheduler.state(), StepwiseState::SETTLING);
  scheduler.poll(0.091);
  EXPECT_EQ(scheduler.state(), StepwiseState::STOPPING);
}

TEST(StepwiseSchedulerCompletion, DeadlineAlsoAppliesBeforeAcknowledgement)
{
  StepwiseConfig config;
  config.ack_timeout_s = 0.2;
  config.completion_timeout_s = 0.05;
  StepwiseScheduler scheduler(config);
  arm(scheduler);
  const auto command = send(scheduler, {0.002, 0.0, 0.0});
  scheduler.acknowledge(command.id, command.epoch, true, 0.07);
  EXPECT_EQ(scheduler.state(), StepwiseState::STOPPING);
  EXPECT_NE(scheduler.reason().find("completion"), std::string::npos);
}

TEST(StepwiseSchedulerStop, RequestIsSingleUseAndEpochInvalidatesLateAck)
{
  StepwiseScheduler scheduler;
  arm(scheduler);
  const auto command = send(scheduler, {0.002, 0.0, 0.0});
  const auto previous_epoch = scheduler.epoch();
  scheduler.request_stop(0.02, "operator stop");
  ASSERT_EQ(scheduler.state(), StepwiseState::STOPPING);
  EXPECT_GT(scheduler.epoch(), previous_epoch);
  const auto stop_epoch = scheduler.take_stop_request();
  ASSERT_TRUE(stop_epoch.has_value());
  EXPECT_FALSE(scheduler.take_stop_request().has_value());
  scheduler.acknowledge(command.id, command.epoch, true, 0.03);
  EXPECT_EQ(scheduler.state(), StepwiseState::STOPPING);
}

TEST(StepwiseSchedulerStop, NormalStopNeedsAckAndThreeNewFrames)
{
  StepwiseScheduler scheduler;
  arm(scheduler);
  scheduler.request_stop(0.01, "operator stop");
  const auto stop_epoch = scheduler.take_stop_request();
  ASSERT_TRUE(stop_epoch.has_value());
  scheduler.acknowledge_stop(*stop_epoch, true, 0.02);
  ASSERT_TRUE(scheduler.set_feedback(feedback(2U, 0.03), 0.03));
  ASSERT_TRUE(scheduler.set_feedback(feedback(3U, 0.04), 0.04));
  EXPECT_EQ(scheduler.state(), StepwiseState::STOPPING);
  ASSERT_TRUE(scheduler.set_feedback(feedback(4U, 0.05), 0.05));
  EXPECT_EQ(scheduler.state(), StepwiseState::DISABLED);
}

TEST(StepwiseSchedulerStop, FeedbackAfterStopDeadlineCannotResurrectDisabled)
{
  StepwiseConfig config;
  config.ack_timeout_s = 0.05;
  StepwiseScheduler scheduler(config);
  arm(scheduler);
  scheduler.request_stop(0.01, "operator stop");
  const auto stop_epoch = scheduler.take_stop_request();
  ASSERT_TRUE(stop_epoch.has_value());
  scheduler.acknowledge_stop(*stop_epoch, true, 0.02);
  ASSERT_TRUE(scheduler.set_feedback(feedback(2U, 0.07), 0.07));
  EXPECT_EQ(scheduler.state(), StepwiseState::FAULT);
  EXPECT_TRUE(scheduler.fault_pending());
}

TEST(StepwiseSchedulerStop, DuplicateStopCannotExtendDeadlineOrDowngradeFault)
{
  StepwiseConfig config;
  config.ack_timeout_s = 0.05;
  StepwiseScheduler scheduler(config);
  arm(scheduler);
  scheduler.request_stop(0.01, "normal stop");
  const auto stop_epoch = scheduler.take_stop_request();
  ASSERT_TRUE(stop_epoch.has_value());
  scheduler.request_stop(0.02, "fault stop", true);
  EXPECT_EQ(scheduler.epoch(), *stop_epoch);
  EXPECT_FALSE(scheduler.take_stop_request().has_value());
  EXPECT_TRUE(scheduler.fault_pending());
  scheduler.request_stop(0.03, "normal retry", false);
  EXPECT_TRUE(scheduler.fault_pending());
  scheduler.poll(0.061);
  EXPECT_EQ(scheduler.state(), StepwiseState::FAULT);
}

TEST(StepwiseSchedulerStop, StaleFeedbackCannotConfirmAcknowledgedStop)
{
  StepwiseScheduler scheduler;
  arm(scheduler);
  scheduler.request_stop(0.01, "operator stop");
  const auto stop_epoch = scheduler.take_stop_request();
  ASSERT_TRUE(stop_epoch.has_value());
  scheduler.acknowledge_stop(*stop_epoch, true, 0.02);
  ASSERT_TRUE(scheduler.set_feedback(feedback(2U, 0.10), 0.70));
  ASSERT_TRUE(scheduler.set_feedback(feedback(3U, 0.15), 0.70));
  ASSERT_TRUE(scheduler.set_feedback(feedback(4U, 0.20), 0.70));
  EXPECT_EQ(scheduler.state(), StepwiseState::STOPPING);
}

TEST(StepwiseSchedulerStop, PreAckOrEqualTimestampFeedbackCannotConfirmStop)
{
  StepwiseScheduler scheduler;
  arm(scheduler);
  scheduler.request_stop(0.01, "operator stop");
  const auto stop_epoch = scheduler.take_stop_request();
  ASSERT_TRUE(stop_epoch.has_value());
  scheduler.acknowledge_stop(*stop_epoch, true, 0.02);
  ASSERT_TRUE(scheduler.set_feedback(feedback(2U, 0.02), 0.02));
  ASSERT_TRUE(scheduler.set_feedback(feedback(3U, 0.03), 0.03));
  ASSERT_TRUE(scheduler.set_feedback(feedback(4U, 0.04), 0.04));
  ASSERT_TRUE(scheduler.set_feedback(feedback(5U, 0.05), 0.05));
  EXPECT_EQ(scheduler.state(), StepwiseState::DISABLED);
}

TEST(StepwiseSchedulerStop, FailedStopAckIsStickyFault)
{
  StepwiseScheduler scheduler;
  arm(scheduler);
  scheduler.request_stop(0.01, "operator stop");
  const auto stop_epoch = scheduler.take_stop_request();
  ASSERT_TRUE(stop_epoch.has_value());
  scheduler.acknowledge_stop(*stop_epoch, false, 0.02);
  EXPECT_EQ(scheduler.state(), StepwiseState::FAULT);
  EXPECT_FALSE(scheduler.set_enabled(true, 0.03));
}

TEST(StepwiseSchedulerStop, StopTimeoutIsStickyFault)
{
  StepwiseConfig config;
  config.ack_timeout_s = 0.05;
  StepwiseScheduler scheduler(config);
  arm(scheduler);
  scheduler.request_stop(0.01, "operator stop");
  scheduler.poll(0.07);
  EXPECT_EQ(scheduler.state(), StepwiseState::FAULT);
}

TEST(StepwiseSchedulerStop, IdleNormalStopStaysDisabledAndInvalidatesEpoch)
{
  StepwiseScheduler scheduler;
  const auto old_epoch = scheduler.epoch();
  scheduler.request_stop(0.0, "clear cache");
  EXPECT_EQ(scheduler.state(), StepwiseState::DISABLED);
  EXPECT_GT(scheduler.epoch(), old_epoch);
  EXPECT_FALSE(scheduler.take_stop_request().has_value());
}

TEST(StepwiseSchedulerClock, BackwardClockFailsClosed)
{
  StepwiseScheduler scheduler;
  arm(scheduler, 1.0);
  ASSERT_TRUE(scheduler.set_velocity({0.002, 0.0, 0.0}, 1.01));
  EXPECT_FALSE(scheduler.set_velocity({0.002, 0.0, 0.0}, 1.0));
  EXPECT_EQ(scheduler.state(), StepwiseState::STOPPING);
}

TEST(StepwiseSchedulerStatus, HealthyAndMotionIdleHaveStrictMeaning)
{
  StepwiseScheduler scheduler;
  ASSERT_TRUE(scheduler.set_feedback(feedback(1U, 0.0), 0.0));
  EXPECT_TRUE(scheduler.healthy(0.1));
  EXPECT_TRUE(scheduler.motion_idle(0.1));
  ASSERT_TRUE(scheduler.set_enabled(true, 0.1));
  ASSERT_TRUE(scheduler.set_velocity({0.002, 0.0, 0.0}, 0.11));
  static_cast<void>(scheduler.take_command(0.11));
  EXPECT_FALSE(scheduler.motion_idle(0.11));
  EXPECT_FALSE(scheduler.healthy(0.6));
}

TEST(StepwiseSchedulerHardwareResolution, OneMillimetreProfileAcceptsObservedEndpointError)
{
  StepwiseConfig config;
  config.maximum_step_m = 0.001;
  config.horizon_s = 0.5;
  config.arrival_tolerance_m = 0.0002;
  StepwiseScheduler scheduler(config);
  arm(scheduler);
  const auto command = send(scheduler, {0.002, 0.0, 0.0});
  acknowledge(scheduler, command);
  const std::array<double, 3> actual{command.target[0] + 0.0000768, 0.0000613, 0.0};
  ASSERT_TRUE(scheduler.set_feedback(feedback(2U, 0.03, true, 0.0, actual), 0.03));
  ASSERT_TRUE(scheduler.set_feedback(feedback(3U, 0.05, true, 0.0, actual), 0.05));
  ASSERT_TRUE(scheduler.set_feedback(feedback(4U, 0.07, true, 0.0, actual), 0.07));
  EXPECT_EQ(scheduler.state(), StepwiseState::SETTLING);
  scheduler.poll(0.18);
  EXPECT_EQ(scheduler.state(), StepwiseState::READY);
  ASSERT_TRUE(scheduler.set_velocity({0.0012, 0.0, 0.0}, 0.19));
  EXPECT_TRUE(scheduler.take_command(0.19).has_value());
}

TEST(StepwiseSchedulerStop, PreservesOriginalFaultReasonAfterStopConfirmation)
{
  StepwiseScheduler scheduler;
  arm(scheduler);
  const auto command = send(scheduler, {0.002, 0.0, 0.0});
  acknowledge(scheduler, command);
  scheduler.request_stop(0.03, "original completion failure", true);
  const auto epoch = scheduler.take_stop_request();
  ASSERT_TRUE(epoch.has_value());
  scheduler.acknowledge_stop(*epoch, true, 0.04);
  for (std::uint64_t sequence = 2; sequence <= 4; ++sequence) {
    const double time = 0.03 + 0.01 * static_cast<double>(sequence);
    ASSERT_TRUE(scheduler.set_feedback(feedback(sequence, time), time));
  }
  EXPECT_EQ(scheduler.state(), StepwiseState::FAULT);
  EXPECT_EQ(scheduler.fault_reason(), "original completion failure");
}

TEST(StepwiseSchedulerCompletion, MeasuredOrientationErrorDoesNotAccumulateIntoTargets)
{
  StepwiseScheduler scheduler;
  arm(scheduler);
  const auto first = send(scheduler, {0.002, 0.0, 0.0});
  acknowledge(scheduler, first);
  for (std::uint64_t sequence = 2; sequence <= 4; ++sequence) {
    const double time = 0.01 + 0.02 * static_cast<double>(sequence);
    auto measured = feedback(sequence, time, true, 0.0, first.target);
    measured.orientation = {0.0, 0.0, std::sin(0.0005), std::cos(0.0005)};
    ASSERT_TRUE(scheduler.set_feedback(measured, time));
  }
  scheduler.poll(0.20);
  ASSERT_EQ(scheduler.state(), StepwiseState::READY);
  ASSERT_TRUE(scheduler.set_velocity({0.002, 0.0, 0.0}, 0.21));
  const auto second = scheduler.take_command(0.21);
  ASSERT_TRUE(second.has_value());
  EXPECT_EQ(second->orientation, first.orientation);
}

TEST(StepwiseSchedulerStop, IdleDisableRevokesIntentWithoutRedundantSdkStop)
{
  StepwiseScheduler scheduler;
  arm(scheduler);
  ASSERT_TRUE(scheduler.set_velocity({0.002, 0.0, 0.0}, 0.01));
  const auto before_epoch = scheduler.epoch();
  ASSERT_TRUE(scheduler.set_enabled(false, 0.02));
  EXPECT_GT(scheduler.epoch(), before_epoch);
  EXPECT_EQ(scheduler.state(), StepwiseState::DISABLED);
  EXPECT_TRUE(scheduler.motion_idle(0.02));
  EXPECT_FALSE(scheduler.take_command(0.02).has_value());
  EXPECT_FALSE(scheduler.take_stop_request().has_value());
}

TEST(StepwiseSchedulerStop, DisablingAnInflightMotionStillRequiresSdkStop)
{
  StepwiseScheduler scheduler;
  arm(scheduler);
  const auto command = send(scheduler, {0.002, 0.0, 0.0});
  acknowledge(scheduler, command);
  ASSERT_TRUE(scheduler.set_enabled(false, 0.03));
  EXPECT_EQ(scheduler.state(), StepwiseState::STOPPING);
  EXPECT_TRUE(scheduler.take_stop_request().has_value());
  EXPECT_FALSE(scheduler.motion_idle(0.03));
}
