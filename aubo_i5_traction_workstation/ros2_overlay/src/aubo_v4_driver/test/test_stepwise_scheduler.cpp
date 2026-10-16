#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "aubo_v4_driver/stepwise_scheduler.hpp"

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
