#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <aubo_driver/serviceinterface.h>
#include "aubo_v4_driver/stepwise_transport.hpp"

namespace
{
using aubo_v4_driver::StepwiseCommand;
using aubo_v4_driver::StepwiseTransport;
using aubo_v4_driver::TransportOptions;
using aubo_v4_driver::TransportResult;
using namespace std::chrono_literals;

TransportOptions options()
{
  TransportOptions value;
  value.host = "offline-sdk-double";
  value.username = "test-user";
  value.password = "test-password";
  value.maximum_step_m = 0.0005;
  return value;
}

StepwiseCommand command(std::uint64_t id, std::uint64_t epoch)
{
  StepwiseCommand value;
  value.id = id;
  value.epoch = epoch;
  value.start = {0.0998, 0.20, 0.30};
  value.target = {0.10, 0.20, 0.30};
  value.orientation = {0.0, 0.0, 0.0, 1.0};
  return value;
}

const std::array<double, 6> seed{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

bool connect(StepwiseTransport & transport)
{
  std::string error;
  return transport.connect(options(), error);
}

int constructor_count()
{
  auto & state = aubo_v4_test_sdk::state();
  std::lock_guard<std::mutex> lock(state.mutex);
  return state.constructor_count;
}

int line_move_count()
{
  auto & state = aubo_v4_test_sdk::state();
  std::lock_guard<std::mutex> lock(state.mutex);
  return state.line_move_count;
}

int ik_call_count()
{
  auto & state = aubo_v4_test_sdk::state();
  std::lock_guard<std::mutex> lock(state.mutex);
  return state.ik_call_count;
}

int stop_count()
{
  auto & state = aubo_v4_test_sdk::state();
  std::lock_guard<std::mutex> lock(state.mutex);
  return state.stop_count;
}

bool wait_for_diagnosis(StepwiseTransport & transport, std::chrono::milliseconds timeout)
{
  std::size_t seen = 0;
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (transport.sample().diagnosis_ok) {return true;}
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
      deadline - std::chrono::steady_clock::now());
    aubo_v4_test_sdk::wait_for_event(seen, std::min(remaining, 50ms));
  }
  return transport.sample().diagnosis_ok;
}

std::optional<TransportResult> wait_for_motion_result(
  StepwiseTransport & transport, std::chrono::milliseconds timeout)
{
  std::size_t seen = 0;
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (const auto result = transport.take_result()) {
      if (!result->stop) {return result;}
    }
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
      deadline - std::chrono::steady_clock::now());
    aubo_v4_test_sdk::wait_for_event(seen, std::min(remaining, 50ms));
  }
  return std::nullopt;
}

std::optional<TransportResult> wait_for_stop_result(
  StepwiseTransport & transport, std::chrono::milliseconds timeout)
{
  std::size_t seen = 0;
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (const auto result = transport.take_result()) {
      if (result->stop) {return result;}
    }
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
      deadline - std::chrono::steady_clock::now());
    aubo_v4_test_sdk::wait_for_event(seen, std::min(remaining, 50ms));
  }
  return std::nullopt;
}

bool submit_when_available(
  StepwiseTransport & transport, const StepwiseCommand & value,
  std::chrono::milliseconds timeout)
{
  std::size_t seen = 0;
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (transport.submit(value, seed)) {return true;}
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
      deadline - std::chrono::steady_clock::now());
    aubo_v4_test_sdk::wait_for_event(seen, std::min(remaining, 50ms));
  }
  return transport.submit(value, seed);
}

std::vector<std::string> calls()
{
  return aubo_v4_test_sdk::call_order();
}

std::size_t index_of(const std::vector<std::string> & values, const std::string & value)
{
  const auto found = std::find(values.begin(), values.end(), value);
  return static_cast<std::size_t>(std::distance(values.begin(), found));
}

aubo_robot_namespace::wayPoint_S valid_waypoint()
{
  aubo_robot_namespace::wayPoint_S point{};
  point.cartPos.position = {0.1, 0.2, 0.3};
  point.orientation.w = 1.0;
  return point;
}

class StepwiseTransportTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    aubo_v4_test_sdk::reset();
  }

  void TearDown() override
  {
    // Every test owns its transport in a scope, so no SDK-double worker can
    // still be using the shared script before the next reset.
    auto & state = aubo_v4_test_sdk::state();
    std::lock_guard<std::mutex> lock(state.mutex);
    ASSERT_EQ(state.constructor_count, state.destructor_count);
  }
};

TEST_F(StepwiseTransportTest, ConstructionAndDestructionCreateNoSdkObjects)
{
  {
    StepwiseTransport transport;
  }
  EXPECT_EQ(constructor_count(), 0);
}

TEST_F(StepwiseTransportTest, CommandSpeedIsAppliedAndReportedBeforeMotion)
{
  StepwiseTransport transport;
  ASSERT_TRUE(connect(transport));
  auto value = command(1, 1);
  value.speed_mps = 0.0013;
  ASSERT_TRUE(transport.submit(value, seed));
  const auto result = wait_for_motion_result(transport, 1s);
  ASSERT_TRUE(result);
  EXPECT_TRUE(result->success);
  const auto sample = transport.sample();
  EXPECT_DOUBLE_EQ(sample.sdk_speed_mps, value.speed_mps);
  EXPECT_DOUBLE_EQ(sample.requested_speed_mps, value.speed_mps);
  EXPECT_EQ(sample.motion_ack_count, 1U);
  EXPECT_EQ(sample.last_command_id, 1U);
}

TEST_F(StepwiseTransportTest, IKSdkFailureIsReportedSeparatelyFromJointContinuity)
{
  StepwiseTransport transport;
  ASSERT_TRUE(connect(transport));
  {
    auto & state = aubo_v4_test_sdk::state();
    std::lock_guard<std::mutex> lock(state.mutex);
    state.ik_result = 37;
  }
  ASSERT_TRUE(transport.submit(command(1, 1), seed));
  const auto result = wait_for_motion_result(transport, 1s);
  ASSERT_TRUE(result);
  EXPECT_FALSE(result->success);
  EXPECT_EQ(result->detail, "robotServiceRobotIk SDK result 37");
  const auto sample = transport.sample();
  EXPECT_EQ(sample.last_ik_result, 37);
  EXPECT_EQ(sample.last_ik_failure, result->detail);
  EXPECT_EQ(line_move_count(), 0);
}

TEST_F(StepwiseTransportTest, IKJointContinuityRejectionReportsJointAndDelta)
{
  StepwiseTransport transport;
  ASSERT_TRUE(connect(transport));
  {
    auto & state = aubo_v4_test_sdk::state();
    std::lock_guard<std::mutex> lock(state.mutex);
    state.ik_delta[4] = 0.06;
  }
  ASSERT_TRUE(transport.submit(command(1, 1), seed));
  const auto result = wait_for_motion_result(transport, 1s);
  ASSERT_TRUE(result);
  EXPECT_FALSE(result->success);
  EXPECT_NE(
    result->detail.find("IK joint 5 delta 0.060000 rad exceeds 0.050000 rad"),
    std::string::npos);
  const auto sample = transport.sample();
  EXPECT_EQ(sample.last_ik_result, 0);
  EXPECT_DOUBLE_EQ(sample.last_ik_max_joint_delta_rad, 0.06);
  EXPECT_EQ(sample.last_ik_joint_index, 5U);
  EXPECT_EQ(sample.last_ik_failure, result->detail);
  EXPECT_EQ(line_move_count(), 0);
}

TEST_F(StepwiseTransportTest, InvalidSpeedAndPollIntervalFailClosed)
{
  StepwiseTransport transport;
  auto settings = options();
  settings.state_poll_interval_s = 0.0;
  std::string error;
  EXPECT_FALSE(transport.connect(settings, error));
  EXPECT_EQ(constructor_count(), 0);
  ASSERT_TRUE(connect(transport));
  for (double speed : {0.0, -0.001, 0.003, std::numeric_limits<double>::quiet_NaN()}) {
    auto value = command(1, 1);
    value.speed_mps = speed;
    EXPECT_FALSE(transport.submit(value, seed));
  }
  EXPECT_EQ(ik_call_count(), 0);
}

TEST_F(StepwiseTransportTest, ProfileReadbackFailurePreventsMotion)
{
  StepwiseTransport transport;
  ASSERT_TRUE(connect(transport));
  {
    auto & state = aubo_v4_test_sdk::state();
    std::lock_guard<std::mutex> lock(state.mutex);
    state.bad_profile_readback = true;
  }
  ASSERT_TRUE(transport.submit(command(1, 1), seed));
  const auto result = wait_for_motion_result(transport, 1s);
  ASSERT_TRUE(result);
  EXPECT_FALSE(result->success);
  EXPECT_EQ(line_move_count(), 0);
}

TEST_F(StepwiseTransportTest, CancellationDuringProfileUpdateCannotDispatchLateMotion)
{
  StepwiseTransport transport;
  ASSERT_TRUE(connect(transport));
  auto & state = aubo_v4_test_sdk::state();
  {
    std::lock_guard<std::mutex> lock(state.mutex);
    state.block_profile = true;
    state.profile_entered = false;
  }
  ASSERT_TRUE(transport.submit(command(1, 1), seed));
  bool entered;
  {
    std::unique_lock<std::mutex> lock(state.mutex);
    entered = state.changed.wait_for(lock, 1s, [&state]() {return state.profile_entered;});
  }
  transport.cancel_and_stop(2);
  {
    std::lock_guard<std::mutex> lock(state.mutex);
    state.release_profile = true;
    state.changed.notify_all();
  }
  EXPECT_TRUE(entered);
  const auto result = wait_for_motion_result(transport, 1s);
  ASSERT_TRUE(result);
  EXPECT_FALSE(result->success);
  EXPECT_EQ(line_move_count(), 0);
  const auto stopped = wait_for_stop_result(transport, 1s);
  ASSERT_TRUE(stopped);
  EXPECT_TRUE(stopped->success);
}

TEST_F(StepwiseTransportTest, InvalidOptionsRejectBeforeSdkConstruction)
{
  StepwiseTransport transport;
  TransportOptions invalid;
  std::string error;
  EXPECT_FALSE(transport.connect(invalid, error));
  EXPECT_NE(error.find("invalid"), std::string::npos);
  EXPECT_EQ(constructor_count(), 0);
}

TEST_F(StepwiseTransportTest, FastDragProfileReturnsToOriginalSlowForceProfile)
{
  StepwiseTransport transport;
  auto settings = options();
  settings.speed_mps = 0.020;
  settings.maximum_step_m = 0.005;
  settings.fast_acceleration_mps2 = 0.05;
  std::string error;
  ASSERT_TRUE(transport.connect(settings, error));
  auto fast = command(1, 1);
  fast.speed_mps = 0.020;
  fast.target[0] = fast.start[0] + 0.005;
  ASSERT_TRUE(transport.submit(fast, seed));
  const auto result = wait_for_motion_result(transport, 1s);
  ASSERT_TRUE(result);
  ASSERT_TRUE(result->success);
  EXPECT_DOUBLE_EQ(transport.sample().sdk_acceleration_mps2, 0.05);
  auto slow = command(2, 1);
  ASSERT_TRUE(submit_when_available(transport, slow, 1s));
  const auto slow_result = wait_for_motion_result(transport, 1s);
  ASSERT_TRUE(slow_result);
  EXPECT_TRUE(slow_result->success);
  EXPECT_DOUBLE_EQ(transport.sample().sdk_acceleration_mps2, 0.01);
  EXPECT_DOUBLE_EQ(transport.sample().sdk_speed_mps, 0.002);
}

TEST_F(StepwiseTransportTest, FollowModeRequiresOptInAndChecksJointProfiles)
{
  StepwiseTransport transport;
  auto settings = options();
  settings.allow_follow_mode = true;
  std::string error;
  ASSERT_TRUE(transport.connect(settings, error));
  auto value = command(1, 1);
  value.follow_joint_mode = true;
  ASSERT_TRUE(transport.submit(value, seed));
  const auto result = wait_for_motion_result(transport, 1s);
  ASSERT_TRUE(result);
  EXPECT_TRUE(result->success);
  EXPECT_TRUE(transport.sample().follow_joint_mode);
  auto & state = aubo_v4_test_sdk::state();
  std::lock_guard<std::mutex> lock(state.mutex);
  EXPECT_EQ(state.follow_move_count, 1);
  EXPECT_EQ(state.line_move_count, 0);
  for (unsigned i = 0; i < 6; ++i) {
    EXPECT_GT(state.joint_velocity.jointPara[i], 0);
    EXPECT_LE(state.joint_velocity.jointPara[i], 0.2);
    EXPECT_LE(state.joint_acceleration.jointPara[i], 0.5);
  }
}

TEST_F(StepwiseTransportTest, DisabledFollowCannotReachSdk)
{
  StepwiseTransport transport;
  ASSERT_TRUE(connect(transport));
  auto value = command(1, 1);
  value.follow_joint_mode = true;
  EXPECT_FALSE(transport.submit(value, seed));
  EXPECT_EQ(ik_call_count(), 0);
}

TEST_F(StepwiseTransportTest, FollowStopCannotBeConfirmedBeforeSenderQuiesces)
{
  StepwiseTransport transport;
  auto settings = options();
  settings.allow_follow_mode = true;
  std::string error;
  ASSERT_TRUE(transport.connect(settings, error));
  {
    auto & state = aubo_v4_test_sdk::state();
    std::lock_guard<std::mutex> lock(state.mutex);
    state.block_line_move = true;
  }
  auto value = command(1, 1);
  value.follow_joint_mode = true;
  ASSERT_TRUE(transport.submit(value, seed));
  ASSERT_TRUE(aubo_v4_test_sdk::wait_for_line_move(500ms));
  transport.cancel_and_stop(2);
  EXPECT_TRUE(aubo_v4_test_sdk::wait_for_stop_count(1, 500ms));
  aubo_v4_test_sdk::release_line_move();
  const auto result = wait_for_stop_result(transport, 1s);
  ASSERT_TRUE(result);
  EXPECT_TRUE(result->success);
  const auto order = calls();
  EXPECT_LT(index_of(order, "follow_move_exit"), index_of(order, "stop_final"));
  EXPECT_FALSE(transport.submit(value, seed));
}

TEST_F(StepwiseTransportTest, ConsumedFollowAckAlwaysMeansSenderSlotIsReleased)
{
  StepwiseTransport transport;
  auto settings = options();
  settings.allow_follow_mode = true;
  std::string error;
  ASSERT_TRUE(transport.connect(settings, error));
  for (unsigned id = 1; id <= 100; ++id) {
    auto value = command(id, 1);
    value.follow_joint_mode = true;
    ASSERT_TRUE(transport.submit(value, seed));
    const auto result = wait_for_motion_result(transport, 1s);
    ASSERT_TRUE(result);
    ASSERT_TRUE(result->success);
  }
}

TEST_F(StepwiseTransportTest, RepeatedConnectIsRejected)
{
  StepwiseTransport transport;
  ASSERT_TRUE(connect(transport));
  std::string error;
  EXPECT_FALSE(transport.connect(options(), error));
  EXPECT_NE(error.find("repeated connect"), std::string::npos);
  EXPECT_EQ(constructor_count(), 3);
}

TEST_F(StepwiseTransportTest, LoginFailureSendsNoMotion)
{
  aubo_v4_test_sdk::set_login_results({aubo_robot_namespace::ErrnoSucc, -1});
  StepwiseTransport transport;
  std::string error;
  EXPECT_FALSE(transport.connect(options(), error));
  EXPECT_NE(error.find("login failed"), std::string::npos);
  EXPECT_EQ(line_move_count(), 0);
}

TEST_F(StepwiseTransportTest, OnlyOneSubmitMayBeInflight)
{
  aubo_v4_test_sdk::state().block_ik = true;
  aubo_v4_test_sdk::state().block_line_move = true;
  StepwiseTransport transport;
  ASSERT_TRUE(connect(transport));
  ASSERT_TRUE(transport.submit(command(1, 1), seed));
  ASSERT_TRUE(aubo_v4_test_sdk::wait_for_ik(500ms));
  EXPECT_FALSE(transport.submit(command(2, 1), seed));
  aubo_v4_test_sdk::release_ik();
  ASSERT_TRUE(aubo_v4_test_sdk::wait_for_line_move(500ms));
  EXPECT_FALSE(transport.submit(command(3, 1), seed));
  aubo_v4_test_sdk::release_line_move();
  ASSERT_TRUE(aubo_v4_test_sdk::wait_for_line_move_completed(1, 500ms));
  ASSERT_TRUE(wait_for_motion_result(transport, 500ms).has_value());
}

TEST_F(StepwiseTransportTest, CancelWhileIkBlockedPreventsLineMove)
{
  aubo_v4_test_sdk::state().block_ik = true;
  StepwiseTransport transport;
  ASSERT_TRUE(connect(transport));
  ASSERT_TRUE(transport.submit(command(1, 10), seed));
  ASSERT_TRUE(aubo_v4_test_sdk::wait_for_ik(500ms));
  transport.cancel_and_stop(11);
  ASSERT_TRUE(aubo_v4_test_sdk::wait_for_stop_count(1, 500ms));
  aubo_v4_test_sdk::release_ik();
  ASSERT_TRUE(aubo_v4_test_sdk::wait_for_ik_completed(1, 500ms));
  ASSERT_TRUE(aubo_v4_test_sdk::wait_for_stop_count(2, 500ms));
  EXPECT_EQ(line_move_count(), 0);
}

TEST_F(StepwiseTransportTest, StaleEpochJobCannotStartAfterCancellation)
{
  aubo_v4_test_sdk::state().block_ik = true;
  StepwiseTransport transport;
  ASSERT_TRUE(connect(transport));
  ASSERT_TRUE(transport.submit(command(1, 20), seed));
  ASSERT_TRUE(aubo_v4_test_sdk::wait_for_ik(500ms));
  transport.cancel_and_stop(21);
  aubo_v4_test_sdk::release_ik();
  ASSERT_TRUE(aubo_v4_test_sdk::wait_for_ik_completed(1, 500ms));
  const auto result = wait_for_motion_result(transport, 500ms);
  ASSERT_TRUE(result.has_value());
  EXPECT_FALSE(result->success);
  EXPECT_NE(result->detail.find("cancelled"), std::string::npos);
  EXPECT_EQ(line_move_count(), 0);
}

TEST_F(StepwiseTransportTest, CancelWhileLineMoveBlockedStopsOnlyAfterQuiescence)
{
  aubo_v4_test_sdk::state().block_line_move = true;
  StepwiseTransport transport;
  ASSERT_TRUE(connect(transport));
  ASSERT_TRUE(transport.submit(command(1, 30), seed));
  ASSERT_TRUE(aubo_v4_test_sdk::wait_for_line_move(500ms));
  transport.cancel_and_stop(31);
  ASSERT_TRUE(aubo_v4_test_sdk::wait_for_stop_count(1, 500ms));
  EXPECT_EQ(stop_count(), 1);
  aubo_v4_test_sdk::release_line_move();
  ASSERT_TRUE(aubo_v4_test_sdk::wait_for_line_move_completed(1, 500ms));
  ASSERT_TRUE(aubo_v4_test_sdk::wait_for_stop_count(2, 1000ms));
  const auto order = calls();
  EXPECT_LT(index_of(order, "line_move_enter"), index_of(order, "stop_first"));
  EXPECT_LT(index_of(order, "stop_first_exit"), index_of(order, "line_move_exit"));
  EXPECT_LT(index_of(order, "line_move_exit"), index_of(order, "stop_final"));
}

TEST_F(StepwiseTransportTest, SubmitDuringStopIsRefused)
{
  aubo_v4_test_sdk::state().block_first_stop = true;
  StepwiseTransport transport;
  ASSERT_TRUE(connect(transport));
  ASSERT_TRUE(transport.submit(command(1, 40), seed));
  ASSERT_TRUE(aubo_v4_test_sdk::wait_for_line_move_completed(1, 500ms));
  transport.cancel_and_stop(41);
  ASSERT_TRUE(aubo_v4_test_sdk::wait_for_stop_count(1, 500ms));
  EXPECT_FALSE(transport.submit(command(2, 41), seed));
  aubo_v4_test_sdk::release_first_stop();
  ASSERT_TRUE(aubo_v4_test_sdk::wait_for_stop_count(2, 500ms));
}

TEST_F(StepwiseTransportTest, IkFailurePreventsLineMove)
{
  aubo_v4_test_sdk::state().ik_result = -1;
  StepwiseTransport transport;
  ASSERT_TRUE(connect(transport));
  ASSERT_TRUE(transport.submit(command(1, 50), seed));
  ASSERT_TRUE(aubo_v4_test_sdk::wait_for_ik_completed(1, 500ms));
  ASSERT_TRUE(wait_for_motion_result(transport, 500ms).has_value());
  EXPECT_EQ(line_move_count(), 0);
}

TEST_F(StepwiseTransportTest, JointStepBoundFailurePreventsLineMove)
{
  aubo_v4_test_sdk::state().ik_delta[0] = 0.06;
  StepwiseTransport transport;
  ASSERT_TRUE(connect(transport));
  ASSERT_TRUE(transport.submit(command(1, 60), seed));
  ASSERT_TRUE(aubo_v4_test_sdk::wait_for_ik_completed(1, 500ms));
  ASSERT_TRUE(wait_for_motion_result(transport, 500ms).has_value());
  EXPECT_EQ(line_move_count(), 0);
}

TEST_F(StepwiseTransportTest, InvalidSubmitValuesAreRejectedBeforeSdkIk)
{
  StepwiseTransport transport;
  ASSERT_TRUE(connect(transport));

  auto nan_target = command(1, 1);
  nan_target.target[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(transport.submit(nan_target, seed));

  auto zero_id = command(0, 1);
  EXPECT_FALSE(transport.submit(zero_id, seed));

  auto nan_seed = command(1, 1);
  auto invalid_seed = seed;
  invalid_seed[2] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(transport.submit(nan_seed, invalid_seed));

  auto zero_quaternion = command(1, 1);
  zero_quaternion.orientation = {0.0, 0.0, 0.0, 0.0};
  EXPECT_FALSE(transport.submit(zero_quaternion, seed));

  auto non_unit_quaternion = command(1, 1);
  non_unit_quaternion.orientation = {0.0, 0.0, 0.0, 2.0};
  EXPECT_FALSE(transport.submit(non_unit_quaternion, seed));

  auto oversized_step = command(1, 1);
  oversized_step.start[0] = 0.0993;
  EXPECT_FALSE(transport.submit(oversized_step, seed));

  auto zero_step = command(1, 1);
  zero_step.start = zero_step.target;
  EXPECT_FALSE(transport.submit(zero_step, seed));

  EXPECT_EQ(ik_call_count(), 0);
  EXPECT_EQ(line_move_count(), 0);
}

TEST_F(StepwiseTransportTest, RepeatedIdAfterCompletedMotionCannotResend)
{
  StepwiseTransport transport;
  ASSERT_TRUE(connect(transport));
  ASSERT_TRUE(transport.submit(command(1, 1), seed));
  ASSERT_TRUE(aubo_v4_test_sdk::wait_for_line_move_completed(1, 500ms));
  ASSERT_TRUE(wait_for_motion_result(transport, 500ms).has_value());

  EXPECT_FALSE(transport.submit(command(1, 1), seed));
  EXPECT_EQ(ik_call_count(), 1);
  EXPECT_EQ(line_move_count(), 1);
}

TEST_F(StepwiseTransportTest, StaleEpochSubmitAfterConfirmedStopIsRefused)
{
  StepwiseTransport transport;
  ASSERT_TRUE(connect(transport));
  ASSERT_TRUE(transport.submit(command(1, 1), seed));
  ASSERT_TRUE(aubo_v4_test_sdk::wait_for_line_move_completed(1, 500ms));
  ASSERT_TRUE(wait_for_motion_result(transport, 500ms).has_value());

  transport.cancel_and_stop(5);
  ASSERT_TRUE(aubo_v4_test_sdk::wait_for_stop_count(2, 500ms));
  ASSERT_TRUE(wait_for_stop_result(transport, 500ms).has_value());

  // A current-floor submit proves the stop worker has quiesced before the
  // lower-epoch assertion below is evaluated.
  ASSERT_TRUE(submit_when_available(transport, command(2, 5), 500ms));
  ASSERT_TRUE(aubo_v4_test_sdk::wait_for_line_move_completed(2, 500ms));
  ASSERT_TRUE(wait_for_motion_result(transport, 500ms).has_value());
  EXPECT_FALSE(transport.submit(command(3, 4), seed));
  EXPECT_EQ(line_move_count(), 2);
}

TEST_F(StepwiseTransportTest, LowerEpochCancellationCannotRollbackEpochFloor)
{
  StepwiseTransport transport;
  ASSERT_TRUE(connect(transport));

  transport.cancel_and_stop(20);
  ASSERT_TRUE(aubo_v4_test_sdk::wait_for_stop_count(2, 500ms));
  ASSERT_TRUE(wait_for_stop_result(transport, 500ms).has_value());

  // This older cancellation must retain the floor at 20, not lower it to 5.
  transport.cancel_and_stop(5);
  ASSERT_TRUE(aubo_v4_test_sdk::wait_for_stop_count(4, 500ms));
  ASSERT_TRUE(wait_for_stop_result(transport, 500ms).has_value());

  ASSERT_TRUE(submit_when_available(transport, command(1, 20), 500ms));
  ASSERT_TRUE(aubo_v4_test_sdk::wait_for_line_move_completed(1, 500ms));
  ASSERT_TRUE(wait_for_motion_result(transport, 500ms).has_value());
  EXPECT_FALSE(transport.submit(command(2, 19), seed));
  EXPECT_EQ(line_move_count(), 1);
}

TEST_F(StepwiseTransportTest, StopFailureIsReportedFalse)
{
  aubo_v4_test_sdk::set_stop_results({-1, -1});
  StepwiseTransport transport;
  ASSERT_TRUE(connect(transport));
  ASSERT_TRUE(transport.submit(command(1, 70), seed));
  ASSERT_TRUE(aubo_v4_test_sdk::wait_for_line_move_completed(1, 500ms));
  transport.cancel_and_stop(71);
  ASSERT_TRUE(aubo_v4_test_sdk::wait_for_stop_count(2, 500ms));
  const auto result = wait_for_stop_result(transport, 500ms);
  ASSERT_TRUE(result.has_value());
  EXPECT_FALSE(result->success);
  EXPECT_NE(result->detail.find("-1,-1"), std::string::npos);
}

TEST_F(StepwiseTransportTest, ShutdownAfterDispatchStopsAndLeavesNoOldJob)
{
  {
    StepwiseTransport transport;
    ASSERT_TRUE(connect(transport));
    ASSERT_TRUE(transport.submit(command(1, 80), seed));
    ASSERT_TRUE(aubo_v4_test_sdk::wait_for_line_move_completed(1, 500ms));
  }
  EXPECT_EQ(line_move_count(), 1);
  EXPECT_EQ(stop_count(), 3);
  const auto order = calls();
  EXPECT_LT(index_of(order, "line_move_exit"), index_of(order, "stop_first"));
  EXPECT_LT(index_of(order, "stop_final_exit"), index_of(order, "destruct"));
}

TEST_F(StepwiseTransportTest, InvalidPoseCannotBecomeHealthyFromDiagnosisAlone)
{
  StepwiseTransport transport;
  ASSERT_TRUE(connect(transport));
  ASSERT_TRUE(wait_for_diagnosis(transport, 500ms));
  auto valid = valid_waypoint();
  aubo_v4_test_sdk::emit_waypoint(valid);
  EXPECT_TRUE(transport.sample().feedback.healthy);

  auto invalid = valid;
  invalid.orientation = {};
  aubo_v4_test_sdk::emit_waypoint(invalid);
  const auto sample = transport.sample();
  EXPECT_TRUE(sample.diagnosis_ok);
  EXPECT_FALSE(sample.feedback.healthy);
  EXPECT_NE(sample.error.find("zero-length orientation"), std::string::npos);
}
}  // namespace
