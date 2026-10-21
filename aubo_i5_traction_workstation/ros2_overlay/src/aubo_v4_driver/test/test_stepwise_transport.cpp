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
    const auto sample = transport.sample();
    if (sample.diagnosis_ok && sample.state_result == 0) {return true;}
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
      deadline - std::chrono::steady_clock::now());
    aubo_v4_test_sdk::wait_for_event(seen, std::min(remaining, 50ms));
  }
  const auto sample = transport.sample();
  return sample.diagnosis_ok && sample.state_result == 0;
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

TEST_F(StepwiseTransportTest, ExperimentalTimedBatchHasNoFollowFallbackAndLeavesOnStop)
{
  StepwiseTransport transport;
  auto settings = options();
  settings.allow_follow_mode = settings.fixed_follow_profile = true;
  settings.experimental_tcp_can_drag = true;
  {
    auto & state = aubo_v4_test_sdk::state();
    std::lock_guard<std::mutex> lock(state.mutex);
    state.tcp_linear_test_kinematics = true;
  }
  std::string error;
  ASSERT_TRUE(transport.connect(settings, error)) << error;
  auto measured = valid_waypoint();
  measured.cartPos.position.x = 0.0998; // Match the explicitly selected algebra FK stub.
  aubo_v4_test_sdk::emit_waypoint(measured);
  std::size_t seen = 0;
  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (!transport.sample().feedback.healthy && std::chrono::steady_clock::now() < deadline) {
    aubo_v4_test_sdk::wait_for_event(seen, 10ms);
  }
  ASSERT_TRUE(transport.sample().feedback.healthy);
  ASSERT_TRUE(transport.update_drag_intent({0.001, 0, 0}, 1, aubo_v4_driver::steady_seconds()));
  auto value = command(1, 1);
  value.assisted_drag = value.follow_joint_mode = value.cartesian_drag_reference = true;
  ASSERT_TRUE(transport.submit(value, seed));
  const auto sent = wait_for_motion_result(transport, 1s);
  ASSERT_TRUE(sent);
  ASSERT_TRUE(sent->success) << sent->detail;
  EXPECT_TRUE(transport.sample().tcp_can_active);
  EXPECT_EQ(transport.sample().tcp_can_points, 10U);
  {
    auto & state = aubo_v4_test_sdk::state();
    std::lock_guard<std::mutex> lock(state.mutex);
    EXPECT_EQ(state.tcp_enter_count, 1);
    EXPECT_EQ(state.tcp_batch_count, 1);
    EXPECT_EQ(state.follow_move_count, 0);
    EXPECT_EQ(state.line_move_count, 0);
  }
  // With no new scheduler job, a zero intent must still produce the final
  // deceleration batch. A regular job arriving during that feed is admitted to
  // the single mailbox, not misreported as a failed/busy SDK motion.
  ASSERT_TRUE(transport.update_drag_intent({}, 1, aubo_v4_driver::steady_seconds()));
  {
    auto & state = aubo_v4_test_sdk::state();
    std::lock_guard<std::mutex> lock(state.mutex);
    state.block_ik = true;
    state.release_ik = state.ik_entered = false;
    state.diagnosis.macTargetPosDataSize = 24;
  }
  {
    auto & state = aubo_v4_test_sdk::state();
    std::unique_lock<std::mutex> lock(state.mutex);
    ASSERT_TRUE(state.changed.wait_for(lock, 1s, [&]() {return state.ik_entered;}));
  }
  value.id = 2;
  ASSERT_TRUE(transport.submit(value, seed));
  EXPECT_FALSE(transport.submit(value, seed));
  {
    auto & state = aubo_v4_test_sdk::state();
    std::lock_guard<std::mutex> lock(state.mutex);
    state.release_ik = true;
    state.changed.notify_all();
  }
  const auto queued = wait_for_motion_result(transport, 1s);
  ASSERT_TRUE(queued);
  EXPECT_EQ(queued->id, 2U);
  ASSERT_TRUE(queued->success) << queued->detail;
  EXPECT_EQ(transport.sample().tcp_can_batches, 2U);
  EXPECT_EQ(transport.sample().tcp_can_points, 16U);
  EXPECT_EQ(transport.sample().motion_ack_count, 2U);
  transport.cancel_and_stop(2);
  const auto stopped = wait_for_stop_result(transport, 1s);
  ASSERT_TRUE(stopped);
  ASSERT_TRUE(stopped->success) << stopped->detail;
  EXPECT_FALSE(transport.sample().tcp_can_active);
  ASSERT_TRUE(submit_when_available(transport, command(3, 2), 500ms));
  const auto ordinary = wait_for_motion_result(transport, 1s);
  ASSERT_TRUE(ordinary);
  ASSERT_TRUE(ordinary->success);
  auto & state = aubo_v4_test_sdk::state();
  std::lock_guard<std::mutex> lock(state.mutex);
  EXPECT_EQ(state.tcp_leave_count, 1);
  EXPECT_EQ(state.line_move_count, 1);
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

TEST_F(StepwiseTransportTest, RecordedPointOneRadProposalIsResolvedByShorterCartesianIk)
{
  StepwiseTransport transport;
  auto settings = options();
  settings.allow_follow_mode = true;
  settings.fixed_follow_profile = true;
  settings.adaptive_follow_ik = true;
  std::string error;
  ASSERT_TRUE(transport.connect(settings, error));
  {
    auto & state = aubo_v4_test_sdk::state();
    std::lock_guard<std::mutex> lock(state.mutex);
    state.ik_delta[5] = 0.100573;
    state.ik_scale_with_position = true;
  }
  auto value = command(1, 1);
  value.follow_joint_mode = true;
  ASSERT_TRUE(transport.submit(value, seed));
  const auto result = wait_for_motion_result(transport, 1s);
  ASSERT_TRUE(result);
  ASSERT_TRUE(result->success) << result->detail;
  const auto sample = transport.sample();
  EXPECT_NEAR(sample.proposed_ik_max_joint_delta_rad, 0.100573, 1e-10);
  EXPECT_EQ(sample.adaptive_ik_attempts, 2);
  EXPECT_DOUBLE_EQ(sample.accepted_ik_scale, 0.25);
  EXPECT_LT(sample.last_ik_max_joint_delta_rad, 0.05);
  EXPECT_EQ(sample.fk_result, 0);
  {
    auto & state = aubo_v4_test_sdk::state();
    std::lock_guard<std::mutex> lock(state.mutex);
    EXPECT_NEAR(state.last_follow_target[5], 0.100573 / 4, 1e-10);
  }
}

TEST_F(StepwiseTransportTest, AdaptiveIkDoesNotHideUnchangingBranchOrFkFailure)
{
  StepwiseTransport transport;
  auto settings = options();
  settings.allow_follow_mode = true;
  settings.fixed_follow_profile = true;
  settings.adaptive_follow_ik = true;
  std::string error;
  ASSERT_TRUE(transport.connect(settings, error));
  {
    auto & state = aubo_v4_test_sdk::state();
    std::lock_guard<std::mutex> lock(state.mutex);
    state.ik_delta[5] = 0.100573; // Does not shrink with Cartesian subgoals.
  }
  auto value = command(1, 1);
  value.follow_joint_mode = true;
  ASSERT_TRUE(transport.submit(value, seed));
  const auto result = wait_for_motion_result(transport, 1s);
  ASSERT_TRUE(result);
  EXPECT_FALSE(result->success);
  EXPECT_EQ(transport.sample().adaptive_ik_attempts, 8);
  {
    auto & state = aubo_v4_test_sdk::state();
    std::lock_guard<std::mutex> lock(state.mutex);
    EXPECT_EQ(state.follow_move_count, 0);
    state.fk_result = 37;
  }
  value.id = 2;
  ASSERT_TRUE(submit_when_available(transport, value, 500ms));
  const auto failed_fk = wait_for_motion_result(transport, 1s);
  ASSERT_TRUE(failed_fk);
  EXPECT_FALSE(failed_fk->success);
  EXPECT_EQ(failed_fk->detail, "adaptive IK anchor FK rejected");
}

TEST_F(StepwiseTransportTest, LongerDragLeadCannotLeakIntoForceOrOrdinaryLineMove)
{
  StepwiseTransport transport;
  auto settings = options();
  settings.allow_follow_mode = true;
  settings.fixed_follow_profile = true;
  settings.maximum_drag_step_m = 0.020;
  std::string error;
  ASSERT_TRUE(transport.connect(settings, error));
  auto value = command(1, 1);
  value.target[0] = value.start[0] + 0.015;
  value.follow_joint_mode = true;
  EXPECT_FALSE(transport.submit(value, seed));
  value.assisted_drag = true;
  value.follow_joint_mode = false;
  EXPECT_FALSE(transport.submit(value, seed));
  value.follow_joint_mode = true;
  ASSERT_TRUE(transport.submit(value, seed));
  const auto result = wait_for_motion_result(transport, 1s);
  ASSERT_TRUE(result);
  EXPECT_TRUE(result->success);
}

TEST_F(StepwiseTransportTest, AdaptiveKinematicsMustMatchMeasuredPoseBeforeHealthy)
{
  StepwiseTransport transport;
  auto settings = options();
  settings.adaptive_follow_ik = true;
  std::string error;
  ASSERT_TRUE(transport.connect(settings, error));
  auto point = valid_waypoint();
  point.cartPos.position.z = 0.9;
  aubo_v4_test_sdk::emit_waypoint(point);
  std::size_t seen = 0;
  const auto bad_deadline = std::chrono::steady_clock::now() + 1s;
  while (transport.sample().fk_position_error_m < 0.1 &&
    std::chrono::steady_clock::now() < bad_deadline)
  {
    aubo_v4_test_sdk::wait_for_event(seen, 20ms);
  }
  EXPECT_FALSE(transport.sample().kinematics_verified);
  EXPECT_FALSE(transport.sample().feedback.healthy);
  aubo_v4_test_sdk::emit_waypoint(valid_waypoint());
  const auto good_deadline = std::chrono::steady_clock::now() + 1s;
  while (!transport.sample().kinematics_verified &&
    std::chrono::steady_clock::now() < good_deadline)
  {
    aubo_v4_test_sdk::wait_for_event(seen, 20ms);
  }
  EXPECT_TRUE(transport.sample().kinematics_verified);
  EXPECT_LE(transport.sample().fk_position_error_m, 0.001);
  EXPECT_TRUE(transport.sample().feedback.healthy);
  EXPECT_EQ(line_move_count(), 0);
}

TEST_F(StepwiseTransportTest, RecordedSlightFollowOversizeIsSubdividedWithoutRaisingJointBound)
{
  StepwiseTransport transport;
  auto settings = options();
  settings.allow_follow_mode = true;
  settings.fixed_follow_profile = true;
  settings.subdivide_follow_joint_steps = true;
  std::string error;
  ASSERT_TRUE(transport.connect(settings, error));
  {
    auto & state = aubo_v4_test_sdk::state();
    std::lock_guard<std::mutex> lock(state.mutex);
    state.ik_delta[5] = 0.050853;
  }
  auto target = command(1, 1);
  target.follow_joint_mode = true;
  ASSERT_TRUE(transport.submit(target, seed));
  const auto result = wait_for_motion_result(transport, 1s);
  ASSERT_TRUE(result);
  ASSERT_TRUE(result->success);
  EXPECT_NEAR(transport.sample().last_ik_max_joint_delta_rad, 0.045, 1e-12);
  EXPECT_DOUBLE_EQ(transport.sample().proposed_ik_max_joint_delta_rad, 0.050853);
  EXPECT_EQ(transport.sample().follow_joint_subdivisions, 1U);
  {
    auto & state = aubo_v4_test_sdk::state();
    std::lock_guard<std::mutex> lock(state.mutex);
    EXPECT_NEAR(state.last_follow_target[5], 0.045, 1e-12);
    state.ik_delta[5] = 0.20;
  }
  target.id = 2;
  ASSERT_TRUE(submit_when_available(transport, target, 500ms));
  const auto rejected = wait_for_motion_result(transport, 1s);
  ASSERT_TRUE(rejected);
  EXPECT_FALSE(rejected->success);
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

TEST_F(StepwiseTransportTest, FollowCachesProfilesAndAcceptsMeasuredHold)
{
  StepwiseTransport transport;
  auto settings = options();
  settings.allow_follow_mode = true;
  settings.speed_mps = 0.05;
  settings.maximum_step_m = 0.005;
  settings.fast_acceleration_mps2 = 0.20;
  settings.follow_joint_speed_radps = 0.8;
  settings.follow_joint_acceleration_radps2 = 2.0;
  std::string error;
  ASSERT_TRUE(transport.connect(settings, error));
  auto value = command(1, 1);
  value.follow_joint_mode = true;
  for (unsigned id = 1; id <= 2; ++id) {
    value.id = id;
    ASSERT_TRUE(transport.submit(value, seed));
    const auto result = wait_for_motion_result(transport, 1s);
    ASSERT_TRUE(result);
    ASSERT_TRUE(result->success);
  }
  EXPECT_EQ(transport.sample().follow_profile_updates, 1U);
  value.id = 3;
  value.hold_position = true;
  value.start = value.target;
  const std::array<double, 6> measured_joints{0.12, -0.11, 0.18, 0.03, -0.02, 0.01};
  auto latest_pose = valid_waypoint();
  std::copy(measured_joints.begin(), measured_joints.end(), latest_pose.jointpos);
  latest_pose.jointpos[0] += 0.001;
  aubo_v4_test_sdk::emit_waypoint(latest_pose);
  const int ik_before_hold = ik_call_count();
  ASSERT_TRUE(transport.submit(value, measured_joints));
  const auto held = wait_for_motion_result(transport, 1s);
  ASSERT_TRUE(held);
  EXPECT_TRUE(held->success);
  EXPECT_EQ(ik_call_count(), ik_before_hold);
  {
    auto & state = aubo_v4_test_sdk::state();
    std::lock_guard<std::mutex> lock(state.mutex);
    EXPECT_DOUBLE_EQ(state.last_follow_target[0], measured_joints[0] + 0.001);
  }
  EXPECT_EQ(transport.sample().follow_profile_updates, 1U);
  EXPECT_EQ(stop_count(), 0);
}

TEST_F(StepwiseTransportTest, IndependentDragRefreshesOldPoseAndReversedIntentAfterDelayedIk)
{
  StepwiseTransport transport;
  auto settings = options();
  settings.allow_follow_mode = settings.fixed_follow_profile = true;
  settings.speed_mps = 0.05;
  std::string error;
  ASSERT_TRUE(transport.connect(settings, error));
  auto point = valid_waypoint();
  aubo_v4_test_sdk::emit_waypoint(point);
  ASSERT_TRUE(transport.update_drag_intent({0.01, 0, 0}, 1, aubo_v4_driver::steady_seconds()));
  {
    auto & state = aubo_v4_test_sdk::state();
    std::lock_guard<std::mutex> lock(state.mutex);
    state.block_ik = true;
  }
  auto value = command(1, 1);
  value.follow_joint_mode = value.assisted_drag = value.cartesian_drag_reference = true;
  ASSERT_TRUE(transport.submit(value, seed));
  {
    auto & state = aubo_v4_test_sdk::state();
    std::unique_lock<std::mutex> lock(state.mutex);
    ASSERT_TRUE(state.changed.wait_for(lock, 1s, [&]() {return state.ik_entered;}));
  }
  point.cartPos.position.x += 0.002;
  point.cartPos.position.z -= 0.001;
  aubo_v4_test_sdk::emit_waypoint(point);
  ASSERT_TRUE(transport.update_drag_intent({0, -0.02, 0}, 1, aubo_v4_driver::steady_seconds()));
  {
    auto & state = aubo_v4_test_sdk::state();
    std::lock_guard<std::mutex> lock(state.mutex);
    state.release_ik = true;
    state.changed.notify_all();
  }
  const auto result = wait_for_motion_result(transport, 1s);
  ASSERT_TRUE(result);
  ASSERT_TRUE(result->success) << result->detail;
  auto & state = aubo_v4_test_sdk::state();
  std::lock_guard<std::mutex> lock(state.mutex);
  EXPECT_EQ(state.ik_call_count, 2);
  EXPECT_DOUBLE_EQ(state.last_ik_position.x, point.cartPos.position.x);
  EXPECT_NEAR(state.last_ik_position.y, point.cartPos.position.y - 0.02 * 0.06, 1e-12);
  EXPECT_DOUBLE_EQ(state.last_ik_position.z, point.cartPos.position.z);
  EXPECT_EQ(transport.sample().drag_intent_refreshes, 1U);
}

TEST_F(StepwiseTransportTest, IndependentDragWithoutFreshEpochIntentHoldsInsteadOfReplayingTarget)
{
  StepwiseTransport transport;
  auto settings = options();
  settings.allow_follow_mode = settings.fixed_follow_profile = true;
  std::string error;
  ASSERT_TRUE(transport.connect(settings, error));
  auto point = valid_waypoint();
  aubo_v4_test_sdk::emit_waypoint(point);
  ASSERT_TRUE(transport.update_drag_intent({0.01, 0, 0}, 99, aubo_v4_driver::steady_seconds()));
  auto value = command(1, 1);
  value.follow_joint_mode = value.assisted_drag = value.cartesian_drag_reference = true;
  ASSERT_TRUE(transport.submit(value, seed));
  const auto result = wait_for_motion_result(transport, 1s);
  ASSERT_TRUE(result);
  ASSERT_TRUE(result->success) << result->detail;
  auto & state = aubo_v4_test_sdk::state();
  std::lock_guard<std::mutex> lock(state.mutex);
  EXPECT_DOUBLE_EQ(state.last_ik_position.x, point.cartPos.position.x);
  EXPECT_DOUBLE_EQ(state.last_ik_position.y, point.cartPos.position.y);
  EXPECT_DOUBLE_EQ(state.last_ik_position.z, point.cartPos.position.z);
}

TEST_F(StepwiseTransportTest, ConstantIntentHeartbeatStillReanchorsPoseAfterDelayedIk)
{
  StepwiseTransport transport;
  auto settings = options();
  settings.allow_follow_mode = settings.fixed_follow_profile = true;
  std::string error;
  ASSERT_TRUE(transport.connect(settings, error));
  auto point = valid_waypoint();
  aubo_v4_test_sdk::emit_waypoint(point);
  ASSERT_TRUE(transport.update_drag_intent({0.001, 0, 0}, 1, aubo_v4_driver::steady_seconds()));
  {
    auto & state = aubo_v4_test_sdk::state();
    std::lock_guard<std::mutex> lock(state.mutex);
    state.block_ik = true;
  }
  auto value = command(1, 1);
  value.follow_joint_mode = value.assisted_drag = value.cartesian_drag_reference = true;
  ASSERT_TRUE(transport.submit(value, seed));
  {
    auto & state = aubo_v4_test_sdk::state();
    std::unique_lock<std::mutex> lock(state.mutex);
    ASSERT_TRUE(state.changed.wait_for(lock, 1s, [&]() {return state.ik_entered;}));
  }
  point.cartPos.position.x += 0.001;
  aubo_v4_test_sdk::emit_waypoint(point);
  ASSERT_TRUE(transport.update_drag_intent({0.001, 0, 0}, 1, aubo_v4_driver::steady_seconds()));
  {
    auto & state = aubo_v4_test_sdk::state();
    std::lock_guard<std::mutex> lock(state.mutex);
    state.release_ik = true;
    state.changed.notify_all();
  }
  const auto result = wait_for_motion_result(transport, 1s);
  ASSERT_TRUE(result);
  ASSERT_TRUE(result->success) << result->detail;
  auto & state = aubo_v4_test_sdk::state();
  std::lock_guard<std::mutex> lock(state.mutex);
  EXPECT_EQ(state.ik_call_count, 2);
  EXPECT_NEAR(state.last_ik_position.x, point.cartPos.position.x + 0.001 * 0.06, 1e-12);
}

TEST_F(StepwiseTransportTest, ReleaseDuringProfileUpdateDiscardsUnsentMoveWithoutFalseAck)
{
  StepwiseTransport transport;
  auto settings = options();
  settings.allow_follow_mode = settings.fixed_follow_profile = true;
  settings.drag_release_prediction_s = 0.04;
  std::string error;
  ASSERT_TRUE(transport.connect(settings, error));
  aubo_v4_test_sdk::emit_waypoint(valid_waypoint());
  auto value = command(1, 1);
  value.follow_joint_mode = value.assisted_drag = value.cartesian_drag_reference = true;
  value.hold_position = true;
  value.start = value.target;
  ASSERT_TRUE(transport.submit(value, seed));
  const auto held = wait_for_motion_result(transport, 1s);
  ASSERT_TRUE(held);
  ASSERT_TRUE(held->success);
  {
    auto & state = aubo_v4_test_sdk::state();
    std::lock_guard<std::mutex> lock(state.mutex);
    state.block_profile = true;
    state.profile_entered = false;
  }
  value = command(2, 1);
  value.follow_joint_mode = value.assisted_drag = value.cartesian_drag_reference = true;
  ASSERT_TRUE(transport.update_drag_intent({0.001, 0, 0}, 1, aubo_v4_driver::steady_seconds()));
  ASSERT_TRUE(transport.submit(value, seed));
  {
    auto & state = aubo_v4_test_sdk::state();
    std::unique_lock<std::mutex> lock(state.mutex);
    ASSERT_TRUE(state.changed.wait_for(lock, 1s, [&]() {return state.profile_entered;}));
  }
  ASSERT_TRUE(transport.update_drag_intent({}, 1, aubo_v4_driver::steady_seconds()));
  {
    auto & state = aubo_v4_test_sdk::state();
    std::lock_guard<std::mutex> lock(state.mutex);
    state.release_profile = true;
    state.changed.notify_all();
  }
  const auto result = wait_for_motion_result(transport, 1s);
  ASSERT_TRUE(result);
  EXPECT_TRUE(result->superseded);
  EXPECT_FALSE(result->success);
  EXPECT_EQ(transport.sample().motion_ack_count, 1U);
  auto & state = aubo_v4_test_sdk::state();
  std::lock_guard<std::mutex> lock(state.mutex);
  EXPECT_EQ(state.follow_move_count, 1);
}

TEST_F(StepwiseTransportTest, IndependentDragReleaseSolvesCartesianEndpointWithMeasuredOrientation)
{
  StepwiseTransport transport;
  auto settings = options();
  settings.allow_follow_mode = settings.fixed_follow_profile = true;
  settings.drag_release_prediction_s = 0.04;
  settings.fast_acceleration_mps2 = 0.20;
  std::string error;
  ASSERT_TRUE(transport.connect(settings, error));
  auto point = valid_waypoint();
  aubo_v4_test_sdk::emit_waypoint(point);
  for (unsigned i = 1; i <= 4; ++i) {
    std::this_thread::sleep_for(20ms);
    point.cartPos.position.x += 0.001;
    aubo_v4_test_sdk::emit_waypoint(point);
  }
  auto value = command(1, 1);
  value.start = value.target;
  value.follow_joint_mode = value.assisted_drag = value.hold_position = true;
  value.cartesian_drag_reference = true;
  ASSERT_TRUE(transport.submit(value, seed));
  const auto result = wait_for_motion_result(transport, 1s);
  ASSERT_TRUE(result);
  ASSERT_TRUE(result->success) << result->detail;
  auto & state = aubo_v4_test_sdk::state();
  std::lock_guard<std::mutex> lock(state.mutex);
  EXPECT_EQ(state.ik_call_count, 1);
  EXPECT_GT(state.last_ik_position.x, point.cartPos.position.x);
  EXPECT_LT(state.last_ik_position.x - point.cartPos.position.x, 0.003);
  EXPECT_DOUBLE_EQ(state.last_ik_position.y, point.cartPos.position.y);
  EXPECT_DOUBLE_EQ(state.last_ik_position.z, point.cartPos.position.z);
  EXPECT_DOUBLE_EQ(state.last_ik_orientation.w, point.orientation.w);
  EXPECT_DOUBLE_EQ(state.last_ik_orientation.x, point.orientation.x);
  EXPECT_EQ(state.last_follow_target, seed);
}

TEST_F(StepwiseTransportTest, DragReleaseTargetsBoundedDecelerationAheadAndDisablesBlending)
{
  for (double direction : {-1.0, 1.0}) {
    StepwiseTransport transport;
    auto settings = options();
    settings.allow_follow_mode = true;
    settings.fixed_follow_profile = true;
    settings.drag_release_prediction_s = 0.04;
    const int stops_before = stop_count();
    std::string error;
    ASSERT_TRUE(transport.connect(settings, error));
    auto point = valid_waypoint();
    aubo_v4_test_sdk::emit_waypoint(point);
    for (unsigned i = 1; i <= 4; ++i) {
      std::this_thread::sleep_for(20ms);
      point.jointpos[0] = direction * 0.001 * i;
      aubo_v4_test_sdk::emit_waypoint(point);
    }
    auto value = command(1, 1);
    value.start = value.target;
    value.follow_joint_mode = true;
    value.assisted_drag = true;
    value.hold_position = true;
    std::array<double, 6> current{};
    current[0] = point.jointpos[0];
    ASSERT_TRUE(transport.submit(value, current));
    const auto held = wait_for_motion_result(transport, 1s);
    ASSERT_TRUE(held);
    EXPECT_TRUE(held->success) << held->detail;
    {
      auto & state = aubo_v4_test_sdk::state();
      std::lock_guard<std::mutex> lock(state.mutex);
      const double delta = state.last_follow_target[0] - current[0];
      EXPECT_GT(direction * delta, 0.0);
      EXPECT_LE(std::abs(delta), 0.2 * settings.maximum_joint_step_rad);
      EXPECT_GT(state.no_arrival_ahead_count, 0);
    }
    EXPECT_EQ(stop_count(), stops_before);
    // Settled feedback must cancel any remaining velocity-estimator tail.
    std::this_thread::sleep_for(20ms);
    aubo_v4_test_sdk::emit_waypoint(point);
    value.id = 2;
    ASSERT_TRUE(transport.submit(value, current));
    const auto quiet_hold = wait_for_motion_result(transport, 1s);
    ASSERT_TRUE(quiet_hold);
    EXPECT_TRUE(quiet_hold->success);
    EXPECT_DOUBLE_EQ(transport.sample().drag_release_predicted_joint_step_rad, 0.0);
    {
      auto & state = aubo_v4_test_sdk::state();
      std::lock_guard<std::mutex> lock(state.mutex);
      EXPECT_DOUBLE_EQ(state.last_follow_target[0], current[0]);
    }
  }
}

TEST_F(StepwiseTransportTest, IdleSenderStopsOnceWithoutTreatingSecondStopErrorAsSuccess)
{
  StepwiseTransport transport;
  auto settings = options();
  settings.single_stop_when_quiescent = true;
  aubo_v4_test_sdk::set_stop_results({0, 10007});
  std::string error;
  ASSERT_TRUE(transport.connect(settings, error));
  transport.cancel_and_stop(1);
  const auto stopped = wait_for_stop_result(transport, 1s);
  ASSERT_TRUE(stopped);
  EXPECT_TRUE(stopped->success);
  EXPECT_EQ(stop_count(), 1);
  EXPECT_EQ(stopped->detail, "quiescent stop SDK result 0");
}

TEST_F(StepwiseTransportTest, SingleQuiescentStopFailureRemainsAFailure)
{
  StepwiseTransport transport;
  auto settings = options();
  settings.single_stop_when_quiescent = true;
  aubo_v4_test_sdk::set_stop_results({10007, 10007});
  std::string error;
  ASSERT_TRUE(transport.connect(settings, error));
  transport.cancel_and_stop(1);
  const auto stopped = wait_for_stop_result(transport, 1s);
  ASSERT_TRUE(stopped);
  EXPECT_FALSE(stopped->success);
  EXPECT_EQ(stop_count(), 1);
}

TEST_F(StepwiseTransportTest, QuiescentStopTimeoutRequiresFreshMeasuredStop)
{
  StepwiseTransport transport;
  auto settings = options();
  settings.single_stop_when_quiescent = true;
  aubo_v4_test_sdk::set_stop_results({10007, 0});
  std::string error;
  ASSERT_TRUE(transport.connect(settings, error));
  transport.cancel_and_stop(1);
  ASSERT_TRUE(aubo_v4_test_sdk::wait_for_stop_count(1, 500ms));
  std::thread stream([]() {
      for (unsigned i = 0; i < 40; ++i) {
        aubo_v4_test_sdk::emit_waypoint(valid_waypoint());
        std::this_thread::sleep_for(10ms);
      }
    });
  const auto stopped = wait_for_stop_result(transport, 1s);
  stream.join();
  ASSERT_TRUE(stopped);
  EXPECT_TRUE(stopped->success);
  EXPECT_EQ(stop_count(), 1);
  EXPECT_NE(stopped->detail.find("verified by fresh post-request telemetry"), std::string::npos);
}

TEST_F(StepwiseTransportTest, MotionResponseErrorsAreClassifiedWithoutResendingOldMotion)
{
  for (const int code : {10007, 10023, 21005, 21006}) {
    aubo_v4_test_sdk::reset();
    aubo_v4_test_sdk::state().line_move_result = code;
    StepwiseTransport transport;
    auto settings = options();
    settings.allow_follow_mode = true;
    std::string error;
    ASSERT_TRUE(transport.connect(settings, error));
    auto target = command(1, 1);
    target.follow_joint_mode = true;
    ASSERT_TRUE(transport.submit(target, seed));
    const auto result = wait_for_motion_result(transport, 1s);
    ASSERT_TRUE(result);
    EXPECT_FALSE(result->success);
    EXPECT_EQ(result->recoverable, code != 21006);
    auto & state = aubo_v4_test_sdk::state();
    std::lock_guard<std::mutex> lock(state.mutex);
    EXPECT_EQ(state.follow_move_count, 1);
  }
}

TEST_F(StepwiseTransportTest, StopTimeoutCannotPassOnFreshPoseIfControllerStillReportsRunning)
{
  aubo_v4_test_sdk::state().robot_state = aubo_robot_namespace::RobotRunning;
  aubo_v4_test_sdk::set_stop_results({10007});
  StepwiseTransport transport;
  auto settings = options();
  settings.single_stop_when_quiescent = true;
  std::string error;
  ASSERT_TRUE(transport.connect(settings, error));
  transport.cancel_and_stop(1);
  ASSERT_TRUE(aubo_v4_test_sdk::wait_for_stop_count(1, 500ms));
  std::thread stream([]() {
      for (unsigned i = 0; i < 80; ++i) {
        aubo_v4_test_sdk::emit_waypoint(valid_waypoint());
        std::this_thread::sleep_for(10ms);
      }
    });
  const auto stopped = wait_for_stop_result(transport, 1500ms);
  stream.join();
  ASSERT_TRUE(stopped);
  EXPECT_FALSE(stopped->success);
}

TEST_F(StepwiseTransportTest, ControllerLimitEventSurvivesLaterNormalNotification)
{
  StepwiseTransport transport;
  ASSERT_TRUE(connect(transport));
  RobotEventCallback callback;
  void * context;
  {
    auto & state = aubo_v4_test_sdk::state();
    std::lock_guard<std::mutex> lock(state.mutex);
    callback = state.event_callback;
    context = state.event_context;
  }
  ASSERT_NE(callback, nullptr);
  aubo_robot_namespace::RobotEventInfo event;
  event.eventType = aubo_robot_namespace::RobotEventJointOutRange;
  event.eventCode = 21006;
  event.eventContent = "joint out of range";
  callback(&event, context);
  event.eventType = aubo_robot_namespace::RobotEventMoveEnterStopState;
  event.eventContent = "stopping";
  callback(&event, context);
  const auto sample = transport.sample();
  EXPECT_EQ(sample.controller_event_type, 1300);
  EXPECT_EQ(sample.controller_error_type, 1006);
  EXPECT_EQ(sample.controller_error_sequence, 1U);
}

TEST_F(StepwiseTransportTest, FinalQuiescentStopAckResolvesOnlyInitialRequestTimeout)
{
  for (const auto & codes : std::vector<std::array<int, 2>>{
      {10007, 0}, {0, 10007}, {10007, 10007}, {10001, 0}, {0, 0}})
  {
    aubo_v4_test_sdk::reset();
    aubo_v4_test_sdk::set_stop_results({codes[0], codes[1]});
    aubo_v4_test_sdk::state().block_line_move = true;
    StepwiseTransport transport;
    ASSERT_TRUE(connect(transport));
    ASSERT_TRUE(transport.submit(command(1, 1), seed));
    ASSERT_TRUE(aubo_v4_test_sdk::wait_for_line_move(500ms));
    transport.cancel_and_stop(2);
    ASSERT_TRUE(aubo_v4_test_sdk::wait_for_stop_count(1, 500ms));
    EXPECT_FALSE(transport.take_result());
    aubo_v4_test_sdk::release_line_move();
    const auto stopped = wait_for_stop_result(transport, 1s);
    ASSERT_TRUE(stopped);
    EXPECT_EQ(stopped->success, codes[1] == 0 && (codes[0] == 0 || codes[0] == 10007));
    EXPECT_EQ(stop_count(), 2);
    const auto order = calls();
    EXPECT_LT(index_of(order, "line_move_exit"), index_of(order, "stop_final"));
    EXPECT_FALSE(transport.submit(command(2, 1), seed));
  }
}

TEST_F(StepwiseTransportTest, ReleaseBrakesWithoutSendingAnyOldJointTarget)
{
  StepwiseTransport transport;
  auto settings = options();
  settings.allow_follow_mode = true;
  settings.fixed_follow_profile = true;
  settings.brake_on_hold = true;
  std::string error;
  ASSERT_TRUE(transport.connect(settings, error));
  auto target = command(1, 1);
  target.follow_joint_mode = true;
  ASSERT_TRUE(transport.submit(target, seed));
  const auto moved = wait_for_motion_result(transport, 1s);
  ASSERT_TRUE(moved);
  ASSERT_TRUE(moved->success);
  target.id = 2;
  target.hold_position = true;
  target.start = target.target;
  ASSERT_TRUE(submit_when_available(transport, target, 500ms));
  const auto held = wait_for_motion_result(transport, 1s);
  ASSERT_TRUE(held);
  ASSERT_TRUE(held->success);
  EXPECT_EQ(held->detail, "hold brake SDK result 0");
  EXPECT_EQ(stop_count(), 1);
  {
    auto & state = aubo_v4_test_sdk::state();
    std::lock_guard<std::mutex> lock(state.mutex);
    EXPECT_EQ(state.follow_move_count, 1);
  }
  target.id = 3;
  target.hold_position = false;
  target.start[0] -= 0.0002;
  ASSERT_TRUE(submit_when_available(transport, target, 500ms));
  const auto resumed = wait_for_motion_result(transport, 1s);
  ASSERT_TRUE(resumed);
  EXPECT_TRUE(resumed->success);
  EXPECT_EQ(transport.sample().follow_profile_updates, 2U);
}

TEST_F(StepwiseTransportTest, HoldTimeoutRetriesOnlyTheBrakeAndRequiresSuccessfulAck)
{
  for (const int final_code : {0, 10007, 10001}) {
    aubo_v4_test_sdk::reset();
    StepwiseTransport transport;
    auto settings = options();
    settings.allow_follow_mode = true;
    settings.fixed_follow_profile = true;
    settings.brake_on_hold = true;
    aubo_v4_test_sdk::set_stop_results({10007, final_code});
    std::string error;
    ASSERT_TRUE(transport.connect(settings, error));
    auto target = command(1, 1);
    target.follow_joint_mode = true;
    ASSERT_TRUE(transport.submit(target, seed));
    const auto moved = wait_for_motion_result(transport, 1s);
    ASSERT_TRUE(moved && moved->success);
    const int before = stop_count();
    target.id = 2;
    target.hold_position = true;
    target.start = target.target;
    ASSERT_TRUE(submit_when_available(transport, target, 500ms));
    const auto held = wait_for_motion_result(transport, 1s);
    ASSERT_TRUE(held);
    EXPECT_EQ(held->success, final_code == 0);
    EXPECT_EQ(stop_count() - before, 2);
    EXPECT_NE(held->detail.find("after initial timeout 10007"), std::string::npos);
    EXPECT_EQ(line_move_count(), 0);
    auto & state = aubo_v4_test_sdk::state();
    std::lock_guard<std::mutex> lock(state.mutex);
    EXPECT_EQ(state.follow_move_count, 1);
  }
}

TEST_F(StepwiseTransportTest, SlowStateGetterDoesNotHideCompletedDiagnosis)
{
  auto & state = aubo_v4_test_sdk::state();
  {
    std::lock_guard<std::mutex> lock(state.mutex);
    state.block_state = true;
  }
  StepwiseTransport transport;
  ASSERT_TRUE(connect(transport));
  bool entered;
  {
    std::unique_lock<std::mutex> lock(state.mutex);
    entered = state.changed.wait_for(lock, 1s, [&state]() {return state.state_entered;});
  }
  const auto during = transport.sample();
  {
    std::lock_guard<std::mutex> lock(state.mutex);
    state.release_state = true;
    state.changed.notify_all();
  }
  EXPECT_TRUE(entered);
  EXPECT_TRUE(during.diagnosis_ok);
  EXPECT_GT(during.diagnosis_at, 0.0);
  EXPECT_DOUBLE_EQ(during.robot_state_at, 0.0);
  EXPECT_FALSE(during.feedback.healthy);
  EXPECT_TRUE(wait_for_diagnosis(transport, 1s));
}

TEST_F(StepwiseTransportTest, ReadTimeoutKeepsOriginalSampleAgeButNeverConfirmsStopped)
{
  StepwiseTransport transport;
  ASSERT_TRUE(connect(transport));
  aubo_v4_test_sdk::emit_waypoint(valid_waypoint());
  ASSERT_TRUE(wait_for_diagnosis(transport, 1s));
  ASSERT_TRUE(transport.sample().feedback.healthy);
  auto & state = aubo_v4_test_sdk::state();
  {
    std::lock_guard<std::mutex> lock(state.mutex);
    state.state_result = aubo_robot_namespace::ErrCode_RequestTimeout;
    state.diagnosis_result = aubo_robot_namespace::ErrCode_RequestTimeout;
  }
  const auto original = transport.sample();
  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (transport.sample().state_result == 0 && std::chrono::steady_clock::now() < deadline) {
    std::size_t seen = 0;
    aubo_v4_test_sdk::wait_for_event(seen, 20ms);
  }
  const auto timed_out = transport.sample();
  EXPECT_EQ(timed_out.state_result, aubo_robot_namespace::ErrCode_RequestTimeout);
  EXPECT_TRUE(timed_out.feedback.healthy);
  EXPECT_FALSE(timed_out.feedback.stopped);
  EXPECT_DOUBLE_EQ(timed_out.robot_state_at, original.robot_state_at);
  EXPECT_DOUBLE_EQ(timed_out.diagnosis_at, original.diagnosis_at);
  {
    std::lock_guard<std::mutex> lock(state.mutex);
    state.state_result = 10004;
  }
  while (transport.sample().state_result != 10004 && std::chrono::steady_clock::now() < deadline) {
    std::size_t seen = 0;
    aubo_v4_test_sdk::wait_for_event(seen, 20ms);
  }
  EXPECT_FALSE(transport.sample().feedback.healthy);
}

TEST_F(StepwiseTransportTest, MotionCannotOverlapBlockedReadRpcAndTimeoutIsNeverResent)
{
  auto & state = aubo_v4_test_sdk::state();
  {
    std::lock_guard<std::mutex> lock(state.mutex);
    state.block_state = true;
    state.line_move_result = aubo_robot_namespace::ErrCode_RequestTimeout;
  }
  StepwiseTransport transport;
  ASSERT_TRUE(connect(transport));
  bool entered;
  {
    std::unique_lock<std::mutex> lock(state.mutex);
    entered = state.changed.wait_for(lock, 1s, [&state]() {return state.state_entered;});
  }
  EXPECT_TRUE(entered);
  EXPECT_TRUE(transport.submit(command(1, 1), seed));
  const bool ik_started = aubo_v4_test_sdk::wait_for_ik(30ms);
  {
    std::lock_guard<std::mutex> lock(state.mutex);
    state.release_state = true;
    state.changed.notify_all();
  }
  EXPECT_FALSE(ik_started);
  const auto result = wait_for_motion_result(transport, 1s);
  ASSERT_TRUE(result);
  EXPECT_FALSE(result->success);
  EXPECT_NE(result->detail.find("10007"), std::string::npos);
  EXPECT_EQ(line_move_count(), 1);
  EXPECT_EQ(ik_call_count(), 1);
}

TEST_F(StepwiseTransportTest, FixedFollowProfileIsConfiguredBeforeMotionAndNotPerTarget)
{
  StepwiseTransport transport;
  auto settings = options();
  settings.allow_follow_mode = true;
  settings.fixed_follow_profile = true;
  settings.speed_mps = 0.15;
  settings.maximum_step_m = 0.005;
  settings.follow_joint_speed_radps = 0.8;
  settings.follow_joint_acceleration_radps2 = 2.0;
  std::string error;
  ASSERT_TRUE(transport.connect(settings, error));
  EXPECT_EQ(transport.sample().follow_profile_updates, 1U);
  for (unsigned id = 1; id <= 4; ++id) {
    auto value = command(id, 1);
    value.follow_joint_mode = true;
    value.speed_mps = 0.01 * id;
    {
      auto & state = aubo_v4_test_sdk::state();
      std::lock_guard<std::mutex> lock(state.mutex);
      state.ik_delta[0] = 0.001 * id;
    }
    ASSERT_TRUE(transport.submit(value, seed));
    const auto result = wait_for_motion_result(transport, 1s);
    ASSERT_TRUE(result);
    EXPECT_TRUE(result->success);
    EXPECT_EQ(transport.sample().follow_profile_updates, 1U);
  }
  EXPECT_DOUBLE_EQ(transport.sample().follow_joint_speed_max_radps, 0.8);
}

TEST_F(StepwiseTransportTest, FollowStopCannotBeConfirmedBeforeSenderQuiesces)
{
  StepwiseTransport transport;
  auto settings = options();
  settings.allow_follow_mode = true;
  settings.single_stop_when_quiescent = true;
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

TEST_F(StepwiseTransportTest, RecoveryCannotPassTransportGateBeforeOldRequestsAndResultsDrain)
{
  aubo_v4_test_sdk::state().block_line_move = true;
  StepwiseTransport transport;
  ASSERT_TRUE(connect(transport));
  EXPECT_TRUE(transport.quiescent());
  ASSERT_TRUE(transport.submit(command(1, 1), seed));
  ASSERT_TRUE(aubo_v4_test_sdk::wait_for_line_move(500ms));
  EXPECT_FALSE(transport.quiescent());
  transport.cancel_and_stop(2);
  EXPECT_FALSE(transport.quiescent());
  ASSERT_TRUE(aubo_v4_test_sdk::wait_for_stop_count(1, 500ms));
  aubo_v4_test_sdk::release_line_move();
  const auto stopped = wait_for_stop_result(transport, 1s);
  ASSERT_TRUE(stopped && stopped->success);
  EXPECT_TRUE(transport.quiescent());
  EXPECT_FALSE(transport.submit(command(2, 1), seed));
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
