#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include "aubo_v4_driver/tcp_can_drag.hpp"

namespace av = aubo_v4_driver;
namespace
{
// Explicit unit-test algebra stub, not a model of any robot and never used by
// a runtime executable/UI. Tests isolate sequencing, buffering and derivatives.
struct Stub
{
  unsigned queued{0};
  int enters{0}, leaves{0}, sends{0}, enter_result{0}, send_result{0};
  bool authorized{true}, fail_fk{false};
  av::StreamPose measured;
  std::vector<std::array<double, 6>> points;
  av::TcpCanIo io()
  {
    av::TcpCanIo value;
    value.enter = [this]() {++enters; return enter_result;};
    value.leave = [this]() {++leaves; return 0;};
    value.queued_points = [this](unsigned & count) {count = queued; return true;};
    value.inverse = [](const av::StreamPose & request, av::StreamPose & solved) {
        solved = request;
        std::copy(request.position.begin(), request.position.end(), solved.joints.begin());
        return true;
      };
    value.forward = [this](const std::array<double, 6> & joints, av::StreamPose & pose) {
        pose = {};
        pose.joints = joints;
        std::copy(joints.begin(), joints.begin() + 3, pose.position.begin());
        return !fail_fk;
      };
    value.send = [this](const auto & batch) {
        ++sends;
        points.insert(points.end(), batch.begin(), batch.end());
        if (send_result == 0) {queued += static_cast<unsigned>(batch.size());}
        return send_result;
      };
    return value;
  }
  av::TcpCanResult update(av::TcpCanDragSession & session, std::array<double, 3> v, double t = 1)
  {return session.update(measured, v, {0, 0, 0, 1}, 1, t, [this]() {return authorized;}, io());}
};
}

TEST(TcpCanDrag, ExplicitEntryAndBoundedFiveMillisecondSamples)
{
  av::TcpCanDragSession session;
  Stub stub;
  const auto first = stub.update(session, {0.01, 0.02, 0.03});
  ASSERT_TRUE(first.success) << first.detail;
  EXPECT_EQ(first.points_sent, 10U);
  EXPECT_EQ(stub.enters, 1);
  std::array<double, 6> previous{}, previous_speed{};
  for (const auto & point : stub.points) {
    for (unsigned j = 0; j < 6; ++j) {
      const double speed = (point[j] - previous[j]) / 0.005;
      EXPECT_LE(std::abs(speed), 1.0 + 1e-9);
      EXPECT_LE(std::abs(speed - previous_speed[j]) / 0.005, 2.5 + 1e-8);
      previous_speed[j] = speed;
    }
    previous = point;
  }
  EXPECT_EQ(session.points(), 10U);
  const auto held = stub.update(session, {0.01, 0.02, 0.03}, 1.01);
  EXPECT_TRUE(held.success);
  EXPECT_EQ(held.points_sent, 0U);
  EXPECT_EQ(stub.sends, 1);
  stub.queued = 4;
  const auto replenished = stub.update(session, {0.01, 0.02, 0.03}, 1.03);
  EXPECT_TRUE(replenished.success);
  EXPECT_EQ(replenished.points_sent, 6U);
  EXPECT_EQ(stub.enters, 1);
}

TEST(TcpCanPacing, AckLatencyDoesNotInventBufferReserve)
{
  av::TcpCanPacing pacing;
  EXPECT_EQ(pacing.queued(1), 0U);
  ASSERT_TRUE(pacing.accept(10, 1, 1.005));
  EXPECT_EQ(pacing.queued(1.005), 9U);
  EXPECT_EQ(pacing.queued(1.03), 4U);
  ASSERT_TRUE(pacing.accept(6, 1.031, 1.036));
  EXPECT_EQ(pacing.queued(1.05), 6U);
  EXPECT_EQ(pacing.queued(1.08), 0U);
  EXPECT_FALSE(pacing.accept(10, 1.10, 1.105));
  pacing.reset();
  EXPECT_TRUE(pacing.accept(10, 2, 2.005));
}

TEST(TcpCanPacing, SlowSendOrInvalidTimesNeverProduceCatchUpCredit)
{
  av::TcpCanPacing pacing;
  EXPECT_FALSE(pacing.accept(10, 1, 1.2));
  EXPECT_FALSE(pacing.initialized());
  EXPECT_FALSE(pacing.accept(0, 1, 1.001));
  EXPECT_FALSE(pacing.accept(11, 1, 1.001));
  EXPECT_FALSE(pacing.accept(10, 1, 0.9));
  EXPECT_FALSE(pacing.accept(10, NAN, 1));
  EXPECT_FALSE(pacing.accept(10, 1, NAN));
  ASSERT_TRUE(pacing.accept(10, 2, 2.005));
  EXPECT_FALSE(pacing.accept(6, 2.049, 2.055));
}

TEST(TcpCanPacing, LongRunWithFiveMillisecondRepliesKeepsBoundedReserveWithoutDrift)
{
  av::TcpCanPacing pacing;
  double now = 10.0;
  ASSERT_TRUE(pacing.accept(10, now, now + 0.005));
  unsigned sent = 10;
  // Explicit time-arithmetic regression, not robot simulation. A slow
  // diagnostic worker does not participate in the sender clock at all.
  for (unsigned i = 0; i < 10000; ++i) {
    now += 0.01;
    const unsigned left = pacing.queued(now);
    ASSERT_GT(left, 0U);
    ASSERT_LE(left, 10U);
    if (left <= 4) {
      ASSERT_TRUE(pacing.accept(10 - left, now + 0.001, now + 0.006));
      sent += 10 - left;
    }
  }
  EXPECT_NEAR(sent * 0.005 - (now - 10.0), 0.04, 0.015);
}

TEST(TcpCanDrag, DirectionChangesRespectJointAccelerationWithoutPositionReset)
{
  av::TcpCanDragSession session;
  Stub stub;
  ASSERT_TRUE(stub.update(session, {0.04, 0, 0}).success);
  const auto tail = stub.points.back();
  stub.queued = 4;
  ASSERT_TRUE(stub.update(session, {-0.04, 0, 0}, 1.03).success);
  EXPECT_LT(std::abs(stub.points[10][0] - tail[0]), 0.00021);
  for (std::size_t i = 2; i < stub.points.size(); ++i) {
    const double acc = (stub.points[i][0] - 2 * stub.points[i - 1][0] + stub.points[i - 2][0]) /
      (0.005 * 0.005);
    EXPECT_LE(std::abs(acc), 2.5 + 1e-8);
  }
}

TEST(TcpCanDrag, ReleaseNeedsNoMoreSchedulerJobsAndStopsThePlannedTail)
{
  av::TcpCanDragSession session;
  Stub stub;
  ASSERT_TRUE(stub.update(session, {0.04, 0, 0}).success);
  stub.queued = 4;
  ASSERT_TRUE(stub.update(session, {}, 1.03).success);
  EXPECT_DOUBLE_EQ(stub.points.back()[0], stub.points[stub.points.size() - 2][0]);
  stub.queued = 0;
  stub.measured.joints = stub.points.back();
  stub.measured.position[0] = stub.points.back()[0];
  const auto idle = stub.update(session, {}, 1.08);
  EXPECT_TRUE(idle.success);
  EXPECT_EQ(idle.points_sent, 0U);
}

TEST(TcpCanDrag, MovingUnderrunAndOversizedBufferNeverCatchUpOrAutoResume)
{
  av::TcpCanDragSession session;
  Stub stub;
  ASSERT_TRUE(stub.update(session, {0.01, 0, 0}).success);
  stub.queued = 0;
  const auto underrun = stub.update(session, {0.01, 0, 0}, 1.2);
  EXPECT_FALSE(underrun.success);
  EXPECT_NE(underrun.detail.find("underrun"), std::string::npos);
  EXPECT_FALSE(stub.update(session, {0.01, 0, 0}, 1.21).success);
  EXPECT_EQ(stub.sends, 1);
  EXPECT_TRUE(session.leave_after_stop(stub.io()));
  EXPECT_FALSE(session.active());
  stub.queued = 11;
  EXPECT_FALSE(stub.update(session, {0.01, 0, 0}, 2).success);
}

TEST(TcpCanDrag, LostSendReplyIsNotRetriedAndLeaveRequiresEmptyBuffer)
{
  av::TcpCanDragSession session;
  Stub stub;
  stub.send_result = 10007;
  EXPECT_FALSE(stub.update(session, {0.01, 0, 0}).success);
  EXPECT_FALSE(stub.update(session, {0.01, 0, 0}, 1.01).success);
  EXPECT_EQ(stub.sends, 1);
  stub.queued = 2;
  EXPECT_FALSE(session.leave_after_stop(stub.io()));
  EXPECT_EQ(stub.leaves, 0);
  stub.queued = 0;
  EXPECT_TRUE(session.leave_after_stop(stub.io()));
  EXPECT_EQ(stub.leaves, 1);
}

TEST(TcpCanDrag, EntryFailureCancellationBadFkAndInvalidInputsCannotSend)
{
  {
    av::TcpCanDragSession session;
    Stub stub;
    stub.enter_result = -1;
    EXPECT_FALSE(stub.update(session, {0.01, 0, 0}).success);
    EXPECT_TRUE(session.active()); // Ownership is ambiguous until stop/leave.
    EXPECT_EQ(stub.sends, 0);
    EXPECT_TRUE(session.leave_after_stop(stub.io()));
  }
  for (unsigned scenario = 0; scenario < 3; ++scenario) {
    av::TcpCanDragSession session;
    Stub stub;
    if (scenario == 0) {stub.authorized = false;}
    if (scenario == 1) {stub.fail_fk = true;}
    const double force_velocity = scenario == 2 ? NAN : 0.01;
    EXPECT_FALSE(stub.update(session, {force_velocity, 0, 0}).success);
    EXPECT_EQ(stub.sends, 0);
  }
  EXPECT_THROW(av::TcpCanDragSession(av::TcpCanConfig{0.01}), std::invalid_argument);
}

TEST(TcpCanDrag, ExceptionAfterSendingLatchesAndJointBranchJumpCannotBeClippedIntoMotion)
{
  {
    av::TcpCanDragSession session;
    Stub stub;
    auto io = stub.io();
    io.send = [&stub](const auto &) -> int {++stub.sends; throw std::runtime_error("lost reply");};
    const auto failed = session.update(
      stub.measured, {0.01, 0, 0}, {0, 0, 0, 1}, 1, 1, []() {
        return true;
      }, io);
    EXPECT_FALSE(failed.success);
    EXPECT_TRUE(session.faulted());
    EXPECT_FALSE(stub.update(session, {0.01, 0, 0}, 1.01).success);
    EXPECT_EQ(stub.sends, 1);
  }
  {
    av::TcpCanDragSession session;
    Stub stub;
    auto io = stub.io();
    const auto inverse = io.inverse;
    io.inverse = [inverse](const auto & request, auto & solved) {
        inverse(request, solved);
        solved.joints[5] += 6.28;
        return true;
      };
    const auto failed = session.update(
      stub.measured, {0.01, 0, 0}, {0, 0, 0, 1}, 1, 1, []() {
        return true;
      }, io);
    EXPECT_FALSE(failed.success);
    EXPECT_NE(failed.detail.find("branch"), std::string::npos);
    EXPECT_EQ(stub.sends, 0);
  }
}
