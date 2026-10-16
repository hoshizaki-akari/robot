#include <gtest/gtest.h>

#include "fr_traction/zero_pose_policy.hpp"

namespace
{
fr_traction::ZeroPose valid_pose()
{
  return {{0.1, -0.2, 0.3}, {0.0, 0.0, 0.0, 1.0}};
}
}  // namespace

TEST(ZeroPosePolicy, OrientationComparisonNormalizesToleratedQuaternionScale)
{
  const auto saved = valid_pose();
  auto current = saved;
  current.orientation_xyzw[3] = 0.9995;
  EXPECT_TRUE(fr_traction::zero_pose_unit_quaternion(current.orientation_xyzw));
  EXPECT_NEAR(fr_traction::zero_pose_orientation_error_rad(current, saved), 0.0, 1e-12);
  EXPECT_FALSE(
    fr_traction::zero_pose_return_allowed(
      current, saved, true, true, true, true, true, true, false, -1.0, 1.0, .01, .05));
  EXPECT_FALSE(
    fr_traction::zero_pose_return_allowed(
      current, saved, true, true, true, true, true, true, false, .5, 1.0, .01,
      std::numeric_limits<double>::infinity()));
}

TEST(ZeroPosePolicy, CaptureRequiresStoppedFreshLiveInputsAndUnitQuaternion)
{
  const auto pose = valid_pose();
  EXPECT_TRUE(fr_traction::zero_pose_capture_allowed(pose, true, true, true, true, true, false));
  EXPECT_FALSE(fr_traction::zero_pose_capture_allowed(pose, false, true, true, true, true, false));
  EXPECT_FALSE(fr_traction::zero_pose_capture_allowed(pose, true, false, true, true, true, false));
  auto invalid = pose;
  invalid.orientation_xyzw = {0.0, 0.0, 0.0, 0.0};
  EXPECT_FALSE(
    fr_traction::zero_pose_capture_allowed(
      invalid, true, true, true, true, true,
      false));
}

TEST(ZeroPosePolicy, ReturnRequiresUnloadedMatchingOrientationAndBoundedTravel)
{
  const auto saved = valid_pose();
  auto current = saved;
  current.position[0] += 0.01;
  EXPECT_TRUE(
    fr_traction::zero_pose_return_allowed(
      current, saved, true, true, true, true, true, true, false, 0.5, 1.0, 0.01, 0.05));
  EXPECT_FALSE(
    fr_traction::zero_pose_return_allowed(
      current, saved, true, true, true, true, true, true, false, 1.1, 1.0, 0.01, 0.05));
  current.orientation_xyzw = {0.0, 0.0, 0.02, std::sqrt(1.0 - 0.0004)};
  EXPECT_FALSE(
    fr_traction::zero_pose_return_allowed(
      current, saved, true, true, true, true, true, true, false, 0.5, 1.0, 0.01, 0.05));
  current = saved;
  current.position[0] += 0.06;
  EXPECT_FALSE(
    fr_traction::zero_pose_return_allowed(
      current, saved, true, true, true, true, true, true, false, 0.5, 1.0, 0.01, 0.05));
}
