#include <gtest/gtest.h>
#include <limits>
#include <string>
#include "aubo_v4_driver/state_validation.hpp"
TEST(StateValidation, AcceptsFiniteInput) {
  aubo_v4_driver::PoseSample s; s.position = {0.1, -0.2, 0.3}; s.orientation = {0.0, 0.0, 0.0, 1.0};
  s.joints = {0.0, 0.1, -0.2, 0.3, -0.4, 0.5}; std::string reason; EXPECT_TRUE(
    aubo_v4_driver::valid_pose_sample(
      s,
      &reason)); EXPECT_EQ(reason, "valid");
}
TEST(StateValidation, RejectsNonFiniteJoint) {
  aubo_v4_driver::PoseSample s; s.orientation = {0.0, 0.0, 0.0, 1.0};
  s.joints[3] = std::numeric_limits<double>::quiet_NaN(); std::string reason; EXPECT_FALSE(
    aubo_v4_driver::valid_pose_sample(
      s,
      &reason));
  EXPECT_NE(reason.find("joint"), std::string::npos);
}
TEST(StateValidation, RejectsZeroQuaternion) {
  aubo_v4_driver::PoseSample s; std::string reason;
  EXPECT_FALSE(aubo_v4_driver::valid_pose_sample(s, &reason)); EXPECT_NE(
    reason.find(
      "zero-length"), std::string::npos);
}
TEST(StateValidation, RequiresPositiveDiagnosisEvidence) {
  aubo_v4_driver::DiagnosisSample d; std::string reason; EXPECT_FALSE(
    aubo_v4_driver::diagnosis_is_healthy(
      d,
      &reason)); EXPECT_NE(reason.find("power"), std::string::npos); d.arm_power_status = true;
  EXPECT_TRUE(aubo_v4_driver::diagnosis_is_healthy(d, &reason));
}
TEST(StateValidation, RejectsEmergencyAndCanFault) {
  aubo_v4_driver::DiagnosisSample d; d.arm_power_status = true; d.remote_emergency = true;
  std::string reason; EXPECT_FALSE(aubo_v4_driver::diagnosis_is_healthy(d, &reason)); EXPECT_NE(
    reason.find(
      "emergency"), std::string::npos); d.remote_emergency = false; d.arm_canbus_status = 1U;
  EXPECT_FALSE(aubo_v4_driver::diagnosis_is_healthy(d, &reason)); EXPECT_NE(
    reason.find(
      "CAN"), std::string::npos);
}
