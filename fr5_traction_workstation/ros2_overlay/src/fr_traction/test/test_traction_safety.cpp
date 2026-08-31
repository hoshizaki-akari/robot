#include <gtest/gtest.h>

#include <limits>

#include "fr_traction/traction_safety.hpp"

namespace
{

fr_traction::SafetySample nominal_sample()
{
  fr_traction::SafetySample sample;
  sample.wrench_valid = true;
  sample.wrench_fresh = true;
  sample.ee_fresh = true;
  sample.controller_healthy = true;
  sample.ui_heartbeat_fresh = true;
  sample.raw_wrench = {0.0, 0.0, -10.0};
  sample.metrics.actual_force_n = 10.0;
  sample.metrics.lateral_force_n = 0.0;
  sample.axis_displacement_m = 0.0;
  return sample;
}

}  // namespace

TEST(TractionSafety, RejectsInvalidStaleAndUnhealthyInputs)
{
  fr_traction::SafetyMonitor monitor;
  auto sample = nominal_sample();
  sample.wrench_valid = false;
  EXPECT_EQ(monitor.update(sample, 0.0, false), fr_traction::SafetyFault::WRENCH_INVALID);
  sample = nominal_sample();
  sample.wrench_fresh = false;
  EXPECT_EQ(monitor.update(sample, 0.0, false), fr_traction::SafetyFault::WRENCH_TIMEOUT);
  sample = nominal_sample();
  sample.ee_fresh = false;
  EXPECT_EQ(monitor.update(sample, 0.0, false), fr_traction::SafetyFault::EE_STATE_TIMEOUT);
  sample = nominal_sample();
  sample.controller_healthy = false;
  EXPECT_EQ(monitor.update(sample, 0.0, false), fr_traction::SafetyFault::ROS2_CONTROL_ERROR);
}

TEST(TractionSafety, HardLimitsAreImmediateAndTimedLimitsLatch)
{
  fr_traction::SafetyMonitor monitor;
  auto sample = nominal_sample();
  sample.raw_wrench = {0.0, 0.0, -30.0};
  EXPECT_EQ(monitor.update(sample, 0.0, false), fr_traction::SafetyFault::HARD_OVERFORCE);

  sample = nominal_sample();
  sample.metrics.actual_force_n = 25.0;
  EXPECT_EQ(monitor.update(sample, 1.0, false), fr_traction::SafetyFault::NONE);
  EXPECT_EQ(monitor.update(sample, 1.049, false), fr_traction::SafetyFault::NONE);
  EXPECT_EQ(monitor.update(sample, 1.050, false), fr_traction::SafetyFault::OVERFORCE);

  monitor.reset();
  sample = nominal_sample();
  sample.metrics.lateral_force_n = 5.0;
  EXPECT_EQ(monitor.update(sample, 2.0, false), fr_traction::SafetyFault::NONE);
  EXPECT_EQ(monitor.update(sample, 2.199, false), fr_traction::SafetyFault::NONE);
  EXPECT_EQ(monitor.update(sample, 2.200, false), fr_traction::SafetyFault::LATERAL_FORCE);
}

TEST(TractionSafety, ChecksTravelAndOptionalUiHeartbeat)
{
  fr_traction::SafetyMonitor monitor;
  auto sample = nominal_sample();
  sample.axis_displacement_m = 0.050;
  EXPECT_EQ(monitor.update(sample, 0.0, false), fr_traction::SafetyFault::AXIAL_TRAVEL_LIMIT);
  sample = nominal_sample();
  sample.ui_heartbeat_fresh = false;
  EXPECT_EQ(monitor.update(sample, 0.0, true), fr_traction::SafetyFault::UI_HEARTBEAT_TIMEOUT);
  EXPECT_STREQ(fr_traction::SafetyMonitor::code(fr_traction::SafetyFault::OVERFORCE), "OVERFORCE");
}

TEST(TractionSafety, RejectsNonFiniteInputs)
{
  fr_traction::SafetyMonitor monitor;
  auto sample = nominal_sample();
  sample.raw_wrench.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(monitor.update(sample, 0.0, false), fr_traction::SafetyFault::WRENCH_INVALID);
}
