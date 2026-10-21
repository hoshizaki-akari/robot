#include <gtest/gtest.h>
#include <limits>

#include "fr_traction/source_freshness.hpp"

TEST(SourceFreshness, ClockStepsRejectTransitFramesAndRecover)
{
  fr_traction::SourceStampGuard guard;
  EXPECT_TRUE(guard.accept(100.0, 100.01, 10.0, 1.2));
  EXPECT_FALSE(guard.accept(100.01, 101.49, 10.01, 1.2));
  EXPECT_TRUE(guard.accept(101.49, 101.50, 10.02, 1.2));
  EXPECT_TRUE(guard.accept(90.0, 90.01, 10.03, 1.2));
  EXPECT_FALSE(guard.accept(90.0, 90.02, 10.04, 1.2));
  EXPECT_FALSE(guard.accept(89.99, 90.03, 10.05, 1.2));
}

TEST(SourceFreshness, RealPauseDoesNotRejuvenateOldSamples)
{
  fr_traction::SourceStampGuard guard;
  EXPECT_TRUE(guard.accept(100.0, 100.01, 10.0, 1.2));
  EXPECT_FALSE(guard.accept(100.0, 102.01, 12.0, 1.2));
  EXPECT_FALSE(guard.accept(103.0, 102.02, 12.01, 1.2));
  EXPECT_TRUE(guard.accept(102.02, 102.03, 12.02, 1.2));
}

TEST(SourceFreshness, BoundedClockSkewButNotStaleOrInvalid)
{
  EXPECT_TRUE(fr_traction::source_age_fresh(-0.001, 1.2));
  EXPECT_TRUE(fr_traction::source_age_fresh(-0.05, 1.2));
  EXPECT_TRUE(fr_traction::source_age_fresh(0.0, 1.2));
  EXPECT_TRUE(fr_traction::source_age_fresh(1.2, 1.2));
  EXPECT_FALSE(fr_traction::source_age_fresh(-0.050001, 1.2));
  EXPECT_FALSE(fr_traction::source_age_fresh(1.200001, 1.2));
  EXPECT_FALSE(fr_traction::source_age_fresh(std::numeric_limits<double>::infinity(), 1.2));
  EXPECT_FALSE(fr_traction::source_age_fresh(std::numeric_limits<double>::quiet_NaN(), 1.2));
  EXPECT_FALSE(fr_traction::source_age_fresh(0.0, -1.0));
}
