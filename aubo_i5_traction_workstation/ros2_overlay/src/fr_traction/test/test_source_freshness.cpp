#include <gtest/gtest.h>
#include <limits>

#include "fr_traction/source_freshness.hpp"

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
