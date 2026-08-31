#include <gtest/gtest.h>

#include <limits>

#include "fairino_hardware/finite_checks.hpp"

TEST(FiniteChecks, AcceptsAllSixFiniteValues)
{
  const double values[6] = {0.0, 1.0, -1.0, 2.5, -3.5, 6.0};
  EXPECT_TRUE(fairino_hardware::all_finite(values, 6));
}

TEST(FiniteChecks, RejectsNonFiniteValueAtEveryIndex)
{
  for (std::size_t index = 0; index < 6; ++index) {
    double values[6] = {0.0, 1.0, -1.0, 2.5, -3.5, 6.0};
    values[index] = index % 2 == 0 ? std::numeric_limits<double>::quiet_NaN() :
      std::numeric_limits<double>::infinity();
    EXPECT_FALSE(fairino_hardware::all_finite(values, 6)) << "index=" << index;
  }
}

TEST(FiniteChecks, HandlesEmptyAndNullInput)
{
  EXPECT_TRUE(fairino_hardware::all_finite(nullptr, 0));
  EXPECT_FALSE(fairino_hardware::all_finite(nullptr, 1));
}
