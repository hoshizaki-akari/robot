#include "fr_traction/traction_math.hpp"

#include <cmath>
#include <vector>

#include "gtest/gtest.h"

namespace fr_traction
{

TEST(TractionMath, ProjectsAxialAndLateralForce)
{
  ForceMetrics metrics;
  ASSERT_TRUE(project_force({10.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, metrics));
  EXPECT_NEAR(metrics.actual_force_n, 10.0, 1e-12);
  EXPECT_NEAR(metrics.lateral_force_n, 0.0, 1e-12);

  ASSERT_TRUE(project_force({10.0, 3.0, 4.0}, {1.0, 0.0, 0.0}, metrics));
  EXPECT_NEAR(metrics.actual_force_n, 10.0, 1e-12);
  EXPECT_NEAR(metrics.lateral_force_n, 5.0, 1e-12);

  ASSERT_TRUE(project_force({0.0, 3.0, 4.0}, {1.0, 0.0, 0.0}, metrics));
  EXPECT_NEAR(metrics.actual_force_n, 0.0, 1e-12);
  EXPECT_NEAR(metrics.lateral_force_n, 5.0, 1e-12);
}

TEST(TractionMath, RejectsInvalidDirectionAndWrench)
{
  ForceMetrics metrics;
  EXPECT_FALSE(project_force({1.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, metrics));
  EXPECT_FALSE(project_force({1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, metrics));
  EXPECT_FALSE(project_force({NAN, 0.0, 0.0}, {1.0, 0.0, 0.0}, metrics));
  Vec3 unit;
  EXPECT_FALSE(normalize({INFINITY, 0.0, 0.0}, unit));
}

TEST(TractionMath, LowPassUsesActualDt)
{
  FirstOrderLowPass filter(5.0);
  filter.update({0.0, 0.0, 0.0}, 0.001);
  Vec3 output{};
  for (int i = 0; i < 100; ++i) {
    output = filter.update({1.0, 0.0, 0.0}, 0.001);
  }
  const double expected = 1.0 - std::exp(-2.0 * 3.14159265358979323846 * 5.0 * 0.1);
  EXPECT_NEAR(output.x, expected, 0.05);
}

TEST(TractionMath, RobustCalibrationRejectsOutliersAndWobble)
{
  std::vector<Vec3> samples;
  for (int i = 0; i < 90; ++i) {
    samples.push_back({3.0, 0.0, 0.0});
  }
  for (int i = 0; i < 10; ++i) {
    samples.push_back({-30.0, 20.0, -15.0});
  }
  const auto result = robust_calibrate_direction(samples, 80, 15.0);
  ASSERT_TRUE(result.success);
  EXPECT_NEAR(result.direction.x, 1.0, 1e-12);
  EXPECT_NEAR(result.direction.y, 0.0, 1e-12);
  EXPECT_LT(result.angle_p95_deg, 2.0);
  EXPECT_NEAR(result.retained_fraction, 0.90, 1e-12);

  samples.clear();
  for (int i = 0; i < 100; ++i) {
    samples.push_back(i % 2 == 0 ? Vec3{3.0, 0.0, 0.0} : Vec3{2.0, 2.0, 0.0});
  }
  EXPECT_FALSE(robust_calibrate_direction(samples, 80, 15.0).success);
}

TEST(TractionMath, DirectionCalibrationIgnoresChangingTensionMagnitude)
{
  std::vector<Vec3> samples;
  for (int i = 0; i < 100; ++i) {
    const double magnitude = 9.0 - 3.0 * static_cast<double>(i) / 99.0;
    const double angle = static_cast<double>((i % 7) - 3) * 0.4 *
      3.14159265358979323846 / 180.0;
    samples.push_back({magnitude * std::cos(angle), magnitude * std::sin(angle), 0.0});
  }
  const auto result = robust_calibrate_direction(samples, 80, 15.0);
  ASSERT_TRUE(result.success);
  EXPECT_GT(result.retained_fraction, 0.95);
  EXPECT_NEAR(result.direction.x, 1.0, 1e-3);
  EXPECT_NEAR(result.direction.y, 0.0, 1e-3);
  EXPECT_LT(result.angle_p95_deg, 2.0);
}

}  // namespace fr_traction
