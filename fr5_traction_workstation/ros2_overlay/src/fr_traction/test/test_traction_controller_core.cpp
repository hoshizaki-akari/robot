#include "fr_traction/traction_controller_core.hpp"

#include <cmath>

#include "gtest/gtest.h"

namespace fr_traction
{

TEST(TractionControllerCore, ProducesOnlyLockedAxisVelocity)
{
  TractionControllerCore core(10.0, 80.0, 0.5, 0.005, 0.02);
  const Vec3 direction{1.0, 0.0, 0.0};
  auto output = core.update(ControlMode::TRACTION, direction, 10.0, {0.0, 0.0, 0.0}, 0.01);
  EXPECT_TRUE(output.valid);
  EXPECT_GT(output.scalar_velocity_mps, 0.0);
  EXPECT_NEAR(output.linear_velocity.y, 0.0, 1e-12);
  EXPECT_NEAR(output.linear_velocity.z, 0.0, 1e-12);

  const double cross_norm = std::sqrt(
    std::pow(output.linear_velocity.y * direction.z - output.linear_velocity.z * direction.y, 2) +
    std::pow(output.linear_velocity.z * direction.x - output.linear_velocity.x * direction.z, 2) +
    std::pow(output.linear_velocity.x * direction.y - output.linear_velocity.y * direction.x, 2));
  EXPECT_LT(cross_norm, 1e-9);
  EXPECT_DOUBLE_EQ(output.linear_velocity.y, 0.0);
  EXPECT_DOUBLE_EQ(output.linear_velocity.z, 0.0);

  core.reset();
  output = core.update(ControlMode::TRACTION, direction, 10.0, {12.0, 0.0, 0.0}, 0.01);
  EXPECT_LT(output.scalar_velocity_mps, 0.0);
}

TEST(TractionControllerCore, DeadbandAndDisableStopTheController)
{
  TractionControllerCore core(10.0, 80.0, 0.5, 0.005, 0.02);
  auto output = core.update(ControlMode::TRACTION, {1.0, 0.0, 0.0}, 10.0, {9.8, 0.0, 0.0}, 0.01);
  EXPECT_TRUE(output.valid);
  EXPECT_NEAR(output.scalar_velocity_mps, 0.0, 1e-12);
  output = core.update(ControlMode::DISABLED, {1.0, 0.0, 0.0}, 0.0, {9.8, 0.0, 0.0}, 0.01);
  EXPECT_TRUE(output.valid);
  EXPECT_DOUBLE_EQ(output.scalar_velocity_mps, 0.0);
  EXPECT_DOUBLE_EQ(output.linear_velocity.x, 0.0);
}

TEST(TractionControllerCore, RejectsInvalidDirection)
{
  TractionControllerCore core(10.0, 80.0, 0.5, 0.005, 0.02);
  EXPECT_FALSE(
    core.update(
      ControlMode::TRACTION, {0.0, 0.0, 0.0}, 10.0, {-1.0, 0.0, 0.0}, 0.01).valid);
}

}  // namespace fr_traction
