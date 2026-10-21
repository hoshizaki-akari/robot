#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include "fr_traction/aubo_drag_model.hpp"

namespace ft = fr_traction;

TEST(AuboDragModel, SameLinearSteadyMobilityOnAllSpatialAxes)
{
  for (const auto axis : {ft::Vec3{1, 0, 0}, ft::Vec3{0, 1, 0}, ft::Vec3{0, 0, 1},
      ft::Vec3{0.6, 0.0, 0.8}})
  {
    for (double sign : {-1.0, 1.0}) {
      ft::AuboDragModel model;
      ft::Vec3 velocity;
      for (unsigned i = 0; i < 300; ++i) {
        velocity = model.update(axis * (sign * 5), 0.01);
      }
      const auto expected = axis * (sign * (5.0 - 0.35) * 0.010);
      EXPECT_NEAR(ft::norm(velocity - expected), 0.0, 1e-9);
    }
  }
}

TEST(AuboDragModel, ReversalCannotCommandAgainstCurrentForceAndReleaseHasNoTail)
{
  ft::AuboDragModel model;
  for (unsigned i = 0; i < 100; ++i) {
    model.update({8, 4, 2}, 0.01);
  }
  for (unsigned i = 0; i < 100; ++i) {
    const ft::Vec3 force{-8, -4, -2};
    EXPECT_GE(ft::dot(model.update(force, 0.01), force), -1e-12);
  }
  EXPECT_DOUBLE_EQ(ft::norm(model.update({}, 0.01)), 0.0);
  for (unsigned i = 0; i < 40; ++i) {
    EXPECT_DOUBLE_EQ(ft::norm(model.update({0.3, 0, 0}, 0.01)), 0.0);
  }
  EXPECT_DOUBLE_EQ(ft::norm(model.update({1, 0, 0}, 0.01)), 0.0);
}

TEST(AuboDragModel, SuppressesTransverseOscillationWithoutErasingDiagonalIntent)
{
  ft::AuboDragConfig config;
  config.maximum_acceleration_mps2 = 100.0;
  ft::AuboDragModel model(config);
  double output_energy = 0.0, raw_energy = 0.0;
  ft::Vec3 sum;
  for (unsigned i = 0; i < 1000; ++i) {
    const double ripple = std::sin(2.0 * 3.141592653589793 * 8.0 * i * 0.01);
    const ft::Vec3 force{5.0, ripple, 5.0};
    const auto velocity = model.update(force, 0.01);
    if (i > 100) {
      output_energy += velocity.y * velocity.y;
      raw_energy += std::pow(0.010 * ripple, 2);
      sum = sum + velocity;
    }
  }
  EXPECT_LT(output_energy, raw_energy * 0.1);
  EXPECT_NEAR(sum.x, sum.z, 1e-12);
  EXPECT_GT(sum.x, 30.0);
}

TEST(AuboDragModel, InvalidInputResetsRatherThanReplayingPreviousVelocity)
{
  ft::AuboDragModel model;
  for (unsigned i = 0; i < 50; ++i) {
    model.update({10, 0, 0}, 0.01);
  }
  EXPECT_DOUBLE_EQ(ft::norm(model.update({NAN, 0, 0}, 0.01)), 0.0);
  EXPECT_DOUBLE_EQ(ft::norm(model.update({10, 0, 0}, 0.4)), 0.0);
  EXPECT_DOUBLE_EQ(ft::norm(model.update({10, 0, 0}, 0.01)), 0.0);
}

TEST(AuboDragModel, EquivalentElapsedTimeIsIndependentOfTimerFrequency)
{
  ft::AuboDragConfig config;
  config.start_confirm_s = 0.0;
  config.maximum_acceleration_mps2 = 100.0;
  ft::AuboDragModel slow(config), fast(config);
  ft::Vec3 a, b;
  for (unsigned i = 0; i < 20; ++i) {
    a = slow.update({5, 5, 0}, 0.01);
  }
  for (unsigned i = 0; i < 40; ++i) {
    b = fast.update({5, 5, 0}, 0.005);
  }
  EXPECT_NEAR(ft::norm(a - b), 0.0, 1e-12);
}
