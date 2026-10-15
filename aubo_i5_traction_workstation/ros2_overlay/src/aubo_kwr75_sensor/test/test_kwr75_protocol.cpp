#include "aubo_kwr75_sensor/kwr75_protocol.hpp"

#include <cmath>
#include <cstdint>
#include <vector>

#include "gtest/gtest.h"

namespace aubo_kwr75_sensor
{
namespace
{

std::vector<std::uint8_t> example_frame()
{
  return {
    0x49U, 0xAAU, 0xBBU, 0x8AU, 0x5DU, 0xBEU, 0xA1U, 0x7DU,
    0x47U, 0x3FU, 0xE7U, 0xD1U, 0x89U, 0x3DU, 0x84U, 0xE8U,
    0xB3U, 0x3CU, 0x32U, 0x16U, 0x19U, 0xBCU, 0x73U, 0xDEU,
    0x15U, 0x3BU, 0x0DU, 0x0AU};
}

}  // namespace

TEST(Kwr75FrameParser, DecodesSuppliedExampleAndConvertsToSi)
{
  Kwr75FrameParser parser;
  const auto decoded = parser.push(example_frame());
  ASSERT_EQ(decoded.size(), 1U);
  EXPECT_NEAR(decoded[0].fx_n, -2.1216640913, 1e-8);
  EXPECT_NEAR(decoded[0].fy_n, 7.6419368944, 1e-8);
  EXPECT_NEAR(decoded[0].fz_n, 0.6599374154, 1e-8);
  EXPECT_NEAR(decoded[0].mx_nm, 0.2153683324, 1e-8);
  EXPECT_NEAR(decoded[0].my_nm, -0.0916301081, 1e-8);
  EXPECT_NEAR(decoded[0].mz_nm, 0.0224260294, 1e-8);
}

TEST(Kwr75FrameParser, HandlesSerialReadBoundaries)
{
  Kwr75FrameParser parser;
  const auto frame = example_frame();
  EXPECT_TRUE(parser.push(frame.data(), 5U).empty());
  EXPECT_TRUE(parser.push(frame.data() + 5U, 19U).empty());
  const auto decoded = parser.push(frame.data() + 24U, frame.size() - 24U);
  ASSERT_EQ(decoded.size(), 1U);
  EXPECT_NEAR(decoded[0].fy_n, 7.6419368944, 1e-8);
  EXPECT_EQ(parser.buffered_bytes(), 0U);
}

TEST(Kwr75FrameParser, RecoversAfterNoiseAndBadTrailer)
{
  const auto frame = example_frame();
  std::vector<std::uint8_t> stream{0x00U, 0x49U, 0xAAU, 0x01U, 0x02U, 0x03U};
  auto damaged = frame;
  damaged[26U] = 0x00U;
  stream.insert(stream.end(), damaged.begin(), damaged.end());
  stream.insert(stream.end(), frame.begin(), frame.end());

  Kwr75FrameParser parser;
  const auto decoded = parser.push(stream);
  ASSERT_EQ(decoded.size(), 1U);
  EXPECT_NEAR(decoded.front().fx_n, -2.1216640913, 1e-8);
}

TEST(Kwr75FrameParser, RejectsNonFinitePayloadAndFindsFollowingFrame)
{
  auto invalid = example_frame();
  invalid[2U] = 0x00U;
  invalid[3U] = 0x00U;
  invalid[4U] = 0xC0U;
  invalid[5U] = 0x7FU;
  const auto valid = example_frame();
  invalid.insert(invalid.end(), valid.begin(), valid.end());

  Kwr75FrameParser parser;
  const auto decoded = parser.push(invalid);
  ASSERT_EQ(decoded.size(), 1U);
  EXPECT_TRUE(std::isfinite(decoded.front().fx_n));
}

TEST(Kwr75FrameParser, ExposesVerifiedControlCommands)
{
  EXPECT_EQ(
    start_continuous_command(),
    (std::array<std::uint8_t, 4U>{{0x48U, 0xAAU, 0x0DU, 0x0AU}}));
  EXPECT_EQ(
    stop_continuous_command(),
    (std::array<std::uint8_t, 4U>{{0x43U, 0xAAU, 0x0DU, 0x0AU}}));
}

}  // namespace aubo_kwr75_sensor
