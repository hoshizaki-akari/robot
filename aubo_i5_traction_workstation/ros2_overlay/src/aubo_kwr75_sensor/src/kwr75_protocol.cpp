#include "aubo_kwr75_sensor/kwr75_protocol.hpp"

#include <cmath>
#include <cstring>
#include <iterator>

namespace aubo_kwr75_sensor
{
namespace
{

constexpr std::uint8_t kHeaderA = 0x48U;
constexpr std::uint8_t kHeaderB = 0x49U;
constexpr std::uint8_t kHeaderSecond = 0xAAU;
constexpr std::uint8_t kTrailerFirst = 0x0DU;
constexpr std::uint8_t kTrailerSecond = 0x0AU;
constexpr std::size_t kPayloadOffset = 2U;
constexpr std::size_t kTrailerOffset = 26U;

float read_little_endian_float(const std::uint8_t * bytes)
{
  static_assert(sizeof(float) == sizeof(std::uint32_t), "KWR75D uses IEEE-754 binary32 payloads");
  const std::uint32_t raw =
    static_cast<std::uint32_t>(bytes[0]) |
    (static_cast<std::uint32_t>(bytes[1]) << 8U) |
    (static_cast<std::uint32_t>(bytes[2]) << 16U) |
    (static_cast<std::uint32_t>(bytes[3]) << 24U);
  float value = 0.0F;
  std::memcpy(&value, &raw, sizeof(value));
  return value;
}

}  // namespace

bool Kwr75FrameParser::is_header(const std::uint8_t * frame)
{
  return (frame[0] == kHeaderA || frame[0] == kHeaderB) && frame[1] == kHeaderSecond;
}

bool Kwr75FrameParser::is_valid_frame(const std::uint8_t * frame)
{
  if (!is_header(frame) || frame[kTrailerOffset] != kTrailerFirst ||
    frame[kTrailerOffset + 1U] != kTrailerSecond)
  {
    return false;
  }

  for (std::size_t index = 0U; index < 6U; ++index) {
    if (!std::isfinite(read_little_endian_float(frame + kPayloadOffset + index * sizeof(float)))) {
      return false;
    }
  }
  return true;
}

Kwr75Wrench Kwr75FrameParser::decode_si(const std::uint8_t * frame)
{
  const auto value = [frame](const std::size_t index) {
      return static_cast<double>(
        read_little_endian_float(frame + kPayloadOffset + index * sizeof(float)));
    };
  return {
    value(0U) * kStandardGravityMps2,
    value(1U) * kStandardGravityMps2,
    value(2U) * kStandardGravityMps2,
    value(3U) * kStandardGravityMps2,
    value(4U) * kStandardGravityMps2,
    value(5U) * kStandardGravityMps2};
}

std::vector<Kwr75Wrench> Kwr75FrameParser::push(const std::uint8_t * data, const std::size_t size)
{
  std::vector<Kwr75Wrench> decoded;
  if (data == nullptr || size == 0U) {
    return decoded;
  }

  buffer_.insert(buffer_.end(), data, data + size);
  std::size_t offset = 0U;
  while (buffer_.size() - offset >= kKwr75FrameSize) {
    const auto * candidate = buffer_.data() + offset;
    if (is_valid_frame(candidate)) {
      decoded.push_back(decode_si(candidate));
      offset += kKwr75FrameSize;
      continue;
    }
    ++offset;
  }

  if (offset > 0U) {
    buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(offset));
  }
  // A candidate header may begin in either of the final two bytes. Keeping
  // the entire incomplete tail lets the next serial read complete that frame.
  if (buffer_.size() > kKwr75FrameSize - 1U) {
    buffer_.erase(
      buffer_.begin(),
      buffer_.end() - static_cast<std::ptrdiff_t>(kKwr75FrameSize - 1U));
  }
  return decoded;
}

std::vector<Kwr75Wrench> Kwr75FrameParser::push(const std::vector<std::uint8_t> & data)
{
  return push(data.data(), data.size());
}

void Kwr75FrameParser::reset()
{
  buffer_.clear();
}

std::size_t Kwr75FrameParser::buffered_bytes() const
{
  return buffer_.size();
}

const std::array<std::uint8_t, 4U> & start_continuous_command()
{
  static constexpr std::array<std::uint8_t, 4U> command{{0x48U, 0xAAU, 0x0DU, 0x0AU}};
  return command;
}

const std::array<std::uint8_t, 4U> & stop_continuous_command()
{
  static constexpr std::array<std::uint8_t, 4U> command{{0x43U, 0xAAU, 0x0DU, 0x0AU}};
  return command;
}

}  // namespace aubo_kwr75_sensor
