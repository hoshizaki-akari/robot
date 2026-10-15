#ifndef AUBO_KWR75_SENSOR__KWR75_PROTOCOL_HPP_
#define AUBO_KWR75_SENSOR__KWR75_PROTOCOL_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace aubo_kwr75_sensor
{

constexpr std::size_t kKwr75FrameSize = 28U;
constexpr double kStandardGravityMps2 = 9.80665;

struct Kwr75Wrench
{
  double fx_n;
  double fy_n;
  double fz_n;
  double mx_nm;
  double my_nm;
  double mz_nm;
};

// Incremental parser for KWR75D RS422 frames. It accepts arbitrary serial
// read boundaries and discards only enough bytes to re-establish synchronization.
class Kwr75FrameParser
{
public:
  std::vector<Kwr75Wrench> push(const std::uint8_t * data, std::size_t size);
  std::vector<Kwr75Wrench> push(const std::vector<std::uint8_t> & data);
  void reset();
  std::size_t buffered_bytes() const;

private:
  static bool is_header(const std::uint8_t * frame);
  static bool is_valid_frame(const std::uint8_t * frame);
  static Kwr75Wrench decode_si(const std::uint8_t * frame);

  std::vector<std::uint8_t> buffer_;
};

const std::array<std::uint8_t, 4U> & start_continuous_command();
const std::array<std::uint8_t, 4U> & stop_continuous_command();

}  // namespace aubo_kwr75_sensor

#endif  // AUBO_KWR75_SENSOR__KWR75_PROTOCOL_HPP_
