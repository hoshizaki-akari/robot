#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace aubo_v4_driver
{
struct DragBackendSelection
{
  bool use_tcp_can{false};
  std::string reason;
};
// Production qualification gate. Repeated real 298-300 ms send calls violate
// the 50 ms candidate contract. No command-line/ROS opt-in may bypass this
// withdrawal. Keep the pure candidate/tests for investigation, not live use.
inline DragBackendSelection select_drag_backend(bool requested_tcp_can)
{
  return {false, requested_tcp_can ?
    "TCP2CAN candidate withdrawn: measured SDK send latency exceeds short buffer; native FollowMode selected before motion"
    :
    "native FollowMode selected"};
}

// Sender-local pacing ledger, NOT a measurement of controller occupancy or
// physical arrival. Only SDK-accepted 5 ms points extend the deadline. Slow
// diagnostic RPCs must never run between short-buffer replenishments.
class TcpCanPacing
{
public:
  bool initialized() const {return initialized_;}
  unsigned queued(double now) const;
  bool accept(unsigned count, double started, double finished);
  void reset() {initialized_ = false; deadline_ = 0.0;}

private:
  bool initialized_{false};
  double deadline_{0.0};
};

struct StreamPose
{
  std::array<double, 6> joints{};
  std::array<double, 3> position{};
  std::array<double, 4> orientation{0, 0, 0, 1};
};
struct TcpCanConfig
{
  double period_s{0.005};
  unsigned low_water_points{4}, high_water_points{10};
  double joint_speed_radps{1.0}, joint_acceleration_radps2{2.5};
  double maximum_speed_mps{0.2}, maximum_lead_m{0.02};
  double maximum_ik_step_rad{0.05};
  double orientation_tolerance_rad{0.1};
};
// Injected boundary: tests use named doubles; production binds the one existing
// SDK owner. Construction/planning never creates a controller connection.
struct TcpCanIo
{
  std::function<int()> enter, leave;
  std::function<bool(unsigned &)> queued_points;
  std::function<bool(const StreamPose &, StreamPose &)> inverse;
  std::function<bool(const std::array<double, 6> &, StreamPose &)> forward;
  std::function<int(const std::vector<std::array<double, 6>> &)> send;
};
struct TcpCanResult
{
  bool success{false};
  unsigned points_sent{0}, queued_points{0};
  std::string detail;
};

class TcpCanDragSession
{
public:
  explicit TcpCanDragSession(TcpCanConfig config = {});
  TcpCanResult update(
    const StreamPose & measured, const std::array<double, 3> & velocity,
    const std::array<double, 4> & orientation, std::uint64_t epoch,
    double now, const std::function<bool()> & authorized, const TcpCanIo & io);
  // Call only after the sender has quiesced and the existing stop path has
  // succeeded. Leaving mode alone is NOT evidence of physical standstill.
  bool leave_after_stop(const TcpCanIo & io);
  bool active() const {return active_;}
  bool faulted() const {return faulted_;}
  std::uint64_t batches() const {return batches_;}
  std::uint64_t points() const {return points_;}

private:
  TcpCanConfig config_;
  bool active_{false}, faulted_{false}, initialized_{false};
  bool tail_moving_{false};
  std::uint64_t epoch_{0}, batches_{0}, points_{0};
  double last_send_at_{0.0};
  StreamPose tail_;
  std::array<double, 6> tail_velocity_{};
};
}  // namespace aubo_v4_driver
