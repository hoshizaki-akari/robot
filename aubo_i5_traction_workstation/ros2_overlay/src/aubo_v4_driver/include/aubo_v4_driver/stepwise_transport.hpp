#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include "aubo_v4_driver/stepwise_scheduler.hpp"

namespace aubo_v4_driver
{
struct TransportOptions
{
  std::string host, username, password;
  int port{8899};
  double speed_mps{0.002}, acceleration_mps2{0.01};
  double maximum_joint_step_rad{0.05};
  double maximum_step_m{0.0005};
  double state_poll_interval_s{0.1};
};
struct TransportSample
{
  StepwiseFeedback feedback;
  std::array<double, 6> joints{};
  double diagnosis_at{0.0}, robot_state_at{0.0};
  bool diagnosis_ok{false};
  double sdk_speed_mps{0.0}, sdk_acceleration_mps2{0.0}, waypoint_interval_s{0.0};
  std::uint64_t motion_ack_count{0}, last_command_id{0};
  double requested_speed_mps{0.0};
  std::string error;
};
struct TransportResult
{
  std::uint64_t id{0}, epoch{0};
  bool success{false}, stop{false};
  std::string detail;
};

// One owner, separate SDK sessions for send/read/urgent-stop. Construction
// alone never creates an SDK object. Only explicitly calling connect() does.
class StepwiseTransport
{
public:
  StepwiseTransport();
  ~StepwiseTransport();
  StepwiseTransport(const StepwiseTransport &) = delete;
  StepwiseTransport & operator=(const StepwiseTransport &) = delete;
  bool connect(const TransportOptions & options, std::string & error);
  TransportSample sample() const;
  bool submit(const StepwiseCommand & command, const std::array<double, 6> & seed);
  void cancel_and_stop(std::uint64_t epoch);
  std::optional<TransportResult> take_result();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
double steady_seconds();
}  // namespace aubo_v4_driver
