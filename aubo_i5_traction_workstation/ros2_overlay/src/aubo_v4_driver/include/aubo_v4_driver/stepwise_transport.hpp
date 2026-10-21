#pragma once

#include <array>
#include <cstddef>
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
  // Slow force regulation keeps the original profile. Faster drag opts in.
  double fast_acceleration_mps2{0.01};
  double maximum_joint_step_rad{0.05};
  double maximum_step_m{0.0005};
  double state_poll_interval_s{0.1};
  bool allow_follow_mode{false};
  bool fixed_follow_profile{false};
  bool single_stop_when_quiescent{false};
  bool subdivide_follow_joint_steps{false};
  bool brake_on_hold{false};
  bool adaptive_follow_ik{false};
  double maximum_drag_step_m{0.005};
  double follow_joint_speed_radps{0.2}, follow_joint_acceleration_radps2{0.5};
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
  int diagnosis_result{-1}, state_result{-1};
  std::uint64_t diagnosis_read_timeouts{0}, state_read_timeouts{0};
  int last_ik_result{-1};
  double last_ik_max_joint_delta_rad{0.0};
  double proposed_ik_max_joint_delta_rad{0.0};
  std::uint64_t follow_joint_subdivisions{0};
  double accepted_ik_scale{1.0}, accepted_cartesian_step_m{0.0};
  int adaptive_ik_attempts{0}, fk_result{-1};
  bool kinematics_verified{false};
  double fk_position_error_m{0.0};
  std::size_t last_ik_joint_index{0};
  std::string last_ik_failure;
  double query_duration_s{0.0};
  double ik_duration_s{0.0}, motion_call_duration_s{0.0};
  double dispatch_interval_s{0.0}, maximum_dispatch_interval_s{0.0};
  double maximum_ik_duration_s{0.0}, maximum_motion_call_duration_s{0.0};
  std::string last_telemetry_failure;
  bool follow_joint_mode{false};
  std::uint64_t follow_profile_updates{0};
  double follow_joint_speed_max_radps{0.0}, follow_joint_acceleration_max_radps2{0.0};
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
  bool quiescent() const;
  bool submit(const StepwiseCommand & command, const std::array<double, 6> & seed);
  void cancel_and_stop(std::uint64_t epoch);
  std::optional<TransportResult> take_result();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
double steady_seconds();
}  // namespace aubo_v4_driver
