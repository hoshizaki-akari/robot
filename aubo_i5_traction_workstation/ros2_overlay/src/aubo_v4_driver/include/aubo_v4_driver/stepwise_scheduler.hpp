#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>

namespace aubo_v4_driver
{

struct StepwiseConfig
{
  double maximum_step_m{0.0005};
  double maximum_speed_mps{0.002};
  double horizon_s{0.1};
  double feedback_timeout_s{0.5};
  double command_timeout_s{0.25};
  double ack_timeout_s{2.0};
  double completion_timeout_s{5.0};
  double settling_time_s{0.1};
  double arrival_tolerance_m{0.00005};
  double quiet_speed_mps{0.0002};
  unsigned int confirmation_frames{3};
};

struct StepwiseFeedback
{
  std::array<double, 3> position{};
  // Quaternion in XYZW order.
  std::array<double, 4> orientation{0.0, 0.0, 0.0, 1.0};
  double received_at{0.0};
  double speed_mps{0.0};
  std::uint64_t sequence{0};
  bool healthy{false};
  bool stopped{false};
};

struct StepwiseCommand
{
  std::uint64_t id{0};
  std::uint64_t epoch{0};
  std::array<double, 3> start{};
  std::array<double, 3> target{};
  std::array<double, 4> orientation{};
  double speed_mps{0.002};
  bool follow_joint_mode{false};
};

enum class StepwiseState
{
  DISABLED,
  READY,
  SENDING,
  MOVING,
  SETTLING,
  STOPPING,
  FAULT
};

// Single-threaded pure state machine.  The caller is responsible for executing
// returned commands and stop requests through its hardware adapter.
class StepwiseScheduler
{
public:
  explicit StepwiseScheduler(const StepwiseConfig & config = {});

  bool set_enabled(bool enabled, double now);
  bool set_follow_mode(bool enabled, double now);
  bool set_feedback(const StepwiseFeedback & feedback, double now);
  bool set_velocity(const std::array<double, 3> & velocity, double now);
  std::optional<StepwiseCommand> take_command(double now);
  void acknowledge(std::uint64_t id, std::uint64_t epoch, bool success, double now);
  void request_stop(double now, const std::string & reason, bool fault = false);
  std::optional<std::uint64_t> take_stop_request();
  void acknowledge_stop(std::uint64_t epoch, bool success, double now);
  void poll(double now);

  StepwiseState state() const;
  bool fault_pending() const;
  bool motion_idle(double now) const;
  bool healthy(double now) const;
  const std::string & reason() const;
  const std::string & fault_reason() const;
  std::uint64_t epoch() const;

private:
  bool accept_time(double now);
  bool feedback_is_fresh(double now) const;
  bool feedback_is_healthy_and_quiet(double now) const;
  bool velocity_is_current(double now) const;
  bool velocity_is_zero() const;
  bool target_reached() const;
  void evaluate_arrival(const StepwiseFeedback & feedback);
  void evaluate_stop_confirmation(const StepwiseFeedback & feedback);
  void clear_motion_cache();
  void fail_closed(const std::string & reason, double now);
  void begin_stop(double now, const std::string & reason, bool fault);

  StepwiseConfig config_;
  StepwiseState state_{StepwiseState::DISABLED};
  std::string reason_{"disabled"};
  std::string fault_reason_;
  std::array<double, 4> activation_orientation_{0.0, 0.0, 0.0, 1.0};
  std::optional<StepwiseFeedback> feedback_;
  std::optional<std::array<double, 3>> velocity_;
  std::optional<StepwiseCommand> pending_;
  double last_now_{0.0};
  bool have_now_{false};
  double last_feedback_timestamp_{0.0};
  bool have_feedback_timestamp_{false};
  std::uint64_t last_feedback_sequence_{0};
  bool have_feedback_sequence_{false};
  double velocity_received_at_{0.0};
  std::uint64_t velocity_generation_{0};
  std::uint64_t activation_velocity_generation_{0};
  std::uint64_t next_command_id_{1};
  std::uint64_t epoch_{0};
  double ack_deadline_{0.0};
  double completion_deadline_{0.0};
  double acknowledged_at_{0.0};
  std::uint64_t acknowledged_sequence_{0};
  unsigned int arrival_frames_{0};
  double quiet_since_{0.0};
  bool have_quiet_since_{false};
  bool stop_fault_{false};
  bool stop_request_taken_{false};
  bool stop_acknowledged_{false};
  double stop_deadline_{0.0};
  double stop_acknowledged_at_{0.0};
  std::uint64_t stop_acknowledged_sequence_{0};
  unsigned int stop_frames_{0};
  bool follow_mode_{false};
  double last_follow_dispatch_at_{0.0}, follow_progress_at_{0.0};
  std::array<double, 3> follow_progress_position_{};
  std::uint64_t last_follow_feedback_sequence_{0};
};

}  // namespace aubo_v4_driver
