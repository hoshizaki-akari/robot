#include "aubo_v4_driver/stepwise_scheduler.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace aubo_v4_driver
{
namespace
{
constexpr double kQuaternionNormTolerance = 1.0e-3;
constexpr unsigned int kStopConfirmationFrames = 3U;

bool finite(double value)
{
  return std::isfinite(value);
}

template<std::size_t N>
bool finite_array(const std::array<double, N> & values)
{
  for (const double value : values) {
    if (!finite(value)) {
      return false;
    }
  }
  return true;
}

double norm3(const std::array<double, 3> & vector)
{
  return std::hypot(vector[0], vector[1], vector[2]);
}

double distance3(const std::array<double, 3> & left, const std::array<double, 3> & right)
{
  std::array<double, 3> difference{};
  for (std::size_t index = 0; index < difference.size(); ++index) {
    difference[index] = left[index] - right[index];
  }
  return norm3(difference);
}

void validate_config(const StepwiseConfig & config)
{
  const std::array<double, 10> values{
    config.maximum_step_m, config.maximum_speed_mps, config.horizon_s,
    config.feedback_timeout_s, config.command_timeout_s, config.ack_timeout_s,
    config.completion_timeout_s, config.settling_time_s, config.arrival_tolerance_m,
    config.quiet_speed_mps};
  if (!finite_array(values)) {
    throw std::invalid_argument("stepwise scheduler configuration must be finite");
  }
  if (config.maximum_step_m <= 0.0 || config.maximum_step_m > 0.1 ||
    config.maximum_speed_mps <= 0.0 || config.maximum_speed_mps > 1.0 ||
    config.horizon_s <= 0.0 || config.horizon_s > 10.0 ||
    config.feedback_timeout_s <= 0.0 || config.feedback_timeout_s > 60.0 ||
    config.command_timeout_s <= 0.0 || config.command_timeout_s > 60.0 ||
    config.ack_timeout_s <= 0.0 || config.ack_timeout_s > 120.0 ||
    config.completion_timeout_s <= 0.0 || config.completion_timeout_s > 120.0 ||
    config.settling_time_s < 0.0 || config.settling_time_s > 60.0 ||
    config.arrival_tolerance_m <= 0.0 ||
    config.arrival_tolerance_m >= config.maximum_step_m ||
    config.quiet_speed_mps < 0.0 || config.quiet_speed_mps > config.maximum_speed_mps ||
    config.confirmation_frames == 0U || config.confirmation_frames > 1000U ||
    !finite(config.follow_update_interval_s) || config.follow_update_interval_s < 0.01 ||
    !finite(config.follow_horizon_s) || config.follow_horizon_s < config.follow_update_interval_s ||
    config.follow_horizon_s > 0.5)
  {
    throw std::invalid_argument("stepwise scheduler configuration is outside safe bounds");
  }
}
}  // namespace

StepwiseScheduler::StepwiseScheduler(const StepwiseConfig & config)
: config_(config)
{
  validate_config(config_);
}

bool StepwiseScheduler::accept_time(double now)
{
  if (!finite(now) || (have_now_ && now < last_now_)) {
    fail_closed("non-finite or backward clock", now);
    return false;
  }
  last_now_ = now;
  have_now_ = true;
  return true;
}

bool StepwiseScheduler::feedback_is_fresh(double now) const
{
  return feedback_.has_value() && finite(now) && feedback_->received_at <= now &&
         now - feedback_->received_at <= config_.feedback_timeout_s;
}

bool StepwiseScheduler::feedback_is_healthy_and_quiet(double now) const
{
  return feedback_is_fresh(now) && feedback_->healthy && feedback_->stopped &&
         feedback_->speed_mps <= config_.quiet_speed_mps;
}

bool StepwiseScheduler::velocity_is_current(double now) const
{
  return velocity_.has_value() && finite(now) && now >= velocity_received_at_ &&
         now - velocity_received_at_ <= config_.command_timeout_s;
}

bool StepwiseScheduler::velocity_is_zero() const
{
  return !velocity_.has_value() || norm3(*velocity_) <= std::numeric_limits<double>::epsilon();
}

bool StepwiseScheduler::target_reached() const
{
  return pending_.has_value() && feedback_.has_value() &&
         distance3(feedback_->position, pending_->target) <= config_.arrival_tolerance_m;
}

void StepwiseScheduler::clear_motion_cache()
{
  pending_.reset();
  velocity_.reset();
  arrival_frames_ = 0U;
  have_quiet_since_ = false;
}

void StepwiseScheduler::begin_stop(double now, const std::string & reason, bool fault)
{
  if (fault && fault_reason_.empty()) {fault_reason_ = reason;}
  if (state_ == StepwiseState::STOPPING) {
    const bool fault_escalated = fault && !stop_fault_;
    stop_fault_ = stop_fault_ || fault;
    if (fault_escalated) {
      reason_ = reason;
    }
    return;
  }
  ++epoch_;
  clear_motion_cache();
  stop_fault_ = fault;
  stop_request_taken_ = false;
  stop_acknowledged_ = false;
  stop_frames_ = 0U;
  stop_acknowledged_sequence_ = have_feedback_sequence_ ? last_feedback_sequence_ : 0U;
  reason_ = reason;
  if (state_ == StepwiseState::DISABLED && !fault) {
    state_ = StepwiseState::DISABLED;
    return;
  }
  if (state_ == StepwiseState::DISABLED && fault) {
    state_ = StepwiseState::FAULT;
    return;
  }
  state_ = StepwiseState::STOPPING;
  stop_deadline_ = finite(now) ? now + config_.ack_timeout_s : last_now_;
}

void StepwiseScheduler::fail_closed(const std::string & reason, double now)
{
  if (state_ == StepwiseState::FAULT) {
    return;
  }
  const double safe_now = finite(now) && (!have_now_ || now >= last_now_) ? now : last_now_;
  begin_stop(safe_now, reason, true);
}

bool StepwiseScheduler::set_enabled(bool enabled, double now)
{
  if (!accept_time(now)) {
    return false;
  }
  poll(now);
  if (enabled) {
    if (state_ != StepwiseState::DISABLED || !feedback_is_healthy_and_quiet(now)) {
      return false;
    }
    state_ = StepwiseState::READY;
    activation_orientation_ = feedback_->orientation;
    reason_ = "enabled; waiting for a new velocity";
    activation_velocity_generation_ = velocity_generation_;
    follow_progress_at_ = now;
    follow_progress_position_ = feedback_->position;
    last_follow_feedback_sequence_ = 0;
    return true;
  }
  if (state_ == StepwiseState::FAULT) {
    return false;
  }
  // READY means any previous endpoint has already passed fresh arrival and
  // settling confirmation. No dispatch is in flight. Revoke intent locally
  // rather than issuing redundant vendor FastStop calls on an idle robot.
  if (state_ == StepwiseState::READY && !pending_ && feedback_is_healthy_and_quiet(now)) {
    ++epoch_;
    clear_motion_cache();
    state_ = StepwiseState::DISABLED;
    reason_ = "disabled after measured idle; no pending dispatch";
    return true;
  }
  begin_stop(now, "disabled by caller", false);
  return true;
}

bool StepwiseScheduler::set_follow_mode(bool enabled, double now)
{
  if (!accept_time(now)) {return false;}
  if (follow_mode_ == enabled) {return true;}
  if ((state_ != StepwiseState::DISABLED && state_ != StepwiseState::READY) || pending_) {
    return false;
  }
  follow_mode_ = enabled;
  follow_progress_at_ = now;
  if (feedback_) {follow_progress_position_ = feedback_->position;}
  return true;
}

bool StepwiseScheduler::set_feedback(const StepwiseFeedback & feedback, double now)
{
  if (!accept_time(now)) {
    return false;
  }
  // Check all active deadlines before a newly received frame can influence a
  // completion or stop-confirmation transition.
  poll(now);
  const double quaternion_norm = std::sqrt(
    feedback.orientation[0] * feedback.orientation[0] +
    feedback.orientation[1] * feedback.orientation[1] +
    feedback.orientation[2] * feedback.orientation[2] +
    feedback.orientation[3] * feedback.orientation[3]);
  const bool valid = finite_array(feedback.position) && finite_array(feedback.orientation) &&
    finite(feedback.received_at) && finite(feedback.speed_mps) && feedback.received_at <= now &&
    feedback.speed_mps >= 0.0 && finite(quaternion_norm) &&
    std::abs(quaternion_norm - 1.0) <= kQuaternionNormTolerance &&
    (!have_feedback_timestamp_ || feedback.received_at >= last_feedback_timestamp_);
  if (!valid) {
    fail_closed("invalid feedback", now);
    return false;
  }
  if (have_feedback_sequence_ && feedback.sequence == last_feedback_sequence_) {
    // A repeated waypoint sequence can carry newer independent diagnosis or
    // robot-state flags.  It must never make the retained pose fresher or
    // advance confirmation frames, but it may make the cached safety view
    // more conservative.
    if (feedback_.has_value()) {
      feedback_->healthy = feedback_->healthy && feedback.healthy;
      feedback_->stopped = feedback_->stopped && feedback.stopped;
    }
    if (!feedback.healthy && state_ != StepwiseState::DISABLED &&
      state_ != StepwiseState::FAULT)
    {
      begin_stop(now, "unhealthy duplicate feedback", true);
    }
    return true;
  }
  if (have_feedback_sequence_ && feedback.sequence < last_feedback_sequence_) {
    fail_closed("stale feedback sequence", now);
    return false;
  }
  feedback_ = feedback;
  last_feedback_timestamp_ = feedback.received_at;
  have_feedback_timestamp_ = true;
  last_feedback_sequence_ = feedback.sequence;
  have_feedback_sequence_ = true;
  if (distance3(feedback.position, follow_progress_position_) >= config_.arrival_tolerance_m) {
    follow_progress_position_ = feedback.position;
    follow_progress_at_ = feedback.received_at;
  }

  if ((state_ == StepwiseState::SENDING || state_ == StepwiseState::MOVING ||
    state_ == StepwiseState::SETTLING) && !feedback.healthy)
  {
    begin_stop(now, "unhealthy feedback during motion", true);
    return true;
  }
  if ((!follow_mode_ || (pending_ && pending_->hold_position)) &&
    (state_ == StepwiseState::MOVING || state_ == StepwiseState::SETTLING))
  {
    evaluate_arrival(feedback);
  } else if (state_ == StepwiseState::STOPPING) {
    evaluate_stop_confirmation(feedback);
  }
  return true;
}

bool StepwiseScheduler::set_velocity(const std::array<double, 3> & velocity, double now)
{
  if (!accept_time(now)) {
    return false;
  }
  poll(now);
  if (!finite_array(velocity) || !finite(norm3(velocity))) {
    if (state_ != StepwiseState::DISABLED && state_ != StepwiseState::FAULT) {
      begin_stop(now, "invalid velocity", true);
    }
    return false;
  }
  if (state_ == StepwiseState::FAULT || state_ == StepwiseState::STOPPING) {
    return false;
  }
  const bool was_zero = velocity_is_zero();
  velocity_ = velocity;
  velocity_received_at_ = now;
  ++velocity_generation_;
  if ((state_ == StepwiseState::SENDING || state_ == StepwiseState::MOVING ||
    state_ == StepwiseState::SETTLING) && velocity_is_zero() &&
    !(follow_mode_ && config_.follow_hold_enabled))
  {
    begin_stop(now, "zero velocity during motion", false);
  }
  if (follow_mode_ && was_zero && !velocity_is_zero()) {
    follow_progress_at_ = now;
    if (feedback_) {follow_progress_position_ = feedback_->position;}
  }
  return true;
}

std::optional<StepwiseCommand> StepwiseScheduler::take_command(double now)
{
  if (!accept_time(now)) {
    return std::nullopt;
  }
  poll(now);
  const bool updating_follow = follow_mode_ &&
    (state_ == StepwiseState::MOVING || state_ == StepwiseState::SETTLING) &&
    now - last_follow_dispatch_at_ >= config_.follow_update_interval_s &&
    feedback_is_fresh(now) && feedback_->healthy &&
    feedback_->sequence > last_follow_feedback_sequence_;
  if (!updating_follow && (state_ != StepwiseState::READY || pending_.has_value())) {
    return std::nullopt;
  }
  if (!velocity_is_current(now)) {
    velocity_.reset();
    return std::nullopt;
  }
  const bool holding = follow_mode_ && config_.follow_hold_enabled && velocity_is_zero();
  if ((holding && (!updating_follow || (pending_ && pending_->hold_position))) ||
    (!holding && velocity_is_zero()) || velocity_generation_ <= activation_velocity_generation_ ||
    (!updating_follow && !feedback_is_healthy_and_quiet(now)))
  {
    return std::nullopt;
  }
  std::array<double, 3> displacement = *velocity_;
  const double requested_speed = holding ? config_.maximum_speed_mps : norm3(displacement);
  const double horizon = follow_mode_ ?
    std::min(config_.horizon_s, config_.follow_horizon_s) : config_.horizon_s;
  const double speed_scale = std::min(1.0, config_.maximum_speed_mps / requested_speed);
  if (follow_mode_ && config_.follow_reference_enabled && !holding) {
    double projection = 0.0;
    for (unsigned i = 0; i < 3; ++i) {
      projection += last_follow_velocity_[i] * displacement[i];
    }
    const bool continuing = updating_follow && pending_ && !pending_->hold_position &&
      projection > 0.5 * norm3(last_follow_velocity_) * requested_speed;
    // With a fixed SDK joint profile, velocity is encoded by a time-paced
    // reference. Do not add a full lookahead to the measured pose every tick:
    // that would multiply low-force speed by horizon/update_interval.
    const double dt = continuing ? std::min(
      now - last_follow_dispatch_at_,
      config_.command_timeout_s) : config_.follow_update_interval_s;
    for (unsigned i = 0; i < 3; ++i) {
      displacement[i] = (continuing ? pending_->target[i] - feedback_->position[i] : 0.0) +
        (*velocity_)[i] * speed_scale * dt;
    }
    double forward = 0.0;
    for (unsigned i = 0; i < 3; ++i) {
      forward += displacement[i] * (*velocity_)[i];
    }
    if (forward <= 0.0) {
      // If the measured arm overtook the reference, never pull it backwards
      // toward an obsolete endpoint while the operator still pushes forward.
      for (unsigned i = 0; i < 3; ++i) {
        displacement[i] = (*velocity_)[i] * speed_scale * config_.follow_update_interval_s;
      }
    }
    const double lead = norm3(displacement);
    const double maximum_lead = std::min(
      config_.maximum_step_m,
      requested_speed * speed_scale * horizon);
    if (lead > maximum_lead) {
      for (double & component : displacement) {
        component *= maximum_lead / lead;
      }
    }
  } else {
    const double step_scale = std::min(
      1.0, config_.maximum_step_m / (requested_speed * speed_scale * horizon));
    for (double & component : displacement) {
      component *= speed_scale * horizon * step_scale;
    }
  }
  const double displacement_norm = norm3(displacement);
  const double minimum_step = follow_mode_ ? 1e-10 : 2.0 * config_.arrival_tolerance_m;
  if (!finite(displacement_norm) || (!holding && displacement_norm <= minimum_step)) {
    reason_ = "belowminimumstep";
    return std::nullopt;
  }
  StepwiseCommand command;
  command.speed_mps = std::min(requested_speed, config_.maximum_speed_mps);
  command.follow_joint_mode = follow_mode_;
  command.hold_position = holding;
  command.id = next_command_id_++;
  command.epoch = epoch_;
  command.start = feedback_->position;
  command.orientation = holding ? feedback_->orientation : activation_orientation_;
  for (std::size_t index = 0; index < command.target.size(); ++index) {
    command.target[index] = command.start[index] + displacement[index];
  }
  if (command.target == command.start && !holding) {
    reason_ = "belowminimumstep";
    return std::nullopt;
  }
  if (follow_mode_ && !updating_follow) {
    follow_progress_at_ = now;
    follow_progress_position_ = feedback_->position;
  }
  pending_ = command;
  last_follow_velocity_ = *velocity_;
  last_follow_dispatch_at_ = now;
  last_follow_feedback_sequence_ = feedback_->sequence;
  ack_deadline_ = now + config_.ack_timeout_s;
  completion_deadline_ = now + config_.completion_timeout_s;
  state_ = StepwiseState::SENDING;
  reason_ = "command awaiting acknowledgement";
  return command;
}

void StepwiseScheduler::acknowledge(
  std::uint64_t id, std::uint64_t epoch, bool success, double now)
{
  if (!accept_time(now)) {
    return;
  }
  poll(now);
  if (state_ != StepwiseState::SENDING || !pending_.has_value() || pending_->id != id ||
    pending_->epoch != epoch)
  {
    return;
  }
  if (!success) {
    begin_stop(now, "command acknowledgement failed", true);
    return;
  }
  state_ = StepwiseState::MOVING;
  reason_ = "command acknowledged";
  acknowledged_at_ = now;
  acknowledged_sequence_ = have_feedback_sequence_ ? last_feedback_sequence_ : 0U;
  arrival_frames_ = 0U;
  have_quiet_since_ = false;
}

void StepwiseScheduler::evaluate_arrival(const StepwiseFeedback & feedback)
{
  if (!feedback_is_fresh(last_now_) || !pending_.has_value() || !feedback.healthy ||
    !feedback.stopped ||
    feedback.speed_mps > config_.quiet_speed_mps ||
    (!pending_->hold_position && !target_reached()) ||
    feedback.sequence <= acknowledged_sequence_ || feedback.received_at <= acknowledged_at_)
  {
    arrival_frames_ = 0U;
    have_quiet_since_ = false;
    if (state_ == StepwiseState::SETTLING) {
      state_ = StepwiseState::MOVING;
      reason_ = "arrival confirmation interrupted";
    }
    return;
  }
  ++arrival_frames_;
  if (!have_quiet_since_) {
    quiet_since_ = feedback.received_at;
    have_quiet_since_ = true;
  }
  if (arrival_frames_ >= config_.confirmation_frames) {
    state_ = StepwiseState::SETTLING;
    reason_ = "arrival confirmed; settling";
  }
}

void StepwiseScheduler::request_stop(double now, const std::string & reason, bool fault)
{
  if (!accept_time(now)) {
    return;
  }
  if (state_ == StepwiseState::FAULT) {
    return;
  }
  begin_stop(now, reason.empty() ? "stop requested" : reason, fault);
}

std::optional<std::uint64_t> StepwiseScheduler::take_stop_request()
{
  if (state_ != StepwiseState::STOPPING || stop_request_taken_) {
    return std::nullopt;
  }
  stop_request_taken_ = true;
  return epoch_;
}

void StepwiseScheduler::acknowledge_stop(std::uint64_t epoch, bool success, double now)
{
  if (!accept_time(now)) {
    return;
  }
  poll(now);
  if (state_ != StepwiseState::STOPPING || epoch != epoch_) {
    return;
  }
  if (!success) {
    state_ = StepwiseState::FAULT;
    reason_ = "stop acknowledgement failed";
    return;
  }
  stop_acknowledged_ = true;
  stop_acknowledged_at_ = now;
  stop_acknowledged_sequence_ = have_feedback_sequence_ ? last_feedback_sequence_ : 0U;
  stop_frames_ = 0U;
  reason_ = "stop acknowledged; awaiting stopped feedback";
}

void StepwiseScheduler::evaluate_stop_confirmation(const StepwiseFeedback & feedback)
{
  if (!feedback_is_fresh(last_now_) || !stop_acknowledged_ ||
    feedback.sequence <= stop_acknowledged_sequence_ ||
    feedback.received_at <= stop_acknowledged_at_ || !feedback.healthy || !feedback.stopped ||
    feedback.speed_mps > config_.quiet_speed_mps)
  {
    stop_frames_ = 0U;
    return;
  }
  ++stop_frames_;
  if (stop_frames_ >= kStopConfirmationFrames) {
    if (stop_fault_) {
      state_ = StepwiseState::FAULT;
      reason_ = "fault stop confirmed";
    } else {
      state_ = StepwiseState::DISABLED;
      reason_ = "stop confirmed";
    }
  }
}

void StepwiseScheduler::poll(double now)
{
  if (!accept_time(now)) {
    return;
  }
  if (state_ == StepwiseState::FAULT || state_ == StepwiseState::DISABLED) {
    return;
  }
  if (state_ == StepwiseState::STOPPING) {
    if (now >= stop_deadline_) {
      state_ = StepwiseState::FAULT;
      reason_ = "stop acknowledgement timeout";
    }
    return;
  }
  if (state_ == StepwiseState::READY) {
    if (!velocity_is_current(now)) {
      velocity_.reset();
    }
    return;
  }
  if (!feedback_is_fresh(now) || !feedback_->healthy) {
    begin_stop(now, "feedback unavailable during motion", true);
    return;
  }
  if (!velocity_is_current(now) ||
    (velocity_is_zero() && !(follow_mode_ && config_.follow_hold_enabled)))
  {
    begin_stop(now, "velocity expired during motion", false);
    return;
  }
  if (follow_mode_ && !velocity_is_zero() && now - follow_progress_at_ >= 2.0) {
    begin_stop(now, "follow target acknowledged but no measured progress", true);
    return;
  }
  if (state_ == StepwiseState::SENDING && now >= ack_deadline_) {
    begin_stop(now, "command acknowledgement timeout", true);
    return;
  }
  if ((state_ == StepwiseState::SENDING || state_ == StepwiseState::MOVING ||
    state_ == StepwiseState::SETTLING) &&
    now >= completion_deadline_)
  {
    begin_stop(now, "command completion timeout", true);
    return;
  }
  if (state_ == StepwiseState::SETTLING && have_quiet_since_ &&
    now - quiet_since_ >= config_.settling_time_s)
  {
    pending_.reset();
    arrival_frames_ = 0U;
    have_quiet_since_ = false;
    state_ = StepwiseState::READY;
    reason_ = "step complete";
  }
}

StepwiseState StepwiseScheduler::state() const
{
  return state_;
}

bool StepwiseScheduler::fault_pending() const
{
  return state_ == StepwiseState::FAULT ||
         (state_ == StepwiseState::STOPPING && stop_fault_);
}

bool StepwiseScheduler::motion_idle(double now) const
{
  return (state_ == StepwiseState::READY || state_ == StepwiseState::DISABLED) &&
         !pending_.has_value() && feedback_is_healthy_and_quiet(now);
}

bool StepwiseScheduler::healthy(double now) const
{
  return state_ != StepwiseState::FAULT && state_ != StepwiseState::STOPPING &&
         feedback_is_fresh(now) && feedback_->healthy;
}

const std::string & StepwiseScheduler::reason() const
{
  return reason_;
}

const std::string & StepwiseScheduler::fault_reason() const
{
  return fault_reason_;
}

std::uint64_t StepwiseScheduler::epoch() const
{
  return epoch_;
}

}  // namespace aubo_v4_driver
