#include "aubo_v4_driver/tcp_can_drag.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include "aubo_v4_driver/state_validation.hpp"
#include "aubo_v4_driver/stepwise_scheduler.hpp"

namespace aubo_v4_driver
{
namespace
{
template<std::size_t N>
bool finite(const std::array<double, N> & values)
{
  return std::all_of(values.begin(), values.end(), [](double v) {return std::isfinite(v);});
}
double norm(const std::array<double, 3> & v) {return std::hypot(v[0], v[1], v[2]);}
double distance(const std::array<double, 3> & a, const std::array<double, 3> & b)
{return norm({a[0] - b[0], a[1] - b[1], a[2] - b[2]});}
bool valid(const StreamPose & p)
{
  PoseSample sample;
  sample.position = p.position;
  sample.orientation = p.orientation;
  sample.joints = p.joints;
  double squared = 0.0;
  for (double q : p.orientation) {
    squared += q * q;
  }
  return valid_pose_sample(sample, nullptr) && std::abs(squared - 1.0) <= 0.001;
}
}

unsigned TcpCanPacing::queued(double now) const
{
  if (!initialized_ || !std::isfinite(now) || now >= deadline_) {return 0;}
  const double remaining = std::ceil((deadline_ - now) / 0.005 - 1e-9);
  // Reject a regressed/unrepresentable caller clock in the core's existing
  // oversized-buffer check rather than narrowing an unbounded double.
  return remaining > 10 ? 11U : static_cast<unsigned>(remaining);
}

bool TcpCanPacing::accept(unsigned count, double started, double finished)
{
  if (count == 0 || count > 10 || !std::isfinite(started) || !std::isfinite(finished) ||
    finished < started || (initialized_ && finished >= deadline_)) {return false;}
  // Start, not ACK time: otherwise each 5 ms SDK reply would silently add
  // another 5 ms of imagined reserve and eventually cause an actual underrun.
  const double next = (initialized_ ? deadline_ : started) + count * 0.005;
  if (finished >= next) {return false;}
  deadline_ = next;
  initialized_ = true;
  return true;
}

TcpCanDragSession::TcpCanDragSession(TcpCanConfig config)
: config_(config)
{
  if (!std::isfinite(config.period_s) || std::abs(config.period_s - 0.005) > 1e-12 ||
    config.low_water_points == 0 || config.low_water_points >= config.high_water_points ||
    config.high_water_points > 12 || !std::isfinite(config.joint_speed_radps) ||
    config.joint_speed_radps <= 0 || config.joint_speed_radps > 1.0 ||
    !std::isfinite(config.joint_acceleration_radps2) ||
    config.joint_acceleration_radps2 <= 0 || config.joint_acceleration_radps2 > 5.0 ||
    !std::isfinite(config.maximum_speed_mps) || config.maximum_speed_mps <= 0 ||
    config.maximum_speed_mps > 0.2 || !std::isfinite(config.maximum_lead_m) ||
    config.maximum_lead_m <= 0 || config.maximum_lead_m > 0.02 ||
    !std::isfinite(config.maximum_ik_step_rad) || config.maximum_ik_step_rad <= 0 ||
    config.maximum_ik_step_rad > 0.1 ||
    !std::isfinite(config.orientation_tolerance_rad) ||
    config.orientation_tolerance_rad <= 0 || config.orientation_tolerance_rad > 0.1)
  {
    throw std::invalid_argument("invalid experimental TCP2CAN drag configuration");
  }
}

TcpCanResult TcpCanDragSession::update(
  const StreamPose & measured, const std::array<double, 3> & velocity,
  const std::array<double, 4> & orientation, std::uint64_t epoch,
  double now, const std::function<bool()> & authorized, const TcpCanIo & io)
{
  TcpCanResult result;
  const auto fail = [this, &result](const std::string & reason) {
      faulted_ = true;
      result.detail = reason;
      return result;
    };
  try {
    if (faulted_) {return fail("TCP2CAN latched fault; stop and recover before another stream");}
    if (!io.enter || !io.leave || !io.queued_points || !io.inverse || !io.forward || !io.send ||
      !authorized || !authorized() || !valid(measured) || !finite(velocity) ||
      !finite(orientation) || !std::isfinite(now) ||
      norm(velocity) > config_.maximum_speed_mps + 1e-9)
    {
      return fail("invalid or revoked TCP2CAN input");
    }
    StreamPose orientation_check = measured;
    orientation_check.orientation = orientation;
    if (!valid(orientation_check)) {return fail("invalid stream orientation");}
    if (active_ && epoch != epoch_) {return fail("TCP2CAN epoch changed without confirmed stop");}
    if (!active_) {
      StreamPose checked;
      if (!io.forward(measured.joints, checked) || !valid(checked) ||
        distance(checked.position, measured.position) > 1e-5 ||
        quaternion_distance_rad(checked.orientation, measured.orientation) > 0.001)
      {
        return fail("TCP2CAN entry measured pose/FK mismatch");
      }
      // A timeout can mean the mode changed but its reply was lost. Retain
      // ownership uncertainty until the stop/leave boundary resolves it.
      active_ = true;
      epoch_ = epoch;
      if (io.enter() != 0) {
        return fail("controller rejected/ambiguous TCP2CAN mode; no automatic fallback");
      }
    }
    unsigned queued = 0;
    if (!io.queued_points(queued)) {return fail("TCP2CAN buffer query failed");}
    result.queued_points = queued;
    if (!authorized()) {return fail("TCP2CAN revoked during buffer query");}
    if (queued > config_.high_water_points) {return fail("unexpected TCP2CAN buffered trajectory");}
    if (!initialized_) {
      if (queued != 0) {return fail("TCP2CAN entry buffer was not empty");}
      tail_ = measured;
      tail_velocity_ = {};
      initialized_ = true;
    } else if (now < last_send_at_) {
      return fail("TCP2CAN monotonic clock regressed");
    } else if (queued == 0 && tail_moving_) {
      // Never hide an underrun by refilling a trajectory with an old moving tail.
      return fail("TCP2CAN moving buffer underrun; stop required");
    } else if (queued == 0) {
      tail_ = measured;
      tail_velocity_ = {};
    }
    if (queued > config_.low_water_points) {
      result.success = true;
      result.detail = "TCP2CAN existing short buffer retained; no extra points sent";
      return result;
    }
    if (norm(velocity) <= 1e-12 &&
      std::all_of(
        tail_velocity_.begin(), tail_velocity_.end(), [](double v) {
          return std::abs(v) <= 1e-8;
        }))
    {
      result.success = true;
      result.detail = "TCP2CAN zero-velocity tail draining; no stationary refill";
      return result;
    }
    auto next = tail_;
    auto speed = tail_velocity_;
    std::vector<std::array<double, 6>> batch;
    const double dt = config_.period_s;
    for (unsigned n = queued; n < config_.high_water_points; ++n) {
      if (!authorized()) {return fail("TCP2CAN revoked while planning");}
      StreamPose request = next, solved;
      request.orientation = orientation;
      for (unsigned i = 0; i < 3; ++i) {
        request.position[i] += velocity[i] * dt;
      }
      if (!io.inverse(request, solved) || !valid(solved) ||
        distance(solved.position, request.position) > 1e-5 ||
        quaternion_distance_rad(solved.orientation, request.orientation) > 0.001)
      {
        return fail("TCP2CAN IK/FK request verification failed");
      }
      std::array<double, 6> desired{}, change{}, joints{};
      double speed_scale = 1.0, acceleration_scale = 1.0;
      for (unsigned j = 0; j < 6; ++j) {
        if (std::abs(solved.joints[j] - next.joints[j]) > config_.maximum_ik_step_rad) {
          return fail("TCP2CAN IK branch discontinuity");
        }
        desired[j] = (solved.joints[j] - next.joints[j]) / dt;
        speed_scale =
          std::min(speed_scale, config_.joint_speed_radps / std::max(1e-12, std::abs(desired[j])));
      }
      for (unsigned j = 0; j < 6; ++j) {
        change[j] = desired[j] * speed_scale - speed[j];
        acceleration_scale = std::min(
          acceleration_scale,
          config_.joint_acceleration_radps2 * dt / std::max(1e-12, std::abs(change[j])));
      }
      for (unsigned j = 0; j < 6; ++j) {
        speed[j] += change[j] * acceleration_scale;
        joints[j] = next.joints[j] + speed[j] * dt;
      }
      StreamPose advanced;
      if (!io.forward(joints, advanced) || !valid(advanced) ||
        advanced.joints != joints ||
        distance(advanced.position, next.position) > config_.maximum_speed_mps * dt + 1e-8 ||
        distance(advanced.position, measured.position) > config_.maximum_lead_m + 1e-9 ||
        quaternion_distance_rad(
          advanced.orientation,
          orientation) > config_.orientation_tolerance_rad)
      {
        return fail("TCP2CAN continuous joint step/Cartesian tracking verification failed");
      }
      batch.push_back(joints);
      next = advanced;
    }
    if (!authorized()) {return fail("TCP2CAN revoked before send");}
    const int code = io.send(batch);
    if (code != 0) {
      if (code == -7101 || code == -7102) {
        return fail("TCP2CAN feeder missed timing deadline; stop required, no catch-up resend");
      }
      return fail("TCP2CAN send result " + std::to_string(code) + "; batch will not be resent");
    }
    // Commit the tail only after SDK acceptance. An ambiguous timeout is latched,
    // never retried: the controller may already own some/all of those points.
    tail_ = next;
    tail_velocity_ = speed;
    tail_moving_ =
      std::any_of(speed.begin(), speed.end(), [](double v) {return std::abs(v) > 1e-8;});
    last_send_at_ = now;
    ++batches_;
    points_ += batch.size();
    result.success = true;
    result.points_sent = static_cast<unsigned>(batch.size());
    result.detail = "TCP2CAN timed batch accepted; not a physical arrival acknowledgement";
    return result;
  } catch (...) {
    return fail("TCP2CAN SDK/callback exception; uncertain batch will not be resent");
  }
}

bool TcpCanDragSession::leave_after_stop(const TcpCanIo & io)
{
  try {
    unsigned queued = 0;
    if (active_ && (!io.queued_points || !io.queued_points(queued) || queued != 0)) {
      faulted_ = true; return false;
    }
    if (active_ && (!io.leave || io.leave() != 0)) {faulted_ = true; return false;}
    active_ = faulted_ = initialized_ = tail_moving_ = false;
    tail_velocity_ = {};
    last_send_at_ = 0;
    return true;
  } catch (...) {
    faulted_ = true;
    return false;
  }
}
}  // namespace aubo_v4_driver
