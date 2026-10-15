#include "fr_traction/traction_safety.hpp"

#include <cmath>

namespace fr_traction
{

SafetyMonitor::SafetyMonitor(const SafetyLimits & limits)
: limits_(limits)
{
}

void SafetyMonitor::set_limits(const SafetyLimits & limits)
{
  limits_ = limits;
  reset();
}

void SafetyMonitor::reset()
{
  hard_overforce_started_at_s_ = -1.0;
}

SafetyFault SafetyMonitor::update(
  const SafetySample & sample, double now_s, bool require_ui_heartbeat)
{
  if (!sample.wrench_valid) {
    return SafetyFault::WRENCH_INVALID;
  }
  if (!sample.wrench_fresh) {
    return SafetyFault::WRENCH_TIMEOUT;
  }
  if (!sample.ee_fresh) {
    return SafetyFault::EE_STATE_TIMEOUT;
  }
  if (!sample.controller_healthy) {
    return SafetyFault::ROS2_CONTROL_ERROR;
  }
  if (!finite(sample.raw_wrench) || !std::isfinite(now_s)) {
    return SafetyFault::WRENCH_INVALID;
  }
  if (!std::isfinite(sample.metrics.actual_force_n) ||
    sample.metrics.actual_force_n < 0.0 ||
    !finite(sample.metrics.lateral_force_vector) ||
    !std::isfinite(sample.metrics.lateral_force_n) || sample.metrics.lateral_force_n < 0.0)
  {
    return SafetyFault::WRENCH_INVALID;
  }
  if (sample.metrics.actual_force_n > limits_.hard_overforce_n) {
    if (hard_overforce_started_at_s_ < 0.0) {
      hard_overforce_started_at_s_ = now_s;
    }
    if (now_s - hard_overforce_started_at_s_ >= limits_.hard_overforce_confirm_s) {
      return SafetyFault::HARD_OVERFORCE;
    }
  } else {
    hard_overforce_started_at_s_ = -1.0;
  }
  // Lateral force and software travel remain control/diagnostic inputs. The
  // only software force trip is the separately debounced 150 N hard limit;
  // native controller reach protection remains in force as well.
  if (require_ui_heartbeat && !sample.ui_heartbeat_fresh) {
    return SafetyFault::UI_HEARTBEAT_TIMEOUT;
  }
  return SafetyFault::NONE;
}

const char * SafetyMonitor::code(SafetyFault fault)
{
  switch (fault) {
    case SafetyFault::WRENCH_INVALID: return "WRENCH_INVALID";
    case SafetyFault::WRENCH_TIMEOUT: return "WRENCH_TIMEOUT";
    case SafetyFault::EE_STATE_TIMEOUT: return "EE_STATE_TIMEOUT";
    case SafetyFault::ROS2_CONTROL_ERROR: return "ROS2_CONTROL_ERROR";
    case SafetyFault::HARD_OVERFORCE: return "HARD_OVERFORCE";
    case SafetyFault::OVERFORCE: return "OVERFORCE";
    case SafetyFault::LATERAL_FORCE: return "LATERAL_FORCE";
    case SafetyFault::UI_HEARTBEAT_TIMEOUT: return "UI_HEARTBEAT_TIMEOUT";
    case SafetyFault::NONE: return "NONE";
  }
  return "UNKNOWN";
}

}  // namespace fr_traction
