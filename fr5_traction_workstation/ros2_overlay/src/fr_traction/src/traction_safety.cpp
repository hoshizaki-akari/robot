#include "fr_traction/traction_safety.hpp"

#include <cmath>

namespace fr_traction
{

SafetyMonitor::SafetyMonitor(const SafetyLimits & limits)
{
  (void)limits;
}

void SafetyMonitor::set_limits(const SafetyLimits & limits)
{
  (void)limits;
  reset();
}

void SafetyMonitor::reset()
{
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
  // Force magnitude is a control/diagnostic input, never a workstation fault
  // threshold. Invalid feedback and controller-native faults still stop it.
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
