#ifndef FR_TRACTION__TRACTION_SAFETY_HPP_
#define FR_TRACTION__TRACTION_SAFETY_HPP_

#include <string>

#include "fr_traction/traction_math.hpp"

namespace fr_traction
{

enum class SafetyFault : unsigned char
{
  NONE = 0,
  WRENCH_INVALID,
  WRENCH_TIMEOUT,
  EE_STATE_TIMEOUT,
  ROS2_CONTROL_ERROR,
  HARD_OVERFORCE,
  OVERFORCE,
  LATERAL_FORCE,
  AXIAL_TRAVEL_LIMIT,
  UI_HEARTBEAT_TIMEOUT
};

struct SafetyLimits
{
  double overforce_n = 25.0;
  double hard_overforce_n = 30.0;
  double lateral_force_n = 5.0;
  double axial_travel_m = 0.050;
  double overforce_duration_s = 0.050;
  double lateral_duration_s = 0.200;
};

struct SafetySample
{
  bool wrench_valid = false;
  bool wrench_fresh = false;
  bool ee_fresh = false;
  bool controller_healthy = false;
  bool ui_heartbeat_fresh = false;
  Vec3 raw_wrench;
  ForceMetrics metrics;
  double axis_displacement_m = 0.0;
};

class SafetyMonitor
{
public:
  explicit SafetyMonitor(const SafetyLimits & limits = {});

  void set_limits(const SafetyLimits & limits);
  void reset();
  SafetyFault update(const SafetySample & sample, double now_s, bool require_ui_heartbeat);
  static const char * code(SafetyFault fault);

private:
  SafetyLimits limits_;
  double overforce_started_s_ = -1.0;
  double lateral_started_s_ = -1.0;
};

}  // namespace fr_traction

#endif  // FR_TRACTION__TRACTION_SAFETY_HPP_
