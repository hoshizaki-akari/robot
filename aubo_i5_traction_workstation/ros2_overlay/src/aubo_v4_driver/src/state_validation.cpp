#include "aubo_v4_driver/state_validation.hpp"
#include <cmath>
namespace aubo_v4_driver
{namespace
{bool finite(double v) {return std::isfinite(v);}
void set_reason(std::string * r, const std::string & v) {if (r != nullptr) {*r = v;}}
}
bool valid_pose_sample(const PoseSample & sample, std::string * reason)
{
  for (double v : sample.position) {
    if (!finite(v)) {
      set_reason(reason, "non-finite Cartesian position from V4 SDK"); return false;
    }
  }
  for (double v : sample.orientation) {
    if (!finite(v)) {
      set_reason(reason, "non-finite orientation from V4 SDK"); return false;
    }
  }
  double norm_squared = 0.0; for (double v : sample.orientation) {
    norm_squared += v * v;
  }
  if (!finite(norm_squared) || norm_squared < 1.0e-12) {
    set_reason(reason, "invalid zero-length orientation from V4 SDK"); return false;
  }
  for (double v : sample.joints) {
    if (!finite(v)) {
      set_reason(reason, "non-finite joint position from V4 SDK"); return false;
    }
  }
  set_reason(reason, "valid"); return true;
}
bool diagnosis_is_healthy(const DiagnosisSample & s, std::string * reason)
{
  if (!s.arm_power_status) {
    set_reason(reason, "controller reports arm power is off or unavailable"); return false;
  }
  if (s.arm_canbus_status != 0U) {
    set_reason(reason, "controller reports arm CAN bus fault"); return false;
  }
  // This AUBOPE V4 firmware reports remoteHalt=1 even while accepting
  // computer-issued moves. It is not an active-stop signal on this controller.
  if (s.soft_emergency || s.remote_emergency) {
    set_reason(reason, "controller reports emergency state"); return false;
  }
  if (s.robot_collision || s.joint_collision_detect != 0U) {
    set_reason(reason, "controller reports collision state"); return false;
  }
  if (s.encoder_error_status || s.encoder_lines_error || s.joint_error_status) {
    set_reason(reason, "controller reports encoder or joint error"); return false;
  }
  if (s.singularity_overspeed_alarm || s.robot_current_alarm) {
    set_reason(reason, "controller reports motion or current alarm"); return false;
  }
  if (s.tool_io_error != 0U) {
    set_reason(reason, "controller reports tool I/O error"); return false;
  }
  set_reason(reason, "diagnosis reports no checked fault and arm power is on"); return true;
}
}  // namespace aubo_v4_driver
