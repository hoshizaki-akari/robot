#include <aubo_driver/serviceinterface.h>
#include <iostream>

// Offline ABI check only: never instantiate ServiceInterface or call the SDK.
// Taking method addresses forces the linker/loader to resolve the exact APIs.
template<typename Method>
bool present(const char * name, Method method)
{
  const bool ok = method != nullptr;
  std::cout << name << '=' << (ok ? "RESOLVED" : "MISSING") << '\n';
  return ok;
}

int main()
{
  using S = ServiceInterface;
  using namespace aubo_robot_namespace;
  bool ok = true;
  ok &= present("absolute_line_move", static_cast<int (S::*)(wayPoint_S &, bool)>(
    &S::robotServiceLineMove));
  ok &= present("relative_line_move", &S::robotMoveLineToTargetPositionByRelative);
  ok &= present("line_velocity", &S::robotServiceSetGlobalMoveEndMaxLineVelc);
  ok &= present("line_acceleration", &S::robotServiceSetGlobalMoveEndMaxLineAcc);
  ok &= present("state", &S::robotServiceGetRobotCurrentState);
  ok &= present("waypoint", &S::robotServiceGetCurrentWaypointInfo);
  ok &= present("diagnosis", &S::robotServiceGetRobotDiagnosisInfo);
  ok &= present("tool", &S::robotServiceGetToolKinematicsParam);
  ok &= present("fast_stop", &S::robotMoveFastStop);
  ok &= present("leave_passthrough", &S::robotServiceLeaveTcp2CanbusMode);
  ok &= present("seeded_ik", static_cast<int (S::*)(const double *, const Pos &,
    const Ori &, wayPoint_S &)>(&S::robotServiceRobotIk));
  std::cout << "network_connections=0; sdk_calls=0\n";
  std::cout << "status=" << (ok ? "OFFLINE_ABI_PASS" : "FAIL") << '\n';
  return ok ? 0 : 1;
}
