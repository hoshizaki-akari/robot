#pragma once
#include <cmath>
namespace aubo_v4_driver
{
enum class StopHandoff {PENDING, ACCEPTED, REJECTED, TIMED_OUT};
inline StopHandoff stop_handoff_status(bool ready, bool accepted, double now, double deadline)
{
  if (!std::isfinite(now) || !std::isfinite(deadline) || now >= deadline) {
    return StopHandoff::TIMED_OUT;
  }
  if (!ready) {return StopHandoff::PENDING;}
  return accepted ? StopHandoff::ACCEPTED : StopHandoff::REJECTED;
}
}  // namespace aubo_v4_driver
