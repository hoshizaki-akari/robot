#ifndef FR_TRACTION__SOURCE_FRESHNESS_HPP_
#define FR_TRACTION__SOURCE_FRESHNESS_HPP_

#include <cmath>
#include <chrono>

namespace fr_traction
{
inline double monotonic_seconds()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

// Reject a bad timestamp without replacing the caller's last valid sample or
// renewing its monotonic lifetime. A wall-clock step starts a new ordering
// epoch, allowing fresh lower timestamps after a backward clock adjustment.
class SourceStampGuard
{
public:
  bool accept(double source, double wall, double steady, double timeout)
  {
    if (!std::isfinite(wall) || !std::isfinite(steady)) {return false;}
    if (observed_ && std::abs((wall - wall_) - (steady - steady_)) > 0.05) {
      last_source_ = 0.0;
    }
    wall_ = wall;
    steady_ = steady;
    observed_ = true;
    const double age = wall - source;
    if (!std::isfinite(source) || source <= 0.0 || source <= last_source_ ||
      !std::isfinite(timeout) || timeout <= 0.0 || age < -0.05 || age > timeout)
    {
      return false;
    }
    last_source_ = source;
    return true;
  }

private:
  bool observed_{false};
  double wall_{0.0}, steady_{0.0}, last_source_{0.0};
};
// Match the wrench transform's bounded cross-process ROS clock tolerance.
// Receipt-time freshness remains a separate monotonic-clock requirement.
inline bool source_age_fresh(double age_s, double timeout_s)
{
  return std::isfinite(age_s) && std::isfinite(timeout_s) && timeout_s > 0.0 &&
         age_s >= -0.05 && age_s <= timeout_s;
}
}  // namespace fr_traction

#endif  // FR_TRACTION__SOURCE_FRESHNESS_HPP_
