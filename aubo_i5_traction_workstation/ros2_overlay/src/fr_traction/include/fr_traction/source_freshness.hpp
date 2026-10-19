#ifndef FR_TRACTION__SOURCE_FRESHNESS_HPP_
#define FR_TRACTION__SOURCE_FRESHNESS_HPP_

#include <cmath>

namespace fr_traction
{
// Match the wrench transform's bounded cross-process ROS clock tolerance.
// Receipt-time freshness remains a separate monotonic-clock requirement.
inline bool source_age_fresh(double age_s, double timeout_s)
{
  return std::isfinite(age_s) && std::isfinite(timeout_s) && timeout_s > 0.0 &&
         age_s >= -0.05 && age_s <= timeout_s;
}
}  // namespace fr_traction

#endif  // FR_TRACTION__SOURCE_FRESHNESS_HPP_
