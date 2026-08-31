#ifndef FAIRINO_HARDWARE_FINITE_CHECKS_HPP_
#define FAIRINO_HARDWARE_FINITE_CHECKS_HPP_

#include <cmath>
#include <cstddef>

namespace fairino_hardware
{

inline bool all_finite(const double * values, std::size_t count)
{
  if (count == 0) {
    return true;
  }
  if (values == nullptr) {
    return false;
  }
  for (std::size_t i = 0; i < count; ++i) {
    if (!std::isfinite(values[i])) {
      return false;
    }
  }
  return true;
}

}  // namespace fairino_hardware

#endif  // FAIRINO_HARDWARE_FINITE_CHECKS_HPP_
