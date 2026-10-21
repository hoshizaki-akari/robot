#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>

namespace aubo_v4_driver
{
// Fresh-pose anchored Cartesian offset. In particular it never rotates an old
// accumulated lead around the measured handle when the force direction changes.
inline std::array<double, 3> aubo_drag_offset(
  const std::array<double, 3> & velocity, double horizon, double maximum_step)
{
  const double speed = std::hypot(velocity[0], velocity[1], velocity[2]);
  if (!std::isfinite(speed) || !std::isfinite(horizon) || horizon <= 0.0 ||
    !std::isfinite(maximum_step) || maximum_step <= 0.0 || speed <= 0.0) {return {};}
  const double time = std::min(horizon, maximum_step / speed);
  return {velocity[0] * time, velocity[1] * time, velocity[2] * time};
}

// Window velocity from actual samples. Callback arrival bursts must not turn a
// micrometre change divided by microseconds into a release-prediction impulse.
class CartesianVelocityWindow
{
public:
  void push(const std::array<double, 3> & position, double time)
  {
    if (!std::isfinite(time)) {samples_.clear(); return;}
    for (double value : position) {
      if (!std::isfinite(value)) {samples_.clear(); return;}
    }
    if (!samples_.empty() &&
      (time <= samples_.back().time || time - samples_.back().time > 0.25))
    {
      samples_.clear();
    }
    samples_.push_back({position, time});
    while (samples_.size() > 2 && samples_[1].time <= time - 0.08) {samples_.pop_front();}
    while (samples_.size() > 256) {samples_.pop_front();}
  }

  std::array<double, 3> velocity() const
  {
    const double span = window_s();
    if (span < 0.03 || span > 0.25) {return {};}
    std::array<double, 3> result{};
    for (unsigned i = 0; i < 3; ++i) {
      result[i] = (samples_.back().position[i] - samples_.front().position[i]) / span;
    }
    return result;
  }

  double window_s() const
  {return samples_.size() < 2 ? 0.0 : samples_.back().time - samples_.front().time;}

private:
  struct Sample {std::array<double, 3> position; double time;};
  std::deque<Sample> samples_;
};
}  // namespace aubo_v4_driver
