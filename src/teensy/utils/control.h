#pragma once

#include "types.h"

#include <array>
#include <string_view>

namespace utils::control {

class Ramp {
public:
  Ramp(float rise_rate, float fall_rate);
  float update(float target, float dt);

  void reset(float v);
  float value() const;

  void configure(std::string_view setting, float value);

private:
  float rise_rate_; // units per second
  float fall_rate_; // units per second
  float value_{0.0f};
};

struct TrajectoryPoint {
  float time;
  float value;
};

using Trajectory = std::array<TrajectoryPoint, 4>;

class TrajectoryFollower {
public:
  explicit TrajectoryFollower(const Trajectory &trajectory);

  void update(float dt);

  float value() const { return value_; }
  float value_integrated() const { return value_integrated_; }
  const Trajectory &trajectory() const { return trajectory_; }

private:
  Trajectory trajectory_;
  float t_{0.0f};
  float value_{0.0f};
  float value_integrated_{0.0f};
};
} // namespace utils::control
