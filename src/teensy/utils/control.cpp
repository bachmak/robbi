#include "utils/control.h"

#include "utils/io.h"

#include <algorithm>
#include <utility>

namespace utils::control {
namespace {

float get_trajectory_value(const Trajectory &trajectory, const float t) {
  if (t <= trajectory[0].time) {
    return trajectory[0].value;
  }
  for (size_t i = 1; i < trajectory.size(); i++) {
    if (t <= trajectory[i].time) {
      const auto &p0 = trajectory[i - 1];
      const auto &p1 = trajectory[i];
      const auto alpha = (t - p0.time) / (p1.time - p0.time);
      return p0.value + alpha * (p1.value - p0.value);
    }
  }
  return trajectory.back().value;
}
} // namespace

Ramp::Ramp(float rise_rate, float fall_rate) : rise_rate_(rise_rate), fall_rate_(fall_rate) {}

float Ramp::update(float target, float dt) {
  auto diff = target - value_;

  if (diff > 0.0f) {
    float max_step = rise_rate_ * dt;
    diff = std::min(diff, max_step);
  } else {
    float max_step = fall_rate_ * dt;
    diff = std::max(diff, -max_step);
  }

  value_ += diff;
  return value_;
}

void Ramp::reset(float v) { value_ = v; }

float Ramp::value() const { return value_; }

void Ramp::configure(std::string_view setting, float value) {
  if (setting == "rise-rate") {
    rise_rate_ = value;
  } else if (setting == "fall-rate") {
    fall_rate_ = value;
  } else {
    io::error("Ramp: unknown setting: %s", setting.data());
  }
}

TrajectoryFollower::TrajectoryFollower(const Trajectory &trajectory) : trajectory_(trajectory) {}

void TrajectoryFollower::update(const float dt) {
  t_ += dt;
  value_ = get_trajectory_value(trajectory_, t_);
  value_integrated_ += value_ * dt;
}
} // namespace utils::control