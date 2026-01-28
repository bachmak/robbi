#pragma once

#include "robot/motor.h"
#include "robot/settings.h"
#include "utils/geometry.h"

namespace robot {

class Robot {
public:
  explicit Robot(const RobotSettings &settings);

public:
  void update(Us dt);

  void set_target_speed(const utils::geometry::Twist &twist);
  void set_target_distance(Meter distance, Us duration);
  void set_target_rotation(Degree rotation, Us duration);
  void set_stop(bool value);

  void configure(std::string_view setting, float value);

private:
  RobotSettings settings_;
  robot::Motor left_;
  robot::Motor right_;
};
} // namespace robot