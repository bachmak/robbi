#pragma once

#include "robot/settings.h"
#include "utils/control.h"
#include "utils/math.h"

namespace robot::states {

class VelocityControl {
public:
  explicit VelocityControl(const MotorSettings &settings, Degree curr_angle);

  Pwm update(Us dt, Degree position);

  void set_target_speed(DegSec speed);
  void set_settings(const MotorSettings &settings);

private:
  MotorSettings settings_;
  utils::control::Ramp ramp_;
  utils::math::Ema speed_filter_;
  DegSec target_speed_{0};
  Degree last_angle_;
};

class PositionControl {
public:
  explicit PositionControl(const MotorSettings &settings, Degree start_angle,
                           Degree target_distance, Us duration);

  Pwm update(Us dt, Degree position);

  void set_settings(const MotorSettings &settings);

private:
  struct CalcPwmResult {
    Pwm pwm;
    bool target_achieved;
  };

  CalcPwmResult calc_pwm(Us dt, Degree position) const;

private:
  MotorSettings settings_;
  Degree start_angle_;
  Degree target_distance_;
  utils::control::TrajectoryFollower trajectory_follower_;

  bool completed_{false};
  Degree full_angle_;
};
} // namespace robot::states