#pragma once

#include "types.h"

#include <string>

namespace robot {

struct MotorSettings {
  std::string_view name = "motor";

  float G_vel = 0.3f;
  float G_pos = 1.0f;
  float speed_filter_alpha = 0.1f;

  Pin control_pin{0};
  Pin feedback_pin{0};

  Pwm feedback_pwm_min{1000};
  Pwm feedback_pwm_max{1200};

  float feedback_pwm_duty_cycle_min = 0.029f;
  float feedback_pwm_duty_cycle_max = 0.971f;

  Pwm pwm_min{1400};
  Pwm pwm_stop{1508};
  Pwm pwm_max{1600};

  Pwm pwm_deadband_fwd{22};
  Pwm pwm_deadband_bwd{22};

  float pwm_gain_fwd = 1.0f;
  float pwm_gain_bwd = 1.0f;

  float ff_gain_fwd = 0.155f;
  float ff_gain_bwd = 0.155f;

  float ramp_rise_rate = 100.0f;
  float ramp_fall_rate = 100.0f;

  float trajectory_rise_rate = 400.0f;
  float trajectory_fall_rate = 400.0f;

  bool log = true;

  Degree stop_tolerance_base{0.0f};
  float stop_tolerance_gain{0.1};
};

struct WheelSettings {
  robot::MotorSettings motor = {};
  Meter radius = Meter{0.0326};
};

struct RobotSettings {
  WheelSettings left = {
      .motor =
          {
              .name = "left",
              .pwm_deadband_fwd = Pwm{30},
              .pwm_deadband_bwd = Pwm{27},
          },
  };

  WheelSettings right = {
      .motor =
          {
              .name = "right",
          },
  };

  Meter width = Meter{0.102};
  Ms delay{0};
};
} // namespace robot