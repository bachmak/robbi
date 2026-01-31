#pragma once

#include "types.h"

#include <optional>
#include <string_view>

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

  Pwm pwm_min{1300};
  Pwm pwm_stop{1508};
  Pwm pwm_max{1700};

  Pwm pwm_deadband_fwd{22};
  Pwm pwm_deadband_bwd{22};

  float ff_gain_fwd = 0.155f;
  float ff_gain_bwd = 0.155f;

  float ramp_rise_rate = 100.0f;
  float ramp_fall_rate = 100.0f;

  float trajectory_rise_rate = 200.0f;
  float trajectory_fall_rate = 200.0f;

  bool log = true;
  Pwm pwm_override = Pwm{0}; // no override by default
};

bool set(MotorSettings &settings, std::string_view s, float value);
std::optional<float> get(const MotorSettings &settings, std::string_view s);

struct WheelSettings {
  MotorSettings motor = {};
  Meter radius = Meter{0.033};
};

bool set(WheelSettings &settings, std::string_view s, float value);
std::optional<float> get(const WheelSettings &settings, std::string_view s);

struct RobotSettings {
  WheelSettings left;
  WheelSettings right;

  Meter width = Meter{0.1005};
  Ms delay{0};
};

bool set(RobotSettings &settings, std::string_view s, float value);
std::optional<float> get(const RobotSettings &settings, std::string_view s);
} // namespace robot