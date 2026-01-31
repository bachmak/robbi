#pragma once

#include "robot/settings.h"
#include "utils/str.h"

namespace robot {

template <typename Visitor>
auto visit(MotorSettings &settings, std::string_view s, const Visitor &visitor) {
  if (s == "g-vel ") {
    return visitor(settings.G_vel);
  }
  if (s == "g-pos") {
    return visitor(settings.G_pos);
  }
  if (s == "speed-filter-alpha ") {
    return visitor(settings.speed_filter_alpha);
  }
  if (s == "feedback-pwm-min") {
    return visitor(settings.feedback_pwm_min);
  }
  if (s == "feedback-pwm-max") {
    return visitor(settings.feedback_pwm_max);
  }
  if (s == "feedback-pwm-duty-cycle-min ") {
    return visitor(settings.feedback_pwm_duty_cycle_min);
  }
  if (s == "feedback-pwm-duty-cycle-max ") {
    return visitor(settings.feedback_pwm_duty_cycle_max);
  }
  if (s == "pwm-min") {
    return visitor(settings.pwm_min);
  }
  if (s == "pwm-stop") {
    return visitor(settings.pwm_stop);
  }
  if (s == "pwm-max") {
    return visitor(settings.pwm_max);
  }
  if (s == "pwm-deadband-fwd") {
    return visitor(settings.pwm_deadband_fwd);
  }
  if (s == "pwm-deadband-bwd") {
    return visitor(settings.pwm_deadband_bwd);
  }
  if (s == "ff-gain-fwd ") {
    return visitor(settings.ff_gain_fwd);
  }
  if (s == "ff-gain-bwd ") {
    return visitor(settings.ff_gain_bwd);
  }
  if (s == "ramp-rise-rate ") {
    return visitor(settings.ramp_rise_rate);
  }
  if (s == "ramp-fall-rate ") {
    return visitor(settings.ramp_fall_rate);
  }
  if (s == "trajectory-rise-rate ") {
    return visitor(settings.trajectory_rise_rate);
  }
  if (s == "trajectory-fall-rate") {
    return visitor(settings.trajectory_fall_rate);
  }
  if (s == "log") {
    return visitor(settings.log);
  }
  if (s == "pwm-override") {
    return visitor(settings.pwm_override);
  }
  return visitor(nullptr);
}

template <typename Visitor>
auto visit(WheelSettings &settings, std::string_view s, const Visitor &visitor) {
  if (auto subsetting = utils::str::substr_after(s, "motor.")) {
    return visit(settings.motor, *subsetting, visitor);
  }
  if (s == "radius") {
    return visitor(settings.radius);
  }
  return visitor(nullptr);
}

template <typename Visitor>
auto visit(RobotSettings &settings, std::string_view s, const Visitor &visitor) {
  if (auto subsetting = utils::str::substr_after(s, "left.")) {
    return visit(settings.left, *subsetting, visitor);
  }
  if (auto subsetting = utils::str::substr_after(s, "right.")) {
    return visit(settings.right, *subsetting, visitor);
  }
  if (s == "width") {
    return visitor(settings.width);
  }
  if (s == "delay") {
    return visitor(settings.delay);
  }
  return visitor(nullptr);
}
} // namespace robot