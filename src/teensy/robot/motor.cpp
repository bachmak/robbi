#include "robot/motor.h"

#include "utils/geometry.h"
#include "utils/io.h"
#include "utils/pwm.h"
#include "utils/visitor.h"

#include <Arduino.h>
#include <Servo.h>

#include <algorithm>
#include <utility>

namespace robot {
namespace {

void init_pins(const MotorSettings &settings) {
  pinMode(settings.control_pin.v, OUTPUT);
  pinMode(settings.feedback_pin.v, INPUT);
}

Degree read_position(const MotorSettings &settings) {
  const auto pwd_duration = utils::pwm::measure_pwm_duration(
      settings.feedback_pin, settings.feedback_pwm_min, settings.feedback_pwm_max);

  const auto dc = utils::pwm::to_duty_cycle(pwd_duration);
  const auto dc_min = settings.feedback_pwm_duty_cycle_min;
  const auto dc_max = settings.feedback_pwm_duty_cycle_max;

  const auto normalized = (dc - dc_min) / (dc_max - dc_min);
  const auto angle = Degree{360.0f - (normalized * 360.0f)};

  return std::clamp(angle, Degree{0}, Degree{360});
}
} // namespace

Motor::Motor(const MotorSettings &settings)
    : state_(states::VelocityControl{settings, (init_pins(settings), read_position(settings))}),
      settings_(settings) {
  servo_.attach(settings_.control_pin.v);
}

Motor::~Motor() { servo_.detach(); }

void Motor::update(Us dt) {
  const auto curr_angle = read_position(settings_);
  const auto pwm =
      std::visit([dt, curr_angle](auto &state) { return state.update(dt, curr_angle); }, state_);

  const auto final_pwm = [&] {
    if (stop_) {
      return settings_.pwm_stop;
    }
    if (settings_.pwm_override > 0) {
      return settings_.pwm_override;
    }

    const auto pwm_clamped = std::clamp(pwm, settings_.pwm_min, settings_.pwm_max);
    if (pwm_clamped != pwm) {
      utils::io::warning("Motor=%s: pwm clamped: requested=%d, clamped=%d", //
                         settings_.name.data(), pwm.v, pwm_clamped.v);
    }

    return pwm_clamped;
  }();

  servo_.writeMicroseconds(final_pwm.v);
}

void Motor::set_target_speed(DegSec speed) {
  std::visit(utils::visitor::overloads{
                 [speed](states::VelocityControl &state) { state.set_target_speed(speed); },
                 [&](states::PositionControl &state) {
                   auto curr_angle = read_position(settings_);
                   state_ = states::VelocityControl(settings_, curr_angle);
                   set_target_speed(speed);
                 }},
             state_);
}

void Motor::set_target_distance(Degree target_distance, Us duration) {
  auto curr_angle = read_position(settings_);
  state_ = states::PositionControl(settings_, curr_angle, target_distance, duration);
}

void Motor::set_stop(bool value) { stop_ = value; }

void Motor::set_settings(const MotorSettings &settings) {
  settings_ = settings;
  std::visit([&](auto &state) { state.set_settings(settings); }, state_);
}
} // namespace robot