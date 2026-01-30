#include "robot/states.h"

#include "utils/geometry.h"
#include "utils/io.h"
#include "utils/time.h"

#include <utility>

namespace robot::states {
namespace {

DegSec to_speed(Degree angle, Us dt) { return DegSec{angle.v / utils::time::to_sec(dt)}; }

Pwm speed_to_pwm_ff(DegSec speed, const MotorSettings &settings) {
  const auto [zero_point, gain] = [&]() -> std::pair<Pwm, float> {
    if (speed == 0.0f) {
      return {settings.pwm_stop, 0.0f};
    }
    if (speed > 0.0f) {
      return {
          settings.pwm_stop + settings.pwm_deadband_fwd,
          settings.ff_gain_fwd,
      };
    }
    return {
        settings.pwm_stop - settings.pwm_deadband_bwd,
        settings.ff_gain_bwd,
    };
  }();

  const auto pwm = zero_point + Pwm{speed.v * gain};
  return pwm;
}

bool target_achieved(Degree target_distance, Degree traveled_distance, DegSec speed,
                     const MotorSettings &settings) {
  const auto tolerance =
      settings.stop_tolerance_base + Degree{settings.stop_tolerance_gain * abs(speed).v};

  if (target_distance > 0) {
    return traveled_distance >= target_distance - tolerance;
  }
  return traveled_distance <= target_distance + tolerance;
}

utils::control::Trajectory build_trapezoidal_trajectory(const Degree target, const Us duration,
                                                        const MotorSettings &settings) {
  const auto rise_rate = settings.trajectory_rise_rate;
  const auto fall_rate = settings.trajectory_fall_rate;
  const auto T = utils::time::to_sec(duration);

  const auto sign = (target >= 0.0f) ? 1.0f : -1.0f;
  const auto dist = std::abs(static_cast<float>(target.v));

  // Quadratic equation to find the cruise velocity:
  //            2
  // K / 2 * v_c  -  T * v_c  +  D  =  0
  //
  // Where:
  // v_c = v_cruise
  // K   = (1/rise_rate + 1/fall_rate)
  // T   = total time
  // D   = total distance

  const auto inv_accel = (1.0f / rise_rate) + (1.0f / fall_rate);
  const auto discriminant = T * T - 2.0f * dist * inv_accel;
  if (discriminant < 0.0f) {
    utils::io::error("Cannot create trajectory for target=%.2f, duration=%.2f", target.v,
                     utils::time::to_sec(duration));
    return utils::control::Trajectory{};
  }

  const auto v_cruise = (T - std::sqrt(discriminant)) / inv_accel;

  const auto t_accel_end = v_cruise / rise_rate;
  const auto t_decel_start = T - (v_cruise / fall_rate);

  const auto t_accel_duration = t_accel_end;
  const auto t_decel_duration = T - t_decel_start;

  // uniform acceleration
  const auto x_accel_dist = 0.5f * v_cruise * t_accel_duration;
  const auto x_decel_dist = 0.5f * v_cruise * t_decel_duration;

  const auto x_accel_end = x_accel_dist;
  const auto x_decel_start = dist - x_decel_dist;

  return utils::control::Trajectory{{
      {0.0f, 0.0f},
      {t_accel_end, x_accel_end * sign},
      {t_decel_start, x_decel_start * sign},
      {T, dist * sign},
  }};
}

void log(std::string_view motor, std::string_view state,                  //
         Us dt, float err, Degree pos,                                    //
         float target, float setpoint, float value_raw, float value_filt, //
         Pwm pwm_ff, Pwm pwm_correction, Pwm pwm) {
  utils::io::debug("motor=%s, state=%s,\n"
                   "dt=%lldus, err=%.2f, pos=%.2f,\n"
                   "tg=%.2f, sp=%.2f, raw=%.2f, filt=%.2f,\n"
                   "pwm=(%d+%d=%d)",
                   motor.data(), state.data(),              //
                   dt.count(), err, pos.v,                  //
                   target, setpoint, value_raw, value_filt, //
                   pwm_ff.v, pwm_correction.v, pwm.v        //
  );
}

} // namespace

VelocityControl::VelocityControl(const MotorSettings &settings, Degree curr_angle)
    : settings_(settings), ramp_(settings_.ramp_rise_rate, settings_.ramp_fall_rate),
      speed_filter_(settings.speed_filter_alpha), last_angle_(curr_angle) {}

Pwm VelocityControl::update(Us dt, Degree curr_angle) {
  const auto prev_angle = std::exchange(last_angle_, curr_angle);
  const auto delta_angle = utils::geometry::to_delta(curr_angle, prev_angle);

  const auto speed_raw = to_speed(delta_angle, dt);
  const auto speed = DegSec{speed_filter_.update(speed_raw.v)};
  const auto setpoint_speed = DegSec{ramp_.update(target_speed_.v, utils::time::to_sec(dt))};

  const auto err = setpoint_speed - speed;

  const auto pwm_ff = speed_to_pwm_ff(setpoint_speed, settings_);
  const auto pwm_correction = Pwm{settings_.G_vel * err.v};

  const auto pwm = pwm_ff + pwm_correction;

  if (settings_.log) {
    log(settings_.name, "vel-ctl",         //
        dt, err.v, curr_angle,             //
        target_speed_.v, setpoint_speed.v, //
        speed_raw.v, speed.v,              //
        pwm_ff, pwm_correction, pwm        //
    );
  }

  return pwm;
}

void VelocityControl::set_target_speed(DegSec speed) { target_speed_ = speed; }

void VelocityControl::set_settings(const MotorSettings &settings) { settings_ = settings; }

PositionControl::PositionControl(const MotorSettings &settings, Degree start_angle,
                                 Degree target_distance, Us duration)
    : settings_(settings), start_angle_{start_angle}, target_distance_{target_distance},
      target_speed_{target_distance.v / utils::time::to_sec(duration)},
      trajectory_follower_{build_trapezoidal_trajectory(target_distance, duration, settings_)},
      full_angle_{start_angle} {}

Pwm PositionControl::update(Us dt, Degree curr_angle) {
  elapsed_time_ += dt;
  full_angle_ = utils::geometry::to_full(curr_angle, full_angle_);
  const auto [pwm, completed] = calc_pwm(dt, curr_angle);
  completed_ = completed;
  return pwm;
}

void PositionControl::set_settings(const MotorSettings &settings) { settings_ = settings; }

auto PositionControl::calc_pwm(Us dt, Degree position) const -> CalcPwmResult {
  const auto traveled_distance = full_angle_ - start_angle_;
  const auto elapsed_sec = utils::time::to_sec(elapsed_time_);
  const auto setpoint_distance = Degree{trajectory_follower_.current_value(elapsed_sec)};

  const auto err = setpoint_distance - traveled_distance;

  const auto pwm_ff = speed_to_pwm_ff(target_speed_, settings_);
  const auto pwm_correction = Pwm{settings_.G_pos * err.v};
  const auto result = [&]() -> CalcPwmResult {
    if (target_achieved(target_distance_, traveled_distance, target_speed_, settings_)) {
      return {settings_.pwm_stop, true};
    }
    return {pwm_ff + pwm_correction, false};
  }();

  if (settings_.log) {
    log(settings_.name, "pos-ctl",               //
        dt, err.v, position,                     //
        target_distance_.v, setpoint_distance.v, //
        traveled_distance.v, 0.0f,               //
        pwm_ff, pwm_correction, pwm              //
    );
  }

  return result;
}
} // namespace robot::states
