#include "motor.h"

#include "pwm_utils.h"
#include "geo_utils.h"
#include "common_utils.h"
#include "io_utils.h"

#include <Servo.h>
#include <Arduino.h>

#include <utility>
#include <algorithm>

namespace
{
    void init_pins(const MotorSettings &settings)
    {
        pinMode(settings.control_pin.v, OUTPUT);
        pinMode(settings.feedback_pin.v, INPUT);
    }

    float to_sec(Us t)
    {
        using R = std::ratio_divide<Us::period, Sec::period>;
        constexpr auto num = R::num;
        constexpr auto den = R::den;

        return static_cast<float>(t.count()) * num / den;
    }

    Degree read_position(const MotorSettings &settings)
    {
        const auto pwd_duration = pwm_utils::measure_pwm_duration(
            settings.feedback_pin,
            settings.feedback_pwm_min,
            settings.feedback_pwm_max);

        const auto dc = pwm_utils::to_duty_cycle(pwd_duration);
        const auto dc_min = settings.feedback_pwm_duty_cycle_min;
        const auto dc_max = settings.feedback_pwm_duty_cycle_max;

        // TODO: revisit correctness of the formula
        const auto angle = Degree{
            359 - ((dc - dc_min) * 360.0f) / (dc_max - dc_min + 0.01f)};

        return std::clamp(angle, Degree{0}, Degree{360});
    }

    Degree to_delta(Degree curr_angle, Degree prev_angle)
    {
        const auto delta_angle = curr_angle - prev_angle;
        if (delta_angle > 180.0f)
        {
            return delta_angle - 360.0f;
        }
        if (delta_angle < -180.0f)
        {
            return delta_angle + 360.0f;
        }
        return delta_angle;
    }

    DegSec to_speed(Degree angle, Us dt)
    {
        return DegSec{angle.v / to_sec(dt)};
    }

    Us control_to_pwm(float control, const MotorSettings &settings)
    {
        const auto [zero_point, gain] = [&]() -> std::pair<Us, float>
        {
            if (control == 0.0f)
            {
                return {settings.pwm_stop, 0.0f};
            }
            if (control > 0.0f)
            {
                return {
                    settings.pwm_stop + settings.pwm_deadband_fwd,
                    settings.pwm_gain_fwd,
                };
            }
            return {
                settings.pwm_stop - settings.pwm_deadband_bwd,
                settings.pwm_gain_bwd,
            };
        }();

        const auto pwm = zero_point + Us{static_cast<int64_t>(gain * control)};
        return std::clamp(pwm, settings.pwm_min, settings.pwm_max);
    }
}

Motor::Motor(const MotorSettings &settings)
    : settings_(settings),
      pid_(settings.pid_settings),
      speed_filter_(settings.speed_filter_alpha),
      last_angle_((init_pins(settings), read_position(settings)))
{
    servo_.attach(settings_.control_pin.v);
}

Motor::~Motor()
{
    servo_.detach();
}

void Motor::set_target_speed(DegSec speed)
{
    target_speed_ = speed;
}

DegSec Motor::get_real_speed() const
{
    return DegSec{speed_filter_.last_value()};
}

void Motor::update(Us dt)
{
    const auto curr_angle = read_position(settings_);
    const auto prev_angle = std::exchange(last_angle_, curr_angle);
    const auto delta_angle = to_delta(curr_angle, prev_angle);

    const auto speed_raw = to_speed(delta_angle, dt);
    const auto speed = Speed{speed_filter_.update(speed_raw.v)};

    const auto control = pid_.update(target_speed_.v, speed.v, to_sec(dt));
    const auto pwm = Us{control_to_pwm(control, settings_)};

    io_utils::debug("%s: err=%f, ctl=%f, pwm=%d",
                    settings_.name.c_str(),
                    target_speed_.v - speed.v,
                    control,
                    static_cast<int>(pwm.count()));

    if (stop_)
    {
        servo_.writeMicroseconds(settings_.pwm_stop.count());
    }
    else
    {
    servo_.writeMicroseconds(pwm.count());
    }
}

void Motor::configure(std::string_view s, float value)
{
    if (auto pid_setting = common_utils::substr_after(s, "pid."))
    {
        return pid_.configure(*pid_setting, value);
    }

    const auto pwm = Us{static_cast<int>(value)};

    if (s == "speed-filter-alpha")
    {
        speed_filter_.set_alpha(value);
    }
    else if (s == "fb-pwm-min")
    {
        settings_.feedback_pwm_min = pwm;
    }
    else if (s == "fb-pwm-max")
    {
        settings_.feedback_pwm_max = pwm;
    }
    else if (s == "pwm-min")
    {
        settings_.pwm_min = pwm;
    }
    else if (s == "pwm-max")
    {
        settings_.pwm_max = pwm;
    }
    else if (s == "pwm-stop")
    {
        settings_.pwm_stop = pwm;
    }
    else if (s == "pwm-deadband-fwd")
    {
        settings_.pwm_deadband_fwd = pwm;
    }
    else if (s == "pwm-deadband-bwd")
    {
        settings_.pwm_deadband_bwd = pwm;
    }
    else if (s == "pwm-gain-fwd")
    {
        settings_.pwm_gain_fwd = value;
    }
    else if (s == "pwm-gain-bwd")
    {
        settings_.pwm_gain_bwd = value;
    }
    else
    {
        io_utils::error("Motor: unknown settins: %s", s.data());
    }
}
