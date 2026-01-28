#include "motor/states.h"

#include "utils/io.h"
#include "utils/geometry.h"
#include "utils/time.h"

#include <Arduino.h>

#include <utility>

namespace robot
{
    namespace
    {
        DegSec to_speed(Degree angle, Us dt)
        {
            return DegSec{angle.v / utils::time::to_sec(dt)};
        }

        Pwm speed_to_pwm(DegSec speed, const Settings &settings)
        {
            const auto [zero_point, gain] = [&]() -> std::pair<Pwm, float>
            {
                if (speed == 0.0f)
                {
                    return {settings.pwm_stop, 0.0f};
                }
                if (speed > 0.0f)
                {
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

        bool target_achieved(
            Degree target_distance,
            Degree traveled_distance,
            DegSec speed,
            const Settings &settings)
        {
            const auto tolerance = settings.stop_tolerance_base +
                                   Degree{settings.stop_tolerance_gain * abs(speed).v};

            if (target_distance > 0)
            {
                return traveled_distance >= target_distance - tolerance;
            }
            return traveled_distance <= target_distance + tolerance;
        }
    }

    VelocityControlState::VelocityControlState(const Settings &settings, Degree curr_angle)
        : settings_(settings),
          ramp_(settings_.ramp_rise_rate, settings_.ramp_fall_rate),
          speed_filter_(settings.speed_filter_alpha),
          last_angle_(curr_angle)
    {
    }

    Pwm VelocityControlState::update(Us dt, Degree curr_angle)
    {
        const auto prev_angle = std::exchange(last_angle_, curr_angle);
        const auto delta_angle = utils::geometry::to_delta(curr_angle, prev_angle);

        const auto speed_raw = to_speed(delta_angle, dt);
        const auto speed = DegSec{speed_filter_.update(speed_raw.v)};
        const auto setpoint_speed = DegSec{ramp_.update(target_speed_.v, utils::time::to_sec(dt))};

        const auto ff_pwm = speed_to_pwm(setpoint_speed, settings_);
        const auto pwm_correction = Pwm{settings_.G_vel * (target_speed_.v - speed.v)};

        const auto pwm = ff_pwm + pwm_correction;

        if (settings_.log)
        {
            utils::io::debug("%s: state=vel-ctl, "
                             "err=%f, speed=(tg:%.2f,sp:%.2f,r:%.2f,f:%.2f), "
                             "pwm=(%d+%d=%d)",
                             settings_.name.c_str(),
                             target_speed_.v - speed.v,
                             target_speed_.v,
                             setpoint_speed.v,
                             speed_raw.v,
                             speed.v,
                             ff_pwm.v,
                             pwm_correction.v,
                             pwm.v);
        }

        return pwm;
    }

    void VelocityControlState::set_target_speed(DegSec speed)
    {
        target_speed_ = speed;
    }

    void VelocityControlState::set_settings(const Settings &settings)
    {
        settings_ = settings;
    }

    PositionControlState::PositionControlState(
        const Settings &settings,
        Degree start_angle,
        Degree target_distance,
        Us duration)
        : settings_(settings),
          start_angle_{start_angle},
          target_distance_{target_distance},
          target_speed_{target_distance.v / utils::time::to_sec(duration)},
          full_angle_{start_angle}
    {
    }

    Pwm PositionControlState::update(Us dt, Degree curr_angle)
    {
        elapsed_time_ += dt;
        full_angle_ = utils::geometry::to_full(curr_angle, full_angle_);
        return calc_pwm();
    }

    void PositionControlState::set_settings(const Settings &settings)
    {
        settings_ = settings;
    }

    Pwm PositionControlState::calc_pwm() const
    {
        const auto traveled_distance = full_angle_ - start_angle_;
        const auto setpoint_distance = Degree{target_speed_.v * utils::time::to_sec(elapsed_time_)};

        const auto err = setpoint_distance - traveled_distance;

        const auto pwm_base = speed_to_pwm(target_speed_, settings_);
        const auto pwm_correction = Pwm{settings_.G_pos * err.v};
        const auto pwm = [&]
        {
            if (target_achieved(target_distance_,
                                traveled_distance,
                                target_speed_,
                                settings_))
            {
                return settings_.pwm_stop;
            }
            return pwm_base + pwm_correction;
        }();

        if (settings_.log)
        {
            utils::io::debug("%s: state=pos-ctl, "
                             "err=%f, dist=(tg:%.2f,sp:%.2f,r:%.2f), "
                             "pwm=(%d+%d=%d)",
                             settings_.name.c_str(),
                             err.v,
                             target_distance_.v,
                             setpoint_distance.v,
                             traveled_distance.v,
                             pwm_base.v,
                             pwm_correction.v,
                             pwm.v);
        }

        return pwm;
    }
}
