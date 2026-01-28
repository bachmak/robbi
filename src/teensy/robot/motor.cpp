#include "robot/motor.h"

#include "utils/pwm.h"
#include "utils/geometry.h"
#include "utils/visitor.h"
#include "utils/io.h"

#include <Servo.h>
#include <Arduino.h>

#include <utility>
#include <algorithm>

namespace robot
{
    namespace
    {
        void init_pins(const MotorSettings &settings)
        {
            pinMode(settings.control_pin.v, OUTPUT);
            pinMode(settings.feedback_pin.v, INPUT);
        }

        Degree read_position(const MotorSettings &settings)
        {
            const auto pwd_duration = utils::pwm::measure_pwm_duration(
                settings.feedback_pin,
                settings.feedback_pwm_min,
                settings.feedback_pwm_max);

            const auto dc = utils::pwm::to_duty_cycle(pwd_duration);
            const auto dc_min = settings.feedback_pwm_duty_cycle_min;
            const auto dc_max = settings.feedback_pwm_duty_cycle_max;

            // TODO: revisit correctness of the formula
            const auto angle = Degree{
                359 - ((dc - dc_min) * 360.0f) / (dc_max - dc_min + 0.01f)};

            return std::clamp(angle, Degree{0}, Degree{360});
        }
    }

    Motor::Motor(const MotorSettings &settings)
        : state_(VelocityControlState{settings, (init_pins(settings), read_position(settings))}),
          settings_(settings)
    {
        servo_.attach(settings_.control_pin.v);
    }

    Motor::~Motor()
    {
        servo_.detach();
    }

    void Motor::update(Us dt)
    {
        const auto curr_angle = read_position(settings_);
        const auto pwm = std::visit([dt, curr_angle](auto &state)
                                    { return state.update(dt, curr_angle); }, state_);

        const auto final_pwm = [&]
        {
            if (stop_)
            {
                return settings_.pwm_stop;
            }
            if (pwm_override_.has_value())
            {
                return *pwm_override_;
            }
            return std::clamp(pwm, settings_.pwm_min, settings_.pwm_max);
        }();

        servo_.writeMicroseconds(final_pwm.v);
    }

    void Motor::set_target_speed(DegSec speed)
    {
        std::visit(utils::visitor::overloads{[speed](VelocityControlState &state)
                                             {
                                                 state.set_target_speed(speed);
                                             },
                                             [&](PositionControlState &state)
                                             {
                                                 auto curr_angle = read_position(settings_);
                                                 state_ = VelocityControlState(settings_, curr_angle);
                                                 set_target_speed(speed);
                                             }},
                   state_);
    }

    void Motor::set_target_distance(Degree target_distance, Us duration)
    {
        auto curr_angle = read_position(settings_);
        state_ = PositionControlState(settings_, curr_angle, target_distance, duration);
    }

    void Motor::set_stop(bool value) { stop_ = value; }

    void Motor::configure(std::string_view s, float value)
    {
        const auto pwm = Pwm{value};

        if (s == "speed")
        {
            if (auto state = std::get_if<VelocityControlState>(&state_))
            {
                state->set_target_speed(DegSec{value});
            }
            return;
        }
        if (s == "pwm-override")
        {
            pwm_override_ = pwm == 0 ? std::optional<Pwm>{} : std::optional<Pwm>{pwm};
            return;
        }

        if (s == "ramp-rise-rate")
        {
            settings_.ramp_rise_rate = value;
        }
        else if (s == "ramp-fall-rate")
        {
            settings_.ramp_fall_rate = value;
        }
        else if (s == "speed-filter-alpha")
        {
            settings_.speed_filter_alpha = value;
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
        else if (s == "ff-gain-fwd")
        {
            settings_.ff_gain_fwd = value;
        }
        else if (s == "ff-gain-bwd")
        {
            settings_.ff_gain_bwd = value;
        }
        else if (s == "g-vel")
        {
            settings_.G_vel = value;
        }
        else if (s == "g-pos")
        {
            settings_.G_pos = value;
        }
        else if (s == "log")
        {
            settings_.log = value != 0.0f;
        }
        else if (s == "stop-tolerance-base")
        {
            settings_.stop_tolerance_base = Degree{value};
        }
        else if (s == "stop-tolerance-gain")
        {
            settings_.stop_tolerance_gain = value;
        }
        else
        {
            utils::io::error("Motor: unknown settins: %s", s.data());
        }

        std::visit([this](auto &state)
                   { state.set_settings(settings_); }, state_);
    }
}