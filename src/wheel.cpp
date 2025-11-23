#include "wheel.h"

#include "pwm_utils.h"
#include "io_utils.h"

#include <string>

Wheel::Wheel(const WheelSettings &settings)
    : settings_(settings), current_speed_(0)
{
    pinMode(settings_.feedback_pin, INPUT);

    auto wa = attach();
    rotate(wa, current_speed_);
}

void Wheel::rotate(WheelAttachment &wa, int speed)
{
    if (&wa.servo != &servo_)
    {
        io_utils::error("%s: invalid wheel attachment", settings_.label.c_str());
        return;
    }

    // dead range compensation
    const auto direction = speed < 0 ? -1 : 1;
    const auto delta_pwm = speed == 0
                               ? 0
                               : speed + direction * settings_.speed_dead_range;

    const auto pwm = settings_.stop_pwm + delta_pwm;

    if (pwm < settings_.min_pwm || pwm > settings_.max_pwm)
    {
        io_utils::error("%s: pwm out of bounds: %d", settings_.label, pwm);
        return;
    }

    current_speed_ = speed;
    wa.servo.write(pwm);
}

float Wheel::read_angle() const
{
    const auto pwd_duration = pwm_utils::measure_pwm_duration(
        settings_.feedback_pin,
        settings_.feedback_pwm_min,
        settings_.feedback_pwm_max);

    const auto dc = pwm_utils::to_duty_cycle(pwd_duration);
    const auto dc_min = settings_.feedback_pwm_duty_cycle_min;
    const auto dc_max = settings_.feedback_pwm_duty_cycle_max;

    // TODO: revisit correctness of the formula
    const auto angle = 359 - ((dc - dc_min) * 360.0) / (dc_max - dc_min + 0.01);

    return angle;
}
