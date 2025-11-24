#include "wheel.h"

#include "pwm_utils.h"
#include "io_utils.h"

#include <Arduino.h>
#include <string>

WheelAttachment::WheelAttachment(Servo &servo, Pin pin) : servo(servo) { servo.attach(pin.v); }

WheelAttachment::~WheelAttachment() { servo.detach(); }

Wheel::Wheel(const WheelSettings &settings)
    : settings_(settings), current_speed_(0)
{
    pinMode(settings_.feedback_pin.v, INPUT);

    auto wa = attach();
    rotate(wa, current_speed_);
}

void Wheel::rotate(WheelAttachment &wa, Speed speed)
{
    if (&wa.servo != &servo_)
    {
        io_utils::error("%s: invalid wheel attachment", settings_.label.c_str());
        return;
    }

    // dead range compensation
    const auto direction = Speed{speed < Speed{0} ? -1 : 1};
    const auto delta_pwm = [&]
    {
        if (speed == Speed{0})
        {
            return Pwm{0};
        }

        auto adjusted_speed = speed + direction * settings_.speed_dead_range;
        return Pwm{adjusted_speed.v};
    }();

    const auto pwm = settings_.stop_pwm + delta_pwm;

    if (pwm < settings_.min_pwm || pwm > settings_.max_pwm)
    {
        io_utils::error("%s: pwm out of bounds: %d", settings_.label, pwm);
        return;
    }

    current_speed_ = speed;
    wa.servo.write(pwm.v);
}

Degree Wheel::read_angle() const
{
    const auto pwd_duration = pwm_utils::measure_pwm_duration(
        settings_.feedback_pin,
        settings_.feedback_pwm_min,
        settings_.feedback_pwm_max);

    const auto dc = pwm_utils::to_duty_cycle(pwd_duration);
    const auto dc_min = settings_.feedback_pwm_duty_cycle_min;
    const auto dc_max = settings_.feedback_pwm_duty_cycle_max;

    // TODO: revisit correctness of the formula
    const auto angle = Degree{
        359 - ((dc - dc_min) * 360.0f) / (dc_max - dc_min + 0.01f)};

    return angle;
}
