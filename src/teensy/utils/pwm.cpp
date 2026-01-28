#include "pwm_utils.h"

#include <Arduino.h>

namespace pwm_utils
{
    float to_duty_cycle(CycleDuration duration)
    {
        return static_cast<float>(duration.high.count()) / duration.full.count();
    }

    CycleDuration measure_pwm_duration(Pin pin)
    {
        auto high = Us{pulseIn(pin.v, HIGH)};
        auto low = Us{pulseIn(pin.v, LOW)};
        auto full = high + low;

        return {high, full};
    }

    CycleDuration measure_pwm_duration(Pin pin, Pwm min, Pwm max)
    {
        while (true)
        {
            auto duration = measure_pwm_duration(pin);
            if (duration.full.count() > min.v && duration.full.count() < max.v)
            {
                return duration;
            }
        }
    }
}