#include "types.h"

namespace utils::pwm
{
    struct CycleDuration
    {
        Us high;
        Us full;
    };

    float to_duty_cycle(CycleDuration duration);
    CycleDuration measure_pwm_duration(Pin pin);
    CycleDuration measure_pwm_duration(Pin pin, Pwm min, Pwm max);
}