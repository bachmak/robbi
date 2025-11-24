#include "common.h"

namespace pwm_utils
{
    struct CycleDuration
    {
        Us high;
        Us full;
    };

    float to_duty_cycle(CycleDuration duration);
    CycleDuration measure_pwm_duration(Pin pin);
    CycleDuration measure_pwm_duration(Pin pin, Us min, Us max);
}