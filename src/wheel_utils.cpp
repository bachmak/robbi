#include "wheel_utils.h"

#include "wheel.h"

#include <math.h>

namespace wheel_utils
{
    Degree get_full_angle(const Wheel &wheel, const Degree prev_full_angle)
    {
        const auto prev_angle = Degree{fmod(prev_full_angle.v, 360.0)};
        const auto curr_angle = wheel.read_angle();

        const auto prev_turns = static_cast<int>(prev_full_angle.v / 360.0);
        const auto curr_turns = [&]
        {
            constexpr auto min = Degree{90};
            constexpr auto max = Degree{270};

            if (curr_angle < min && prev_angle > max)
            {
                return prev_turns + 1;
            }

            if (prev_angle < min && curr_angle > max)
            {
                return prev_turns - 1;
            }

            return prev_turns;
        }();

        const auto curr_full_angle = Degree{curr_turns * 360 + curr_angle.v};
        return curr_full_angle;
    }

    Meter to_distance(const Wheel &wheel, Degree angle)
    {
        return Meter{angle.v / 360 * wheel.circumference().v};
    }

    Degree to_angle(const Wheel &wheel, Meter distance)
    {
        return Degree{distance.v / wheel.circumference().v * 360};
    }

    void stop(Wheel &wheel, WheelAttachment &wa)
    {
        wheel.rotate(wa, Speed{0});
    }

    void rotate(Wheel &wheel, WheelAttachment &wa, Speed speed)
    {
        wheel.rotate(wa, speed);
    }

    void change_speed(Wheel &wheel, WheelAttachment &wa, Speed delta_speed)
    {
        auto speed = wheel.current_speed() + delta_speed;
        wheel.rotate(wa, speed);
    }
}
