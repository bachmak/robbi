#include "geo_utils.h"

#include <math.h>

namespace geo_utils
{
    namespace
    {
        Degree normalize(Degree d)
        {
            const auto v = fmod(d.v, 360.0f);
            if (v < 0)
            {
                return Degree{v + 360.0f};
            }
            return Degree{v};
        }

        constexpr auto pi = static_cast<float>(M_PI);
    }

    Meter to_distance(Degree angle, Meter circumference)
    {
        return Meter{angle.v / 360 * circumference.v};
    }

    Degree to_angle(Meter distance, Meter circumference)
    {
        return Degree{(distance / circumference * 360).v};
    }

    Meter to_sector(Degree angle, Meter radius)
    {
        return radius * pi * angle.v / 180.0f;
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

    Degree to_full(Degree curr_angle, Degree prev_full_angle)
    {
        const auto prev_angle = normalize(prev_full_angle);
        const auto diff = to_delta(curr_angle, prev_angle);
        return prev_full_angle + diff;
    }
}
