#pragma once

#include "types.h"

namespace utils::geometry
{
    Meter to_distance(Degree angle, Meter circumference);
    Degree to_angle(Meter distance, Meter circumference);
    Meter to_sector(Degree angle, Meter radius);

    Degree to_delta(Degree curr_angle, Degree prev_angle);
    Degree to_full(Degree curr_angle, Degree prev_full_angle);

    struct Vec3
    {
        float x;
        float y;
        float z;
    };

    struct Twist
    {
        Vec3 linear;
        Vec3 angular;
    };
}