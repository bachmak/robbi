#pragma once

#include "common.h"

class Wheel;
class WheelAttachment;

namespace wheel_utils
{
    Degree get_full_angle(const Wheel &w, Degree prev_full_angle);
    Meter to_distance(const Wheel &w, Degree angle);
    Degree to_angle(const Wheel &w, Meter distance);

    void stop(Wheel &w, WheelAttachment &wa);
    void rotate(Wheel &w, WheelAttachment &wa, Speed speed);
    void change_speed(Wheel &w, WheelAttachment &wa, Speed delta_speed);
}