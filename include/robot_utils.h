#pragma once

#include "common.h"

namespace robot_utils
{
    inline void move_fwd(Robot &robot, Meter distance)
    {
        robot.move(distance, Meter{0});
    }

    inline void move_bwd(Robot &robot, Meter distance)
    {
        robot.move(Meter{0}, distance);
    }

    inline void rotate_left(Robot &robot, Degree angle)
    {
        robot.rotate(angle, Degree{0});
    }

    inline void rotate_right(Robot &robot, Degree angle)
    {
        robot.rotate(Degree{0}, angle);
    }
}