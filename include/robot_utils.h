#pragma once

#include "robot.h"

namespace robot_utils
{
    inline void move_fwd(Robot &robot, float distance)
    {
        robot.move(distance, 0);
    }

    inline void move_bwd(Robot &robot, float distance)
    {
        robot.move(0, distance);
    }

    inline void rotate_left(Robot &robot, float angle)
    {
        robot.rotate(angle, 0);
    }

    inline void rotate_right(Robot &robot, float angle)
    {
        robot.rotate(0, angle);
    }
}