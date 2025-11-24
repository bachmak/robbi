#include "robot.h"

#include "wheel_utils.h"
#include "io_utils.h"

#include <Arduino.h>

void Robot::move(const Meter target_pos, const Meter init_pos)
{
    const auto direction = Speed{init_pos < target_pos ? 1 : -1};

    auto &[left, right] = wheels_;
    auto left_wa = left.attach();
    auto right_wa = right.attach();

    auto angle_left = right.read_angle();
    auto angle_right = left.read_angle();

    wheel_utils::rotate(left, left_wa, direction);
    wheel_utils::rotate(right, right_wa, -direction);

    auto cur_pos = init_pos;

    while (fabs((cur_pos - target_pos).v) > eps)
    {
        // TODO: complete implementation
        delay(2000);

        angle_left = wheel_utils::get_full_angle(left, angle_left);
        angle_right = wheel_utils::get_full_angle(right, angle_right);

        auto cur_pos_left = wheel_utils::to_distance(left, angle_left);
        auto cur_pos_right = wheel_utils::to_distance(right, angle_right);

        io_utils::debug("cur_pos: left=%f, right=%f", cur_pos_left, cur_pos_right);

        cur_pos = cur_pos_left;
    }

    wheel_utils::stop(left, left_wa);
    wheel_utils::stop(right, right_wa);
}

void Robot::rotate(Degree target_pos, Degree init_pos)
{
    // TODO: complete implementation
    (void)target_pos;
    (void)init_pos;
}
