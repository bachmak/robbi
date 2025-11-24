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

    while (abs(cur_pos - target_pos) > Meter{0.01})
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

void Robot::rotate(Meter left, Meter right, Speed speed)
{
    auto left_deg = wheel_utils::to_angle(wheels_.left, left);
    auto right_deg = wheel_utils::to_angle(wheels_.right, right);
    rotate(left_deg, right_deg, speed);
}

void Robot::rotate(Degree target_left, Degree target_right, Speed speed)
{
    auto cur_left = Degree{0};
    auto cur_right = Degree{0};

    auto progress_left = 0.0f;
    auto progress_right = 0.0f;

    auto w_left = wheels_.left;
    auto w_right = wheels_.right;

    auto wa_left = w_left.attach();
    auto wa_right = w_right.attach();

    auto dir_left = Speed{target_left > Degree{0} ? 1 : -1};
    auto dir_right = Speed{target_right > Degree{0} ? 1 : -1};

    auto speed_left = dir_left * speed;
    auto speed_right = -(dir_right * speed);

    while (progress_left < 1.0f || progress_right < 1.0f)
    {
        io_utils::debug(
            "progress_left=%f, progress_right=%f",
            progress_left,
            progress_right);

        w_left.rotate(wa_left, speed_left);
        w_right.rotate(wa_right, speed_right);

        cur_left = wheel_utils::get_full_angle(w_left, cur_left);
        cur_right = wheel_utils::get_full_angle(w_right, cur_right);

        progress_left = (cur_left / target_left).v;
        progress_right = (cur_right / target_right).v;

        auto diff = abs(progress_left - progress_right);

        constexpr auto p_coeff = 1;
        if (cur_right < cur_left)
        {
            speed_right += Speed{static_cast<Speed::ValueType>(dir_right.v * p_coeff * diff)};
        }

        if (cur_left < cur_right)
        {
            speed_left += Speed{static_cast<Speed::ValueType>(dir_left.v * p_coeff * diff)};
        }
    }

    w_left.rotate(wa_left, Speed{0});
    w_right.rotate(wa_right, Speed{0});
}
