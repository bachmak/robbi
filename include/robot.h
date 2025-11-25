#pragma once

#include "wheel.h"

struct RobotSettings
{
    WheelSettings left_wheel_settings;
    WheelSettings right_wheel_settings;
    Ms max_move_time{1'000'000};
    float p_controller_gain = 50.0f;
    float speed_ema_filter_coef = 0.1f;
    Ms wait_after_move{100};
    Degree inertial_stop_angle{5};
};

struct Wheels
{
    Wheels(const WheelSettings &left_settings, const WheelSettings &right_settings)
        : left(left_settings), right(right_settings) {}

    Wheel left;
    Wheel right;
};

class Robot
{
public:
    explicit Robot(const RobotSettings &settings);

public:
    void move(Meter target_pos, Meter cur_pos);
    void rotate(Degree target_pos, Degree cur_pos);

    void rotate(Meter left, Meter right, Speed speed);
    void rotate(Degree left, Degree right, Speed speed);

private:
    Wheels wheels_;
    RobotSettings settings_;
};
