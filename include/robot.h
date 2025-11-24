#pragma once

#include "wheel.h"

struct Wheels
{
    Wheel left;
    Wheel right;
};

class Robot
{
public:
    Robot(const Wheels &wheels) : wheels_(wheels) {}

public:
    void move(Meter target_pos, Meter cur_pos);
    void rotate(Degree target_pos, Degree cur_pos);

    void rotate(Meter left, Meter right, Speed speed);
    void rotate(Degree left, Degree right, Speed speed);

private:
    Wheels wheels_;
};
