#pragma once

#include "motor.h"
#include "geo_utils.h"

struct WheelSettings
{
    MotorSettings motor = {};
    Meter radius = Meter{0.0326};
};

struct WheelySettings
{
    WheelSettings left = {
        .motor = {
            .pwm_deadband_fwd = Us{29},
            .pwm_deadband_bwd = Us{27},
        },
    };

    WheelSettings right;
    Meter width = Meter{0.102};
};

class Wheely
{
public:
    explicit Wheely(const WheelySettings &settings);

public:
    void update(Us dt);

    void set_target_speed(const geo_utils::Twist &twist);
    void set_stop(bool value);

    void configure(std::string_view setting, float value);

private:
    WheelySettings settings_;
    Motor left_;
    Motor right_;
};
