#pragma once

#include "motor/motor.h"
#include "geo_utils.h"

struct WheelSettings
{
    motor::Settings motor = {};
    Meter radius = Meter{0.0326};
};

struct WheelySettings
{
    WheelSettings left = {
        .motor = {
            .pwm_deadband_fwd = Pwm{29},
            .pwm_deadband_bwd = Pwm{27},
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
    void set_target_distance(Meter distance, Us duration);
    void set_target_rotation(Degree rotation, Us duration);
    void set_stop(bool value);

    void configure(std::string_view setting, float value);

private:
    WheelySettings settings_;
    motor::Motor left_;
    motor::Motor right_;
};
