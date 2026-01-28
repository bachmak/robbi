#pragma once

#include "motor/motor.h"
#include "utils/geometry.h"

namespace robot
{
    struct WheelSettings
    {
        motor::Settings motor = {};
        Meter radius = Meter{0.0326};
    };

    struct RobotSettings
    {
        WheelSettings left = {
            .motor = {
                .pwm_deadband_fwd = Pwm{30},
                .pwm_deadband_bwd = Pwm{27},
            },
        };

        WheelSettings right;
        Meter width = Meter{0.102};
    };

    class Robot
    {
    public:
        explicit Robot(const RobotSettings &settings);

    public:
        void update(Us dt);

        void set_target_speed(const utils::geometry::Twist &twist);
        void set_target_distance(Meter distance, Us duration);
        void set_target_rotation(Degree rotation, Us duration);
        void set_stop(bool value);

        void configure(std::string_view setting, float value);

    private:
        RobotSettings settings_;
        motor::Motor left_;
        motor::Motor right_;
    };
}