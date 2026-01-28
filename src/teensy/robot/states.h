#pragma once

#include "robot/settings.h"
#include "utils/control.h"
#include "utils/math.h"

namespace robot
{
    class VelocityControlState
    {
    public:
        explicit VelocityControlState(const MotorSettings &settings, Degree curr_angle);

        Pwm update(Us dt, Degree position);

        void set_target_speed(DegSec speed);
        void set_settings(const MotorSettings &settings);

    private:
        MotorSettings settings_;
        utils::control::Ramp ramp_;
        utils::math::Ema speed_filter_;
        DegSec target_speed_{0};
        Degree last_angle_;
    };

    class PositionControlState
    {
    public:
        explicit PositionControlState(
            const MotorSettings &settings,
            Degree start_angle,
            Degree target_distance,
            Us duration);

        Pwm update(Us dt, Degree position);

        void set_settings(const MotorSettings &settings);

    private:
        Pwm calc_pwm() const;

    private:
        MotorSettings settings_;
        Degree start_angle_;
        Degree target_distance_;
        DegSec target_speed_;

        Degree full_angle_;
        Us elapsed_time_{0};
    };
}