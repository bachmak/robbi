#pragma once

#include "utils/non_copyable.h"
#include "robot/settings.h"
#include "robot/states.h"

#include <Servo.h>

#include <variant>
#include <optional>

namespace robot
{
    class Motor
    {
    public:
        explicit Motor(const MotorSettings &settings);
        ~Motor();

        NON_COPYABLE(Motor)

    public:
        void update(Us dt);

        void set_target_speed(DegSec speed);
        void set_target_distance(Degree distance, Us duration);
        void set_stop(bool value);

        void configure(std::string_view setting, float value);

    private:
        std::variant<VelocityControlState, PositionControlState> state_;
        MotorSettings settings_;
        Servo servo_;
        std::optional<Pwm> pwm_override_;
        bool stop_ = false;
    };
}
