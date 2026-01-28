#pragma once

#include "common.h"

#include <string_view>

namespace utils::control
{
    struct PIDSettings
    {
        float G;
        float T_i;
        float T_d;
        float out_min;
        float out_max;
        float deadband;
    };

    struct PIDParams
    {
        float K_p;
        float K_i;
        float K_d;
    };

    class PID
    {
    public:
        explicit PID(const PIDSettings &settings);

    public:
        float update(float setpoint, float measurement, float dt);

        void configure(std::string_view setting, float value);

    private:
        PIDSettings settings_;
        PIDParams params_;
        float integrator_;
        float prev_error_;
        bool first_run_ = true;
        bool log_ = true;
    };

    class Ramp
    {
    public:
        Ramp(float rise_rate, float fall_rate);
        float update(float target, float dt);

        void reset(float v);
        float value() const;

        void configure(std::string_view setting, float value);

    private:
        float rise_rate_; // units per second
        float fall_rate_; // units per second
        float value_ = 0.0f;
    };
}
