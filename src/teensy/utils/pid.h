#pragma once

#include "common.h"

namespace utils::control
{

    struct PIDSettings
    {
        float G;
        float T_i;
        float T_d;
        float out_min;
        float out_max;
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

    private:
        PIDSettings settings_;
        PIDParams params_;
        float integrator_;
        float prev_error_;
        bool first_run_ = true;
    };
}
