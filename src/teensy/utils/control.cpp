#include "utils/control.h"

#include "io_utils.h"

#include <utility>
#include <algorithm>

namespace utils::control
{
    namespace
    {
        PIDParams to_params(const PIDSettings &settings)
        {
            return {
                .K_p = settings.G,
                .K_i = settings.T_i > 0.0f ? settings.G / settings.T_i : 0.0f,
                .K_d = settings.G * settings.T_d,
            };
        }
    }

    PID::PID(const PIDSettings &settings) : settings_(settings), params_(to_params(settings)) {}

    float PID::update(const float setpoint,
                      const float measurement,
                      const float dt)
    {
        const auto error = setpoint - measurement;
        const auto prev_error = std::exchange(prev_error_, error);

        const auto P = params_.K_p * error;
        const auto I = params_.K_i * integrator_;

        const auto D = [&]
        {
            if (std::exchange(first_run_, false))
            {
                return 0.0f;
            }
            return params_.K_d * (error - prev_error) / dt;
        }();

        const auto output = P + I + D;
        const auto pushing_saturation =
            (output > settings_.out_max && error > 0) ||
            (output < settings_.out_min && error < 0);

        if (!pushing_saturation)
        {
            integrator_ += error * dt;
        }

        return std::clamp(output, settings_.out_min, settings_.out_max);
    }

    void PID::configure(std::string_view setting, float value)
    {
        if (setting == "g")
        {
            settings_.G = value;
        }
        else if (setting == "t-i")
        {
            settings_.T_i = value;
        }
        else if (setting == "t-d")
        {
            settings_.T_d = value;
        }
        else if (setting == "out-min")
        {
            settings_.out_min = value;
        }
        else if (setting == "out-max")
        {
            settings_.out_max = value;
        }
        else
        {
            io_utils::error("PID: unknown setting: %s", setting.data());
        }

        params_ = to_params(settings_);
    }
}