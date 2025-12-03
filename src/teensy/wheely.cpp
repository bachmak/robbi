#include "wheely.h"

#include "common_utils.h"
#include "io_utils.h"

namespace
{
    struct MotorSpeeds
    {
        DegSec left;
        DegSec right;
    };

    MotorSpeeds to_motor_speeds(
        const geo_utils::Twist &twist,
        const WheelySettings &settings)
    {
        const auto v = twist.linear.x;
        const auto w = twist.angular.z;

        const auto v_l = v - settings.width.v / 2 * w;
        const auto v_r = v + settings.width.v / 2 * w;

        const auto w_l = v_l / settings.left.radius.v;
        const auto w_r = -v_r / settings.right.radius.v;

        const auto w_l_deg = DegSec{w_l * 180 / M_PI};
        const auto w_r_deg = DegSec{w_r * 180 / M_PI};

        return {
            .left = w_l_deg,
            .right = w_r_deg,
        };
    }
}

Wheely::Wheely(const WheelySettings &settings)
    : settings_(settings),
      left_(settings_.left.motor),
      right_(settings_.right.motor)
{
}

void Wheely::update(Us dt)
{
    left_.update(dt);
    right_.update(dt);
}

void Wheely::set_target_speed(const geo_utils::Twist &twist)
{
    const auto speeds = to_motor_speeds(twist, settings_);

    io_utils::debug(
        "Setting new speed: left = %.2f deg/sec, right = %.2f deg/sec",
        speeds.left.v,
        speeds.right.v);

    left_.set_target_speed(speeds.left);
    right_.set_target_speed(speeds.right);
}

void Wheely::set_stop(bool value)
{
    io_utils::debug("Setting stop: stop = %s", value ? "true" : "false");

    left_.set_stop(value);
    right_.set_stop(value);
}

void Wheely::configure(std::string_view setting, float value)
{
    using common_utils::substr_after;

    auto configure_wheel = [&](WheelSettings &settings, Motor &motor, std::string_view subsetting)
    {
        if (auto motor_setting = substr_after(subsetting, "motor."))
        {
            return motor.configure(*motor_setting, value);
        }
        if (subsetting == "radius")
        {
            settings.radius = Meter{static_cast<int>(value)};
        }
    };

    if (auto subsetting = common_utils::substr_after(setting, "left."))
    {
        configure_wheel(settings_.left, left_, *subsetting);
    }
    else if (auto subsetting = common_utils::substr_after(setting, "right."))
    {
        configure_wheel(settings_.right, right_, *subsetting);
    }
    else if (setting == "width")
    {
        settings_.width = Meter{value};
    }
    else
    {
        io_utils::error("Wheely: unknown setting: %s", setting.data());
    }
}