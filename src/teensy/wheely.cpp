#include "wheely.h"

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
        const auto w_r = v_r / settings.right.radius.v;

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
    left_.set_target_speed(speeds.left);
    right_.set_target_speed(speeds.right);
}
