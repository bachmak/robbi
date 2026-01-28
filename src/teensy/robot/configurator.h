#pragma once

#include "ros/subscription.h"
#include "robot/robot.h"
#include "utils/str.h"
#include "utils/io.h"

namespace robot
{
    class Configurator
    {
    public:
        explicit Configurator(Robot &robot, ros::Node &node, const char *topic_name)
            : robot_(robot),
              subscription_(node,
                            topic_name,
                            [this](std::string_view str)
                            { configure(str); })
        {
        }

        auto &subscription() { return subscription_; }

    private:
        void configure(std::string_view str)
        {
            utils::io::debug(
                "RobotConfiguration: received message: %.*s",
                static_cast<int>(str.size()),
                str.data());

            const auto tokens = utils::str::split(str);
            if (tokens.size() != 2)
            {
                return;
            }

            const auto setting = tokens[0];
            const auto value = utils::str::str_to_float(tokens[1]);
            if (!value.has_value())
            {
                return;
            }

            utils::io::info(
                "RobotConfiguration: applying setting: %.*s = %f",
                static_cast<int>(setting.size()),
                setting.data(),
                *value);

            robot_.configure(setting, *value);
        }

    private:
        Robot &robot_;
        ros::Subscription<std::string_view> subscription_;
    };
}