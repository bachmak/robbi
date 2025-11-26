#pragma once

#include "robot_utils.h"
#include "common_utils.h"

#include <vector>
#include <variant>

using Args = std::vector<std::string>;

struct ActionType
{
    struct Move
    {
        Speed speed{1};

        void populate(Args &&args)
        {
            if (args.size() > 0)
            {
                speed = Speed{common_utils::str_to_float(args[0], speed.v)};
            }
        };
    };

    struct GoStraight
    {
        Meter distance{1};
        Move base{};

        void populate(const Args &args)
        {
            if (args.size() > 0)
            {
                distance = Meter{common_utils::str_to_float(args[0], distance.v)};
                base.populate(common_utils::remove_first(args));
            }
        }
    };

    struct GoForward : GoStraight
    {
        void apply(Robot &robot) const
        {
            robot_utils::move_fwd(robot, distance, base.speed);
        }

        std::string to_string() const { return "GO_FORWARD"; }
    };

    struct GoBackward : GoStraight
    {
        void apply(Robot &robot) const
        {
            robot_utils::move_bwd(robot, distance, base.speed);
        }

        std::string to_string() const { return "GO_BACKWARD"; }
    };

    struct Turn
    {
        Degree angle{90};
        Move base{};

        void populate(const Args &args)
        {
            if (args.size() > 0)
            {
                angle = Degree{common_utils::str_to_float(args[0], angle.v)};
                base.populate(common_utils::remove_first(args));
            }
        }
    };

    struct TurnLeft : Turn
    {
        void apply(Robot &robot) const
        {
            robot_utils::rotate_left(robot, angle, base.speed);
        }
    };

    struct TurnRight : Turn
    {
        void apply(Robot &robot) const
        {
            robot_utils::rotate_right(robot, angle, base.speed);
        }
    };

    struct BackAndForth : GoStraight
    {
        void apply(Robot &robot) const
        {
            robot_utils::move_fwd(robot, distance, base.speed);
            robot_utils::move_bwd(robot, distance, base.speed);
        }
    };

    struct Square
    {
        Meter side{0.4};
        bool return_back = true;
        Move base{};

        void populate(const Args &args)
        {
            if (args.size() > 0)
            {
                side = Meter{common_utils::str_to_float(args[0], side.v)};
            }
            if (args.size() > 1)
            {
                return_back = common_utils::str_to_bool(args[1], return_back);
            }
            if (args.size() > 2)
            {
                base.populate(common_utils::remove_first(args, 2));
            }
        }

        void apply(Robot &robot) const
        {
            for (int i = 0; i < 3; i++)
            {
                robot_utils::move_fwd(robot, side, base.speed);
                robot_utils::rotate_left(robot, Degree{90}, base.speed);
            }

            robot_utils::move_fwd(robot, side, base.speed);

            if (!return_back)
            {
                return;
            }

            for (int i = 0; i < 3; i++)
            {
                robot_utils::move_bwd(robot, side, base.speed);
                robot_utils::rotate_right(robot, Degree{90}, base.speed);
            }

            robot_utils::move_bwd(robot, side, base.speed);
        }
    };

    struct X
    {
        Meter side{0.4};
        Move base{};

        void populate(const Args &args)
        {
            if (args.size() > 0)
            {
                side = Meter{common_utils::str_to_float(args[0], side.v)};
                base.populate(common_utils::remove_first(args));
            }
        }

        void apply(Robot &robot) const
        {
            const auto diag = Meter{side * sqrt(2.0f)};

            robot_utils::rotate_left(robot, Degree{45}, base.speed);
            robot_utils::move_fwd(robot, diag, base.speed);
            robot_utils::rotate_right(robot, Degree{135}, base.speed);
            robot_utils::move_fwd(robot, side, base.speed);
            robot_utils::rotate_right(robot, Degree{135}, base.speed);
            robot_utils::move_fwd(robot, diag, base.speed);
            robot_utils::rotate_left(robot, Degree{135}, base.speed);
            robot_utils::move_fwd(robot, side, base.speed);
            robot_utils::rotate_left(robot, Degree{90}, base.speed);
        }
    };

    struct Custom
    {
        Degree left_wheel_angle{360};
        Degree right_wheel_angle{360};
        Move base{};

        void populate(const Args &args)
        {
            if (args.size() > 0)
            {
                left_wheel_angle = Degree{common_utils::str_to_float(args[0], left_wheel_angle.v)};
                right_wheel_angle = left_wheel_angle;
            }
            if (args.size() > 1)
            {
                right_wheel_angle = Degree{common_utils::str_to_float(args[0], right_wheel_angle.v)};
            }
            if (args.size() > 2)
            {
                base.populate(common_utils::remove_first(args, 2));
            }
        }

        void apply(Robot &robot) const
        {
            robot.rotate(left_wheel_angle, right_wheel_angle, base.speed);
        }
    };

    struct None
    {
        void populate(const Args &) {}
        void apply(Robot &robot) const {}
    };
};

using Action = std::variant<
    ActionType::GoForward,
    ActionType::GoBackward,
    ActionType::TurnLeft,
    ActionType::TurnRight,
    ActionType::BackAndForth,
    ActionType::Square,
    ActionType::X,
    ActionType::Custom,
    ActionType::None>;

std::string to_string(const Action &action)
{
    using A = ActionType;

    return std::visit(common_utils::overloads{
                          [](const A::GoForward &)
                          { return "GO_FORWARD"; },
                          [](const A::GoBackward &)
                          { return "GO_BACKWARD"; },
                          [](const A::TurnLeft &)
                          { return "TURN_LEFT"; },
                          [](const A::TurnRight &)
                          { return "TURN_RIGHT"; },
                          [](const A::BackAndForth &)
                          { return "BACK_AND_FORTH"; },
                          [](const A::Square &)
                          { return "SQUARE"; },
                          [](const A::X &)
                          { return "X"; },
                          [](const A::Custom &)
                          { return "CUSTOM"; },
                          [](const A::None &)
                          { return "NONE"; },
                      },
                      action);
}
