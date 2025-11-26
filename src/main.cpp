#include "robot.h"
#include "common_utils.h"
#include "io_utils.h"
#include "robot_utils.h"

#include <Arduino.h>

#include <unordered_map>
#include <string>

enum class Action
{
  UNKNOWN,
  GO_FORWARD,
  GO_BACKWARD,
  TURN_LEFT,
  TURN_RIGHT,
  BACK_AND_FORTH,
  SQUARE,
  CUSTOM,
};

const char *to_string(Action action)
{
  switch (action)
  {
  case Action::UNKNOWN:
    return "UNKNOWN";
  case Action::GO_FORWARD:
    return "GO_FORWARD";
  case Action::GO_BACKWARD:
    return "GO_BACKWARD";
  case Action::TURN_LEFT:
    return "TURN_LEFT";
  case Action::TURN_RIGHT:
    return "TURN_RIGHT";
  case Action::BACK_AND_FORTH:
    return "BACK_AND_FORTH";
  case Action::SQUARE:
    return "SQUARE";
  case Action::CUSTOM:
    return "CUSTOM";
  }

  return "";
}

struct ActionInfo
{
  Action action = Action::UNKNOWN;
  float default_param_value = 0.0;
};

struct Config
{
  int serial_baud = 9600;
  Ms delay_after_io_init{2000};
  io_utils::LogLevel log_level = io_utils::LogLevel::DEBUG;

  std::unordered_map<std::string, ActionInfo> actions = {
      {"f", {Action::GO_FORWARD, 2}},
      {"b", {Action::GO_BACKWARD, 2}},
      {"l", {Action::TURN_LEFT, 90.0}},
      {"r", {Action::TURN_RIGHT, 90.0}},
      {"bf", {Action::BACK_AND_FORTH}},
      {"s", {Action::SQUARE}},
      {"c", {Action::CUSTOM}},
  };

  RobotSettings robot_settings = {
      .left_wheel_settings = {
          .control_pin = Pin{5},
          .feedback_pin = Pin{6},
          .speed_dead_range = Speed{4},
          .circumference = Meter{0.205},
          .label = "right-wheel",
      },
      .right_wheel_settings = {
          .control_pin = Pin{7},
          .feedback_pin = Pin{8},
          .speed_dead_range = Speed{4},
          .circumference = Meter{0.205},
          .label = "left-wheel",
      },
      .width = Meter{0.102},
  };
};

ActionInfo str_to_action_info(const std::string &action_name, const Config &config)
{
  if (auto it = config.actions.find(action_name); it != config.actions.end())
  {
    return it->second;
  }

  return {};
}

void do_loop(Robot &robot, const Config &config)
{
  const auto input = common_utils::trim(io_utils::get_string());
  const auto tokens = common_utils::split(input);

  if (tokens.size() < 1)
  {
    io_utils::error("Not enough arguments");
    return;
  }

  const auto action_name = tokens.front();
  const auto action_info = str_to_action_info(action_name, config);

  auto get_float_token = [&](size_t idx, float default_value = 0.0f)
  {
    if (default_value == 0.0f)
    {
      default_value = action_info.default_param_value;
    }

    if (tokens.size() > idx)
    {
      return common_utils::str_to_float(tokens[idx])
          .value_or(default_value);
    }

    return default_value;
  };

  io_utils::info("Received: input: %s, command: %s. Action: %s", input.c_str(), action_name.c_str(), to_string(action_info.action));

  switch (action_info.action)
  {
  case Action::UNKNOWN:
    break;
  case Action::GO_FORWARD:
    robot_utils::move_fwd(robot, Meter{get_float_token(1)});
    break;
  case Action::GO_BACKWARD:
    robot_utils::move_bwd(robot, Meter{get_float_token(1)});
    break;
  case Action::TURN_LEFT:
    robot_utils::rotate_left(robot, Degree{get_float_token(1)});
    break;
  case Action::TURN_RIGHT:
    robot_utils::rotate_right(robot, Degree{get_float_token(1)});
    break;
  case Action::BACK_AND_FORTH:
    robot_utils::move_fwd(robot, Meter{get_float_token(1, 1)});
    robot_utils::move_bwd(robot, Meter{get_float_token(2, 1)});
    break;
  case Action::SQUARE:
  {
    const auto straight = Meter{get_float_token(1, 0.4)};
    const auto turn = Degree{get_float_token(2, 90)};

    for (int i = 0; i < 4; i++)
    {
      robot_utils::move_fwd(robot, straight);
      robot_utils::rotate_left(robot, turn);
    }
    for (int i = 0; i < 4; i++)
    {
      robot_utils::rotate_right(robot, turn);
      robot_utils::move_bwd(robot, straight);
    }
    break;
  }
  case Action::CUSTOM:
  {
    const auto left_wheel_rotation = Degree{get_float_token(1, 200)};
    const auto right_wheel_rotation = Degree{get_float_token(2, 200)};
    const auto speed = Speed{get_float_token(3, 1)};
    robot.rotate(left_wheel_rotation, right_wheel_rotation, speed);
    break;
  }
  }
}

void setup()
{
  // inital setup in loop, to avoid using global state
}

void loop()
{
  const auto config = Config{};
  auto robot = Robot(config.robot_settings);

  io_utils::init(config.serial_baud, config.log_level);
  delay(config.delay_after_io_init.count());
  io_utils::debug("Initialization complete");

  while (true)
  {
    do_loop(robot, config);
  }
}