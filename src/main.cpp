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
  io_utils::LogLevel log_level = io_utils::LogLevel::DEBUG;

  std::unordered_map<std::string, ActionInfo> actions = {
      {"f", {Action::GO_FORWARD, 0.1}},
      {"b", {Action::GO_BACKWARD, 0.1}},
      {"l", {Action::TURN_LEFT, 90.0}},
      {"r", {Action::TURN_RIGHT, 90.0}},
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

Wheels create_wheels()
{
  auto left_wheel = []
  {
    auto settings = WheelSettings{};
    settings.control_pin = Pin{7};
    settings.feedback_pin = Pin{8};
    settings.speed_dead_range = Speed{1};
    settings.label = "left-wheel";
    return Wheel(settings);
  }();

  auto right_wheel = []
  {
    auto settings = WheelSettings{};
    settings.control_pin = Pin{5};
    settings.feedback_pin = Pin{6};
    settings.speed_dead_range = Speed{1};
    settings.label = "right-wheel";
    return Wheel(settings);
  }();

  return Wheels{left_wheel, right_wheel};
}

void do_loop(Robot &robot, const Config &config)
{
  const auto action_str = io_utils::get_string();
  const auto action_info = str_to_action_info(action_str, config);

  const auto param_str = io_utils::try_get_string();
  const auto param = common_utils::str_to_float(param_str, action_info.default_param_value);

  io_utils::info(
      "Received: command: %s, parameter: %s. Action: %s",
      action_str.c_str(),
      param_str.value_or("").c_str(),
      to_string(action_info.action));

  switch (action_info.action)
  {
  case Action::UNKNOWN:
    break;
  case Action::GO_FORWARD:
    robot_utils::move_fwd(robot, Meter{param});
    break;
  case Action::GO_BACKWARD:
    robot_utils::move_bwd(robot, Meter{param});
    break;
  case Action::TURN_LEFT:
    robot_utils::rotate_left(robot, Degree{param});
    break;
  case Action::TURN_RIGHT:
    robot_utils::rotate_right(robot, Degree{param});
    break;
  }
}

void setup()
{
  // inital setup in loop, to avoid using global state
}

void loop()
{
  auto config = Config{};
  auto wheels = create_wheels();
  auto robot = Robot(wheels);

  io_utils::init(config.serial_baud, config.log_level);
  io_utils::debug("Initialization complete");

  while (true)
  {
    do_loop(robot, config);
  }
}