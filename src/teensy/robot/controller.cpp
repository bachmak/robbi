#include "robot/controller.h"

#include "controller.h"
#include "robot/robot.h"
#include "utils/str.h"

namespace robot {

Controller::Controller(Robot &robot, ros::Node &node, std::string_view cmd_vel_topic,
                       std::string_view cmd_action_topic)
    : robot_{robot}, //
      cmd_vel_subscription_{
          node, cmd_vel_topic.data(),
          [this](const utils::geometry::Twist &twist) { robot_.set_target_speed(twist); }},
      cmd_action_subscription_{node, cmd_action_topic.data(),
                               [this](std::string_view cmd) { apply_action(cmd); }} {}

void Controller::apply_action(std::string_view cmd) {
  const auto tokens = utils::str::split(cmd);
  if (tokens.size() < 0) {
    return;
  }

  const auto action = tokens[0];
  if (action == "stop") {
    return robot_.set_stop(true);
  }
  if (action == "go") {
    return robot_.set_stop(false);
  }
  if (tokens.size() < 2) {
    return;
  }

  const auto param_1 = utils::str::to_float(tokens[1]);
  const auto param_2 = utils::str::to_float(tokens[2]);
  if (!param_1.has_value() || !param_2.has_value()) {
    return;
  }

  const auto duration = Us{static_cast<int64_t>(*param_2 * 1'000'000)};
  if (action == "move") {
    return robot_.set_target_distance(Meter{*param_1}, duration);
  }
  if (action == "rotate") {
    return robot_.set_target_rotation(Degree{*param_1}, duration);
  }
}
} // namespace robot
