#pragma once

#include "ros/subscription.h"

namespace robot {

class Robot;

class Controller {
public:
  Controller(Robot &robot, ros::Node &node, std::string_view cmd_vel_topic,
             std::string_view cmd_action_topic);

  auto &cmd_vel_subscription() { return cmd_vel_subscription_; }
  auto &cmd_action_subscription() { return cmd_action_subscription_; }

private:
  void apply_action(std::string_view cmd);

private:
  Robot &robot_;
  ros::Subscription<utils::geometry::Twist> cmd_vel_subscription_;
  ros::Subscription<std::string_view> cmd_action_subscription_;
};

} // namespace robot