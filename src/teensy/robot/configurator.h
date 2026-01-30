#pragma once

#include "ros/subscription.h"

namespace robot {

class Robot;

class Configurator {
public:
  explicit Configurator(Robot &robot, ros::Node &node, std::string_view topic_name);

  auto &subscription() { return subscription_; }

private:
  Robot &robot_;
  ros::Subscription<std::string_view> subscription_;
};
} // namespace robot