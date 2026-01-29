#include "ros/node.h"

#include "ros/check_err.h"
#include "ros/support.h"

#include <rclc/publisher.h>
#include <rclc/subscription.h>

namespace ros {

Node::Node(Support &support, std::string_view name) : impl_{} { support.init(impl_, name); }

void Node::init(rcl_publisher_t &publisher, std::string_view topic_name,
                const rosidl_message_type_support_t *type_support) {
  ROS_CHECK_ERR(rclc_publisher_init_default(&publisher, &impl_, type_support, topic_name.data()));
}

void Node::init(rcl_subscription_t &subscription, std::string_view topic_name,
                const rosidl_message_type_support_t *type_support) {
  ROS_CHECK_ERR(
      rclc_subscription_init_default(&subscription, &impl_, type_support, topic_name.data()));
}

void Node::finalize(rcl_publisher_t &publisher) {
  ROS_CHECK_ERR(rcl_publisher_fini(&publisher, &impl_));
}

void Node::finalize(rcl_subscription_t &subscription) {
  ROS_CHECK_ERR(rcl_subscription_fini(&subscription, &impl_));
}
} // namespace ros