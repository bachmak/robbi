#pragma once

#include "ros/check_err.h"
#include "ros/message_traits.h"
#include "ros/node.h"
#include "utils/non_copyable.h"

namespace ros {

template <typename T> class Publisher {
public:
  NON_COPYABLE(Publisher)

  Publisher(Node &node, const char *topic_name) : impl_(), node_(node) {
    node_.init(impl_, topic_name, Traits::get_type_support());
  }

  ~Publisher() { node_.finalize(impl_); }

  using Traits = MessageTraits<T>;

  void publish(const T &data) {
    auto message = Traits::to_message(data);
    ROS_CHECK_ERR(rcl_publish(&impl_, &message, nullptr));
  }

private:
  rcl_publisher_t impl_;
  Node &node_;
};
} // namespace ros