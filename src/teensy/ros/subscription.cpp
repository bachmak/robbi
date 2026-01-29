#include "ros/subscription.h"

#include "ros/node.h"

namespace ros {

SubscriptionBase::SubscriptionBase(Node &node, std::string_view topic_name, void *message,
                                   const rosidl_message_type_support_t *type_support,
                                   Callback callback)
    : node_(node), message_(message), callback_(std::move(callback)) {
  node.init(impl_, topic_name, type_support);
}

SubscriptionBase::~SubscriptionBase() { node_.finalize(impl_); }

void SubscriptionBase::on_message(const void *message) { callback_(message); }
} // namespace ros