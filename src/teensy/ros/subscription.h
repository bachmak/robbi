#pragma once

#include "ros/message_traits.h"
#include "utils/non_copyable.h"

#include <rcl/subscription.h>

#include <functional>

namespace ros {

class Node;

class SubscriptionBase {
public:
  NON_COPYABLE(SubscriptionBase)
  using Callback = std::function<void(const void *)>;

  SubscriptionBase(Node &node, const char *topic_name, void *message,
                   const rosidl_message_type_support_t *type_support, Callback callback);
  ~SubscriptionBase();

  rcl_subscription_t &impl() { return impl_; }
  void *msg() { return message_; }
  void on_message(const void *message);

private:
  rcl_subscription_t impl_;
  Node &node_;
  void *message_;
  Callback callback_;
};

template <typename T> class Subscription {
public:
  using Traits = MessageTraits<T>;
  using RclMessageType = typename Traits::RclMessageType;
  using Callback = std::function<void(T)>;

  NON_COPYABLE(Subscription)

  Subscription(Node &node, const char *topic_name, Callback callback)
      : dummy_{(Traits::init(&message_), false)},
        base_{node, topic_name, static_cast<void *>(&message_), Traits::get_type_support(),
              [this](const void *void_msg) {
                const auto *rcl_msg = static_cast<const RclMessageType *>(void_msg);
                callback_(Traits::to_original_type(rcl_msg));
              }},
        callback_(std::move(callback)) {}

  ~Subscription() { Traits::finalize(&message_); }

  SubscriptionBase &base() { return base_; }

private:
  RclMessageType message_;
  bool dummy_;
  SubscriptionBase base_;
  Callback callback_;
};
} // namespace ros