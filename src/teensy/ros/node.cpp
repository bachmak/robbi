#include "ros/node.h"

#include "ros/support.h"
#include "utils/rcl.h"

#include <rclc/publisher.h>
#include <rclc/subscription.h>

namespace ros
{
    Node::Node(Support &support, const char *name) : impl_{}
    {
        support.init(impl_, name);
    }

    void Node::init(rcl_publisher_t &publisher,
                    const char *topic_name,
                    const rosidl_message_type_support_t *type_support)
    {
        RCC_CHECK(rclc_publisher_init_default(
            &publisher,
            &impl_,
            type_support,
            topic_name));
    }

    void Node::init(rcl_subscription_t &subscription,
                    const char *topic_name,
                    const rosidl_message_type_support_t *type_support)
    {
        RCC_CHECK(rclc_subscription_init_default(
            &subscription,
            &impl_,
            type_support,
            topic_name));
    }

    void Node::finalize(rcl_publisher_t &publisher)
    {
        RCC_CHECK(rcl_publisher_fini(&publisher, &impl_));
    }

    void Node::finalize(rcl_subscription_t &subscription)
    {
        RCC_CHECK(rcl_subscription_fini(&subscription, &impl_));
    }
}