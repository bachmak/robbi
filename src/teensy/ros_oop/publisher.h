#pragma once

#include "ros_oop/node.h"
#include "ros_oop/message_traits.h"
#include "utils/non_copyable.h"
#include "utils/rcl.h"

namespace ros
{
    template <typename T>
    class Publisher
    {
    public:
        NON_COPYABLE(Publisher)

        Publisher(Node &node, const char *topic_name) : impl_(), node_(node)
        {
            node_.init(impl_, topic_name, Traits::get_type_support());
        }

        ~Publisher()
        {
            node_.finalize(impl_);
        }

        using Traits = MessageTraits<T>;

        void publish(const T &data)
        {
            auto message = Traits::to_message(data);
            RCC_CHECK(rcl_publish(&impl_, &message, nullptr));
        }

    private:
        rcl_publisher_t impl_;
        Node &node_;
    };
}