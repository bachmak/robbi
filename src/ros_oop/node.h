#pragma once

#include "utils/non_copyable.h"

#include <rcl/node.h>
#include <rcl/publisher.h>

namespace ros
{
    class Support;

    class Node
    {
    public:
        Node(Support &support, const char *name);
        NON_COPYABLE(Node)

        void init(rcl_publisher_t &publisher,
                  const char *topic_name,
                  const rosidl_message_type_support_t *type_support);

        void finalize(rcl_publisher_t &publisher);

    private:
        rcl_node_t impl_;
    };
}