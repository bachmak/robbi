#pragma once

#include "utils/non_copyable.h"

#include <rcl/node.h>

namespace ros
{
    class Support;

    class Node
    {
    public:
        Node(Support &support, const char *name);
        NON_COPYABLE(Node)

    private:
        rcl_node_t impl_;
    };
}