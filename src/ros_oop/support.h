#pragma once

#include "utils/non_copyable.h"

#include <rclc/types.h>
#include <rcl/allocator.h>
#include <rcl/node.h>

namespace ros
{
    class Support
    {
    public:
        static Support &get_instance();
        NON_COPYABLE(Support)

        void init(rcl_node_t &node, const char *name);

    private:
        Support();
        ~Support();

        rclc_support_t impl_;
        rcl_allocator_t allocator_;
    };
}