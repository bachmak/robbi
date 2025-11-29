#pragma once

#include "utils/non_copyable.h"

#include <rclc/types.h>
#include <rcl/allocator.h>
#include <rcl/node.h>
#include <rcl/timer.h>
#include <rclc/executor.h>

namespace ros
{
    class Support
    {
    public:
        static Support &get_instance();
        NON_COPYABLE(Support)

        void init(rcl_node_t &node, const char *name);
        void init(rcl_timer_t &timer,
                  int64_t timeout_ns,
                  rcl_timer_callback_t callback);
        void init(rclc_executor_t &executor, size_t num_handles);

        void finalize(rcl_timer_t &timer);
        void finalize(rclc_executor_t &executor);

    private:
        Support();
        ~Support();

        rclc_support_t impl_;
        rcl_allocator_t allocator_;
    };
}