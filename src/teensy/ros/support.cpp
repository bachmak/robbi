#include "ros/support.h"

#include "utils/rcl.h"

#include <rclc/init.h>
#include <rclc/node.h>
#include <rclc/timer.h>
#include <rclc/executor.h>

namespace ros
{
    Support::Support() : allocator_(rcl_get_default_allocator())
    {
        RCC_CHECK(rclc_support_init(&impl_, 0, nullptr, &allocator_));
    }

    Support::~Support()
    {
        RCC_CHECK(rclc_support_fini(&impl_));
    }

    void Support::init(rcl_node_t &node, const char *name)
    {
        RCC_CHECK(rclc_node_init_default(&node, name, "", &impl_));
    }

    void Support::init(rcl_timer_t &timer,
                       int64_t timeout_ns,
                       rcl_timer_callback_t callback)
    {
        RCC_CHECK(rclc_timer_init_default(
            &timer,
            &impl_,
            timeout_ns,
            callback));
    }

    void Support::init(rclc_executor_t &executor, size_t num_handles)
    {
        RCC_CHECK(rclc_executor_init(
            &executor,
            &impl_.context,
            num_handles,
            &allocator_));
    }

    void Support::finalize(rcl_timer_t &timer)
    {
        RCC_CHECK(rcl_timer_fini(&timer));
    }

    void Support::finalize(rclc_executor_t &executor)
    {
        RCC_CHECK(rclc_executor_fini(&executor));
    }
}