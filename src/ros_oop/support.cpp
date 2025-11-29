#include "ros_oop/support.h"

#include "utils/rcl.h"

#include <rclc/init.h>
#include <rclc/node.h>
#include <rclc/timer.h>
#include <rclc/executor.h>

namespace ros
{

    Support &Support::get_instance()
    {
        static auto instance = Support{};
        return instance;
    }

    Support::Support() : allocator_(rcl_get_default_allocator())
    {
        rcc_check(rclc_support_init(&impl_, 0, nullptr, &allocator_));
    }

    Support::~Support()
    {
        rcc_check(rclc_support_fini(&impl_));
    }

    void Support::init(rcl_node_t &node, const char *name)
    {
        rcc_check(rclc_node_init_default(&node, name, "", &impl_));
    }

    void Support::init(rcl_timer_t &timer,
                       int64_t timeout_ns,
                       rcl_timer_callback_t callback)
    {
        rcc_check(rclc_timer_init_default(
            &timer,
            &impl_,
            timeout_ns,
            callback));
    }

    void Support::init(rclc_executor_t &executor, size_t num_handles)
    {
        rcc_check(rclc_executor_init(
            &executor,
            &impl_.context,
            num_handles,
            &allocator_));
    }

    void Support::finalize(rcl_timer_t &timer)
    {
        rcc_check(rcl_timer_fini(&timer));
    }

    void Support::finalize(rclc_executor_t &executor)
    {
        rcc_check(rclc_executor_fini(&executor));
    }
}