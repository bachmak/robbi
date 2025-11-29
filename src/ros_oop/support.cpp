#include "ros_oop/support.h"

#include "utils/rcl.h"

#include <rclc/init.h>
#include <rclc/node.h>

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
}