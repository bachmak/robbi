#include "node.h"

#include <rclc/init.h>
#include <rcl/allocator.h>

namespace rclc
{
    namespace
    {
    }

    Support &Support::getInstance()
    {
        static auto instance = Support();
        return instance;
    }

    Node Support::create_node(const char *name)
    {
        return Node(support_, name);
    }

    Support::Support() : allocator_(rcl_get_default_allocator())
    {
        rcc_verify(rclc_support_init(&support_, 0, nullptr, &allocator_));
    }

    Node::Node(rclc_support_t &support, const char *name) : impl_{}
    {
        rcc_verify(rclc_node_init_default(&impl_, name, "", &support));
    }
}