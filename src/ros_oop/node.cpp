#include "ros_oop/node.h"

#include "ros_oop/support.h"
#include "utils/rcl.h"

namespace ros
{
    Node::Node(Support &support, const char *name) : impl_{}
    {
        support.init(impl_, name);
    }
}