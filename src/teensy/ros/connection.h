#pragma once

#include "types.h"

namespace utils::time
{
    class Timer;
}

namespace ros
{
    bool is_connected(utils::time::Timer &timer, Ms ping_timeout);
    bool is_disconnected(utils::time::Timer &timer, Ms ping_timeout);
}