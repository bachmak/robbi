#pragma once

#include "types.h"

namespace utils::time
{
    class Timer;
}

namespace utils::connection
{
    bool is_connected(time::Timer &timer, Ms ping_timeout);
    bool is_disconnected(time::Timer &timer, Ms ping_timeout);
}