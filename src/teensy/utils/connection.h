#pragma once

#include "common.h"

namespace time_utils
{
    class Timer;
}

namespace utils::connection
{
    bool is_connected(time_utils::Timer &timer, Ms ping_timeout);
    bool is_disconnected(time_utils::Timer &timer, Ms ping_timeout);
}