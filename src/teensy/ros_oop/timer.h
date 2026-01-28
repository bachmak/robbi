#pragma once

#include "common.h"
#include "utils/non_copyable.h"
#include "ros_oop/executor.h"

#include <rcl/timer.h>

#include <functional>

namespace ros
{
    class Support;

    class Timer
    {
    public:
        NON_COPYABLE(Timer)

        using Callback = std::function<void(int64_t)>;

        Timer(Support &support, Ns period, Callback callback);
        ~Timer();

        rcl_timer_t &impl() { return impl_; }

    private:
        std::function<void(int64_t)> callback_;
        rcl_timer_t impl_;
        Support &support_;
    };
}