#include "ros_oop/timer.h"

#include "ros_oop/support.h"
#include "utils/rcl.h"

#include <rclc/timer.h>

namespace ros
{
    static auto timers = std::vector<Timer *>{};

    Timer::Timer(Support &support, Ns period, Callback callback)
        : callback_{callback}, impl_{}, support_{support}
    {
        timers.push_back(this);

        auto rcl_callback =
            [](rcl_timer_t *timer, int64_t last_call_time)
        {
            auto it = std::find_if(
                timers.begin(),
                timers.end(),
                [timer](Timer *t)
                { return &t->impl_ == timer; });

            if (it == timers.end())
            {
                return;
            }

            auto &self = **it;
            self.callback_(last_call_time);
        };

        support.init(impl_, period.count(), rcl_callback);
    }

    Timer::~Timer()
    {
        support_.finalize(impl_);

        timers.erase(
            std::remove(
                timers.begin(),
                timers.end(),
                this),
            timers.end());
    }
}