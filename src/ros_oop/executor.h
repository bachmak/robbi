#pragma once

#include "utils/non_copyable.h"
#include "ros_oop/timer.h"

#include <rclc/executor.h>

#include <variant>
#include <vector>

namespace ros
{
    class Support;
    class Timer;

    using Executable = std::variant<Timer *>;

    class Executor
    {
    public:
        NON_COPYABLE(Executor)

        Executor(Support &support, std::vector<Executable> executables);
        ~Executor();

        void spin_some(Ns timeout);

    private:
        rclc_executor_t impl_;
        Support &support_;
        std::vector<Executable> executables_;
    };
}