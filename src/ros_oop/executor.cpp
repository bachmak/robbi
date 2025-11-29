#include "ros_oop/executor.h"

#include "ros_oop/timer.h"
#include "common_utils.h"
#include "ros_oop/support.h"
#include "utils/rcl.h"
#include "executor.h"

namespace ros
{
    using common_utils::overloads;

    Executor::Executor(Support &support, std::vector<Executable> executables)
        : impl_{}, support_{support}, executables_{std::move(executables)}
    {
        support.init(impl_, executables_.size());

        for (const auto &executable : executables_)
        {
            std::visit(
                overloads{
                    [this](Timer *timer)
                    {
                        rcc_check(rclc_executor_add_timer(&impl_, &timer->impl()));
                    }},
                executable);
        }
    }

    Executor::~Executor()
    {
        for (const auto &executable : executables_)
        {
            std::visit(overloads{[this](Timer *timer)
                                 {
                                     rcc_check(rclc_executor_remove_timer(&impl_, &timer->impl()));
                                 }},
                       executable);
        }

        support_.finalize(impl_);
    }

    void Executor::spin_some(Ns timeout)
    {
        rcc_check(rclc_executor_spin_some(&impl_, timeout.count()));
    }
}
