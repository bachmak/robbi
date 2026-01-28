#include "ros/executor.h"

#include "ros/timer.h"
#include "ros/subscription.h"
#include "utils/visitor.h"
#include "ros/support.h"
#include "ros/check_err.h"

namespace ros
{
    using utils::visitor::overloads;

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
                        ROS_CHECK_ERR(rclc_executor_add_timer(&impl_, &timer->impl()));
                    },
                    [this](SubscriptionBase *subscription)
                    {
                        auto callback = [](const void *msg, void *context)
                        {
                            auto *subscription = static_cast<SubscriptionBase *>(context);
                            subscription->on_message(msg);
                        };

                        auto context = static_cast<void *>(subscription);

                        ROS_CHECK_ERR(rclc_executor_add_subscription_with_context(
                            &impl_,
                            &subscription->impl(),
                            subscription->msg(),
                            callback,
                            context,
                            ON_NEW_DATA));
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
                                     ROS_CHECK_ERR(rclc_executor_remove_timer(&impl_, &timer->impl()));
                                 },
                                 [this](SubscriptionBase *subscription)
                                 {
                                     ROS_CHECK_ERR(rclc_executor_remove_subscription(&impl_, &subscription->impl()));
                                 }},
                       executable);
        }

        support_.finalize(impl_);
    }

    void Executor::spin_some(Ns timeout)
    {
        ROS_CHECK_ERR(rclc_executor_spin_some(&impl_, timeout.count()));
    }
}
