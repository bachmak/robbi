#pragma once

#include "ros/timer.h"
#include "types.h"
#include "utils/non_copyable.h"

#include <rclc/executor.h>

#include <variant>
#include <vector>

namespace ros {

class Support;
class Timer;
class SubscriptionBase;

using Executable = std::variant<Timer *, SubscriptionBase *>;

class Executor {
public:
  explicit Executor(Support &support, std::vector<Executable> executables);
  ~Executor();

  NON_COPYABLE(Executor)

public:
  void spin_some(Ns timeout);

private:
  rclc_executor_t impl_;
  Support &support_;
  std::vector<Executable> executables_;
};
} // namespace ros