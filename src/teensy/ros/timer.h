#pragma once

#include "ros/executor.h"
#include "types.h"
#include "utils/non_copyable.h"

#include <rcl/timer.h>

#include <functional>

namespace ros {

class Support;

class Timer {
public:
  using Callback = std::function<void(int64_t)>;

public:
  explicit Timer(Support &support, Ns period, Callback callback);
  ~Timer();

  NON_COPYABLE(Timer)

public:
  rcl_timer_t &impl() { return impl_; }

private:
  std::function<void(int64_t)> callback_;
  rcl_timer_t impl_;
  Support &support_;
};
} // namespace ros