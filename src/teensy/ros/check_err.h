#pragma once

#include "utils/io.h"

#include <rcl/types.h>

namespace ros {

inline void check_err(rcl_ret_t ret, const char *file, int line) {
  if (ret != RCL_RET_OK) {
    utils::io::error("%s:%d: error: %d. Endless loop entered", ret, file, line);
    while (1) {
    }
  }
}

#define ROS_CHECK_ERR(expr)                                                                        \
  do {                                                                                             \
    ::ros::check_err(expr, __FILE__, __LINE__);                                                    \
  } while (0)
} // namespace ros
