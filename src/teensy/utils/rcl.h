#pragma once

#include "io_utils.h"

#include <rcl/types.h>

namespace ros
{
    inline void rcc_check(rcl_ret_t ret, const char *file, int line)
    {
        if (ret != RCL_RET_OK)
        {
            io_utils::error("%s:%d: error: %d. Endless loop entered", ret, file, line);
            while (1)
            {
            }
        }
    }

#define RCC_CHECK(expr)                      \
    do                                       \
    {                                        \
        rcc_check(expr, __FILE__, __LINE__); \
    } while (0)
}
