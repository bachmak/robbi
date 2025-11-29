#pragma once

#include <rcl/types.h>

namespace ros
{
    inline void error_loop()
    {
        while (1)
        {
        }
    }

    inline void rcc_check(rcl_ret_t ret)
    {
        if (ret != RCL_RET_OK)
        {
            error_loop();
        }
    }
}