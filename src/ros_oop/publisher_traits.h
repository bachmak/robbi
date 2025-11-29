#pragma once

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>

#include <cstdint>

namespace ros
{

    template <typename T>
    struct PublisherTraits
    {
        static_assert(sizeof(T) == 0, "PublisherTraits not specialized for this type");
    };

    template <>
    struct PublisherTraits<int32_t>
    {
        using RclMessageType = std_msgs__msg__Int32;

        static auto get_type_support()
        {
            return ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);
        }

        static auto to_messge(int32_t data)
        {
            return RclMessageType{.data = data};
        }
    };

    template <>
    struct PublisherTraits<float>
    {
        using RclMessageType = std_msgs__msg__Float32;

        static auto get_type_support()
        {
            return ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32);
        }

        static auto to_messge(float data)
        {
            return RclMessageType{.data = data};
        }
    };
}