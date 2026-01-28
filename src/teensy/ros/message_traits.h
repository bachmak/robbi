#pragma once

#include "utils/geometry.h"

#include <geometry_msgs/msg/twist.h>
#include <micro_ros_utilities/type_utilities.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>

#include <cstdint>
#include <string_view>

namespace ros {

template <typename T> struct MessageTraits {
  static_assert(sizeof(T) == 0, "MessageTraits not specialized for this type");
};

template <> struct MessageTraits<int32_t> {
  using RclMessageType = std_msgs__msg__Int32;

  static auto get_type_support() { return ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32); }

  static auto to_message(int32_t data) { return RclMessageType{.data = data}; }

  static auto to_original_type(const RclMessageType *msg) { return int32_t{msg->data}; }

  static auto init(RclMessageType *) {}
  static auto finalize(RclMessageType *) {}
};

template <> struct MessageTraits<float> {
  using RclMessageType = std_msgs__msg__Float32;

  static auto get_type_support() { return ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32); }

  static auto to_message(float data) { return RclMessageType{.data = data}; }

  static auto init(RclMessageType *) {}
  static auto finalize(RclMessageType *) {}
};

template <> struct MessageTraits<utils::geometry::Twist> {
  using RclMessageType = geometry_msgs__msg__Twist;

  static auto get_type_support() { return ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist); }

  static auto to_message(const utils::geometry::Twist &twist) {
    return RclMessageType{
        .linear =
            {
                .x = twist.linear.x,
                .y = twist.linear.y,
                .z = twist.linear.z,
            },
        .angular =
            {
                .x = twist.angular.x,
                .y = twist.angular.y,
                .z = twist.angular.z,
            },
    };
  }

  static auto to_original_type(const RclMessageType *msg) {
    auto to_f = [](double a) { return static_cast<float>(a); };

    return utils::geometry::Twist{
        .linear =
            {
                .x = to_f(msg->linear.x),
                .y = to_f(msg->linear.y),
                .z = to_f(msg->linear.z),
            },
        .angular =
            {
                .x = to_f(msg->angular.x),
                .y = to_f(msg->angular.y),
                .z = to_f(msg->angular.z),
            },
    };
  }

  static auto init(RclMessageType *) {}
  static auto finalize(RclMessageType *) {}
};

template <> struct MessageTraits<std::string_view> {
  using RclMessageType = std_msgs__msg__String;

  static auto get_type_support() { return ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String); }

  static auto to_message(std::string_view str) {
    return RclMessageType{
        .data =
            {
                .data = const_cast<char *>(str.data()),
                .size = str.size(),
                .capacity = str.size() + 1,
            },
    };
  }

  static auto mem_conf() {
    return micro_ros_utilities_memory_conf_t{
        .max_string_capacity = 4096,
    };
  }

  static auto to_original_type(const RclMessageType *msg) {
    return std::string_view{msg->data.data, msg->data.size};
  }

  static auto init(RclMessageType *msg) {
    micro_ros_utilities_create_message_memory(get_type_support(), msg, mem_conf());
  }

  static auto finalize(RclMessageType *msg) {
    micro_ros_utilities_destroy_message_memory(get_type_support(), msg, mem_conf());
  }
};
} // namespace ros