#pragma once

#include "common.h"

#include <optional>
#include <string>

namespace ros
{
    template <typename T>
    class Publisher;
}

namespace io_utils
{
    enum class LogLevel
    {
        OFF,
        ERROR,
        WARNING,
        INFO,
        DEBUG,
    };

    enum class SerialRedirect
    {
        DEFAULT,
        MICRO_ROS,
    };

    struct Settings
    {
        int serial_baud = 9600;
        LogLevel log_level = LogLevel::INFO;
        SerialRedirect serial_redirect = SerialRedirect::DEFAULT;
        Ms delay_after_init{0};
    };

    void init(const Settings &settings);

    void redirect_to(ros::Publisher<std::string_view> &publisher);
    void redirect_reset();

    std::string get_string();
    std::optional<std::string> try_get_string();

    void debug(const char *format, ...);
    void info(const char *format, ...);
    void warning(const char *format, ...);
    void error(const char *format, ...);
}
