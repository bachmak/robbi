#pragma once

#include <optional>
#include <string>

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

    void init(int serial_baud, LogLevel log_level);

    std::string get_string();
    std::optional<std::string> try_get_string();

    void debug(const char *format, ...);
    void info(const char *format, ...);
    void warning(const char *format, ...);
    void error(const char *format, ...);
}
