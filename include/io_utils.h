#pragma once

#include <optional>
#include <string>

namespace io_utils
{
    void init(int serial_baud);
    std::string get_string();
    std::optional<std::string> try_get_string();
    void info(const std::string &msg);
    void warning(const std::string &msg);
    void error(const std::string &msg);
}
