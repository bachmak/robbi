#include "io_utils.h"

#include <Arduino.h>

namespace
{
    void print(const std::string &msg, const char *category)
    {
        Serial.printf("[%s]", category);
        Serial.println(msg.c_str());
    }
}

namespace io_utils
{
    void init(int serial_baud)
    {
        Serial.begin(serial_baud);
    }

    std::string get_string()
    {
        while (Serial.available() <= 0)
        {
        }

        auto serial_str = Serial.readString();
        return {serial_str.c_str(), serial_str.length()};
    }

    std::optional<std::string> try_get_string()
    {
        if (Serial.available() > 0)
        {
            return get_string();
        }

        return {};
    }

    void info(const std::string &msg)
    {
        print(msg, "INFO");
    }

    void warning(const std::string &msg)
    {
        print(msg, "WARNING");
    }

    void error(const std::string &msg)
    {
        print(msg, "ERROR");
    }
}