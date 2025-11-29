#include "io_utils.h"

#include <Arduino.h>
#include <micro_ros_platformio.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This is only avaliable for Arduino framework with serial transport
#endif

constexpr auto color_reset = "\033[0m";
constexpr auto color_red = "\033[31m";
constexpr auto color_green = "\033[32m";
constexpr auto color_yellow = "\033[33m";
constexpr auto color_blue = "\033[34m";

namespace io_utils
{
    namespace
    {
        LogLevel s_log_level = LogLevel::INFO;

        const char *to_string(LogLevel log_level)
        {
            switch (log_level)
            {
            case LogLevel::OFF:
                return "";
            case LogLevel::ERROR:
                return "ERROR";
            case LogLevel::WARNING:
                return "WARNING";
            case LogLevel::INFO:
                return "INFO";
            case LogLevel::DEBUG:
                return "DEBUG";
            };

            return "UNKNOWN";
        }

        const char *to_ansi_terminal_color(LogLevel log_level)
        {
            switch (log_level)
            {
            case LogLevel::OFF:
                return color_reset;
            case LogLevel::ERROR:
                return color_red;
            case LogLevel::WARNING:
                return color_yellow;
            case LogLevel::INFO:
                return color_green;
            case LogLevel::DEBUG:
                return color_blue;
            }

            return color_reset;
        }

        void log(LogLevel log_level, const char *format, va_list ap)
        {
            if (log_level <= s_log_level)
            {
                Serial.printf("%s[%s] ", to_ansi_terminal_color(log_level), to_string(log_level));

                Serial.vprintf(format, ap);
                Serial.printf("%s\n", color_reset);
            }
        }
    }

    void init(const Settings &settings)
    {
        s_log_level = settings.log_level;
        Serial.begin(settings.serial_baud);

        switch (settings.serial_redirect)
        {
        case SerialRedirect::DEFAULT:
            break;
        case SerialRedirect::MICRO_ROS:
            set_microros_serial_transports(Serial);
            break;
        }

        delay(settings.delay_after_init.count());
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

#define LOG_WITH_LEVEL(log_level, format) \
    do                                    \
    {                                     \
        va_list ap;                       \
        va_start(ap, format);             \
        log(log_level, format, ap);       \
        va_end(ap);                       \
    } while (0)

    void debug(const char *format, ...)
    {
        LOG_WITH_LEVEL(LogLevel::DEBUG, format);
    }

    void info(const char *format, ...)
    {
        LOG_WITH_LEVEL(LogLevel::INFO, format);
    }

    void warning(const char *format, ...)
    {
        LOG_WITH_LEVEL(LogLevel::WARNING, format);
    }

    void error(const char *format, ...)
    {
        LOG_WITH_LEVEL(LogLevel::ERROR, format);
    }
}