#include "io_utils.h"

#include <Arduino.h>

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

        void log(LogLevel log_level, const char *format, ...)
        {
            if (log_level <= s_log_level)
            {
                Serial.printf("[%s]", to_string(log_level));

                va_list ap;
                va_start(ap, format);
                Serial.printf(format, ap);
                Serial.print('\n');
                va_end(ap);
            }
        }
    }

    void init(int serial_baud, LogLevel log_level)
    {
        s_log_level = log_level;
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