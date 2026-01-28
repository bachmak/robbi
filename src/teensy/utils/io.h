#pragma once

#include "types.h"

#include <Print.h>

#include <optional>
#include <string>

namespace ros {
template <typename T> class Publisher;
}

namespace utils::io {

enum class LogLevel {
  OFF,
  ERROR,
  WARNING,
  INFO,
  DEBUG,
};

enum class SerialRedirect {
  DEFAULT,
  MICRO_ROS,
};

struct Settings {
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

class StringPrint : public Print {
public:
  std::string buffer;

  size_t write(uint8_t b) override {
    buffer.push_back(static_cast<char>(b));
    return 1;
  }

  size_t write(const uint8_t *data, size_t len) override {
    buffer.append((const char *)data, len);
    return len;
  }
};
} // namespace utils::io
