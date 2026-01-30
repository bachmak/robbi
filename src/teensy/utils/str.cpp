#include "utils/str.h"

#include <algorithm>

namespace utils::str {

std::vector<std::string_view> split(std::string_view s) {
  constexpr auto delim = ' ';
  auto result = std::vector<std::string_view>{};

  auto start = std::size_t{0};
  while (start <= s.size()) {
    auto pos = s.find(delim, start);
    if (pos == std::string::npos) {
      pos = s.size();
    }
    auto len = pos - start;

    if (len > 0) {
      result.emplace_back(s.substr(start, len));
    }

    if (pos == s.size()) {
      break;
    }

    start = pos + 1;
  }

  return result;
}

std::optional<float> to_float(std::string_view str) {
  char *end = nullptr;

  auto result = strtof(str.data(), &end);
  if (end == str.data()) {
    return std::nullopt;
  }

  return result;
}

float to_float(std::string_view str, float default_value) {
  return to_float(str).value_or(default_value);
}

std::optional<bool> to_bool(std::string_view str) {
  if (str == "true" || str == "1") {
    return true;
  }

  if (str == "false" || str == "0") {
    return false;
  }

  return std::nullopt;
}

bool to_bool(std::string_view str, bool default_value) {
  return to_bool(str).value_or(default_value);
}

float to_float(const std::optional<std::string_view> &str, float default_value) {
  if (str.has_value()) {
    auto result = to_float(*str);
    if (result.has_value()) {
      return *result;
    }
  }

  return default_value;
}

std::optional<std::string_view> substr_after(std::string_view sv, std::string_view prefix) {
  if (sv.size() >= prefix.size() && sv.substr(0, prefix.size()) == prefix) {
    return sv.substr(prefix.size());
  }

  return std::nullopt;
}
} // namespace utils::str