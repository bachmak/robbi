#pragma once

#include <optional>
#include <string>
#include <vector>

namespace utils::str {

std::vector<std::string_view> split(std::string_view s);

std::optional<float> to_float(std::string_view str);
float to_float(std::string_view str, float default_value);

std::optional<bool> to_bool(std::string_view str);
bool to_bool(std::string_view str, bool default_value);

float to_float(const std::optional<std::string> &str, float default_value);

std::optional<std::string_view> substr_after(std::string_view sv, std::string_view prefix);
} // namespace utils::str