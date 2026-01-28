#pragma once

#include <optional>
#include <string>
#include <vector>

namespace utils::str {

std::string_view trim(std::string_view s);
std::vector<std::string_view> split(std::string_view s);

std::optional<float> str_to_float(std::string_view str);
float str_to_float(std::string_view str, float default_value);

std::optional<bool> str_to_bool(std::string_view str);
bool str_to_bool(std::string_view str, bool default_value);

float str_to_float(const std::optional<std::string> &str, float default_value);

std::optional<std::string_view> substr_after(std::string_view sv, std::string_view prefix);

std::string dump_bytes_to_string(std::string_view s);
} // namespace utils::str