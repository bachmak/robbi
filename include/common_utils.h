#pragma once

#include <optional>
#include <string>

namespace common_utils
{
    std::optional<float> str_to_float(const std::string &str);
    float str_to_float(const std::optional<std::string> &str, float default_value);
    float ema(float new_value, float prev_value, float alpha);
}