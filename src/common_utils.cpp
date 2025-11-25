#include "common_utils.h"

namespace common_utils
{
    std::optional<float> str_to_float(const std::string &str)
    {
        char *end = nullptr;

        auto result = strtof(str.c_str(), &end);
        if (end == str.c_str())
        {
            return std::nullopt;
        }

        return result;
    }

    float str_to_float(const std::optional<std::string> &str, float default_value)
    {
        if (str.has_value())
        {
            auto result = str_to_float(*str);
            if (result.has_value())
            {
                return *result;
            }
        }

        return default_value;
    }

    float ema(float new_value, float prev_value, float alpha)
    {
        return alpha * new_value + (1.0f - alpha) * prev_value;
    }
}