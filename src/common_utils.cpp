#include "common_utils.h"

#include <algorithm>

namespace common_utils
{
    std::string trim(const std::string &str)
    {
        auto start = std::find_if(str.begin(), str.end(), [](auto ch)
                                  { return !std::isspace(ch); });

        if (start == str.end())
        {
            return "";
        }

        auto end = std::find_if(str.rbegin(), str.rend(), [](auto ch)
                                { return !std::isspace(ch); })
                       .base();

        return std::string(start, end);
    }

    std::vector<std::string> split(const std::string &s)
    {
        constexpr auto delim = ' ';
        auto result = std::vector<std::string>{};

        auto start = std::size_t{0};
        while (start <= s.size())
        {
            auto pos = s.find(delim, start);
            if (pos == std::string::npos)
            {
                pos = s.size();
            }
            auto len = pos - start;

            if (len > 0)
            {
                result.emplace_back(s.substr(start, len));
            }

            if (pos == s.size())
            {
                break;
            }

            start = pos + 1;
        }

        return result;
    }

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