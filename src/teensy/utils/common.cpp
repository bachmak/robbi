#include "utils/common.h"

#include <algorithm>

namespace utils::common
{
    std::string_view trim(const std::string_view &str)
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

        return std::string_view(start, end - start);
    }

    std::vector<std::string_view> split(std::string_view s)
    {
        constexpr auto delim = ' ';
        auto result = std::vector<std::string_view>{};

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

    std::optional<float> str_to_float(std::string_view str)
    {
        char *end = nullptr;

        auto result = strtof(str.data(), &end);
        if (end == str.data())
        {
            return std::nullopt;
        }

        return result;
    }

    float str_to_float(std::string_view str, float default_value)
    {
        return str_to_float(str).value_or(default_value);
    }

    std::optional<bool> str_to_bool(std::string_view str)
    {
        if (str == "true" || str == "1")
        {
            return true;
        }

        if (str == "false" || str == "0")
        {
            return false;
        }

        return std::nullopt;
    }

    bool str_to_bool(std::string_view str, bool default_value)
    {
        return str_to_bool(str).value_or(default_value);
    }

    float str_to_float(const std::optional<std::string_view> &str, float default_value)
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

    std::optional<std::string_view> substr_after(
        std::string_view sv,
        std::string_view prefix)
    {
        if (sv.size() >= prefix.size() &&
            sv.substr(0, prefix.size()) == prefix)
        {
            return sv.substr(prefix.size());
        }

        return std::nullopt;
    }

    std::string dump_bytes_to_string(std::string_view s)
    {
        std::string out;
        out.reserve(s.size() * 3);

        static const char hex[] = "0123456789ABCDEF";

        for (unsigned char c : s)
        {
            out.push_back(hex[(c >> 4) & 0xF]);
            out.push_back(hex[c & 0xF]);
            out.push_back(' ');
        }

        return out;
    }
}