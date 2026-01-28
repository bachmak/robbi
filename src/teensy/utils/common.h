#pragma once

#include <optional>
#include <string>
#include <vector>

namespace common_utils
{
    template <class... Ts>
    struct overloads : Ts...
    {
        using Ts::operator()...;
    };

    template <class... Ts>
    overloads(Ts...) -> overloads<Ts...>;

    std::string_view trim(std::string_view s);
    std::vector<std::string_view> split(std::string_view s);

    std::optional<float> str_to_float(std::string_view str);
    float str_to_float(std::string_view str, float default_value);

    std::optional<bool> str_to_bool(std::string_view str);
    bool str_to_bool(std::string_view str, bool default_value);

    float str_to_float(const std::optional<std::string> &str, float default_value);
    float ema(float new_value, float prev_value, float alpha);

    std::optional<std::string_view> substr_after(
        std::string_view sv,
        std::string_view prefix);

    template <typename T>
    std::vector<T> remove_first(const std::vector<T> &v, size_t n = 1)
    {
        if (v.size() > n - 1)
        {
            auto copy = v;
            for (size_t i = 0; i < n; i++)
            {
                copy.erase(copy.begin());
            }
            return copy;
        }

        return v;
    }

    std::string dump_bytes_to_string(std::string_view s);
}