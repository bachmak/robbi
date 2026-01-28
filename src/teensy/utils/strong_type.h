#pragma once

#include <cmath>

namespace utils::strong_type
{
    template <typename T, typename Tag>
    struct StrongType
    {
        template <typename U>
        constexpr explicit StrongType(U v) : v(static_cast<T>(v)) {}
        T v;

        using Self = StrongType<T, Tag>;

        friend Self operator+(Self a, Self b) { return Self{a.v + b.v}; }
        friend Self operator-(Self a, Self b) { return Self{a.v - b.v}; }
        friend Self operator*(Self a, Self b) { return Self{a.v * b.v}; }
        friend Self operator/(Self a, Self b) { return Self{a.v / b.v}; }
        friend Self operator%(Self a, Self b) { return Self{a.v % b.v}; }
        friend Self operator-(Self a) { return Self{-a.v}; }

        friend Self operator+(Self a, T b) { return Self{a.v + b}; }
        friend Self operator-(Self a, T b) { return Self{a.v - b}; }
        friend Self operator*(Self a, T b) { return Self{a.v * b}; }
        friend Self operator/(Self a, T b) { return Self{a.v / b}; }
        friend Self operator%(Self a, T b) { return Self{a.v % b}; }

        friend Self operator+(T a, Self b) { return Self{a + b.v}; }
        friend Self operator-(T a, Self b) { return Self{a - b.v}; }
        friend Self operator*(T a, Self b) { return Self{a * b.v}; }
        friend Self operator/(T a, Self b) { return Self{a / b.v}; }
        friend Self operator%(T a, Self b) { return Self{a % b.v}; }

        friend bool operator<(Self a, Self b) { return a.v < b.v; }
        friend bool operator>(Self a, Self b) { return a.v > b.v; }
        friend bool operator==(Self a, Self b) { return a.v == b.v; }
        friend bool operator<=(Self a, Self b) { return a.v <= b.v; }
        friend bool operator>=(Self a, Self b) { return a.v >= b.v; }

        friend bool operator<(Self a, T b) { return a.v < b; }
        friend bool operator>(Self a, T b) { return a.v > b; }
        friend bool operator==(Self a, T b) { return a.v == b; }
        friend bool operator<=(Self a, T b) { return a.v <= b; }
        friend bool operator>=(Self a, T b) { return a.v >= b; }

        Self &operator+=(Self b) { return (v += b.v, *this); }
        Self &operator-=(Self b) { return (v -= b.v, *this); }
        Self &operator*=(Self b) { return (v *= b.v, *this); }
        Self &operator/=(Self b) { return (v /= b.v, *this); }
        Self &operator%=(Self b) { return (v %= b.v, *this); }

        Self &operator+=(T b) { return (v += b, *this); }
        Self &operator-=(T b) { return (v -= b, *this); }
        Self &operator*=(T b) { return (v *= b, *this); }
        Self &operator/=(T b) { return (v /= b, *this); }
        Self &operator%=(T b) { return (v %= b, *this); }

        friend Self abs(Self a) { return Self{std::abs(a.v)}; }
    };
}