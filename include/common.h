#pragma once

#include <chrono>
#include <cmath>

template <typename T, typename Tag>
struct StrongType
{
    constexpr explicit StrongType(T v) : v(v) {}
    T v;

    using Self = StrongType<T, Tag>;
    using ValueType = T;

    friend Self operator+(Self a, Self b) { return Self{a.v + b.v}; }
    friend Self operator-(Self a, Self b) { return Self{a.v - b.v}; }
    friend Self operator*(Self a, Self b) { return Self{a.v * b.v}; }
    friend Self operator/(Self a, Self b) { return Self{a.v / b.v}; }
    friend Self operator%(Self a, Self b) { return Self{a.v % b.v}; }
    friend Self operator-(Self a) { return Self{-a.v}; }

    friend bool operator<(Self a, Self b) { return a.v < b.v; }
    friend bool operator>(Self a, Self b) { return a.v > b.v; }
    friend bool operator==(Self a, Self b) { return a.v == b.v; }
    friend bool operator<=(Self a, Self b) { return a.v <= b.v; }
    friend bool operator>=(Self a, Self b) { return a.v >= b.v; }

    Self &operator+=(Self b) { return (v += b.v, *this); }
    Self &operator-=(Self b) { return (v -= b.v, *this); }
    Self &operator*=(Self b) { return (v *= b.v, *this); }
    Self &operator/=(Self b) { return (v /= b.v, *this); }
    Self &operator%=(Self b) { return (v %= b.v, *this); }

    friend Self abs(Self a) { return Self{std::abs(a.v)}; }
};

using Us = std::chrono::microseconds;
using Ms = std::chrono::milliseconds;
using Sec = std::chrono::seconds;

using Degree = StrongType<float, struct DegreeTag>;
using Meter = StrongType<float, struct MeterTag>;
using Pin = StrongType<int, struct PinTag>;
using Pwm = StrongType<int, struct PwmTag>;
using Speed = StrongType<int, struct SpeedTag>;