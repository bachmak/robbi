#pragma once

#include <chrono>
#include <cmath>

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

using Ns = std::chrono::nanoseconds;
using Us = std::chrono::microseconds;
using Ms = std::chrono::milliseconds;
using Sec = std::chrono::seconds;

using Degree = StrongType<float, struct DegreeTag>;
using Meter = StrongType<float, struct MeterTag>;
using Pin = StrongType<int, struct PinTag>;
using Pwm = StrongType<int, struct PwmTag>;
using Speed = StrongType<int, struct SpeedTag>;
using DegSec = StrongType<float, struct DegSecTag>;