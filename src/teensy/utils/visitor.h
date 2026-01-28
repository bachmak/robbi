#pragma once

namespace utils::visitor {

template <class... Ts> struct overloads : Ts... { using Ts::operator()...; };

template <class... Ts> overloads(Ts...) -> overloads<Ts...>;
} // namespace utils::visitor