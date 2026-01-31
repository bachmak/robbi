#include "robot/settings.h"

#include "robot/settings_visitor.h"
#include "utils/visitor.h"

namespace robot {

using utils::strong_type::StrongType;
using utils::visitor::overloads;

namespace {
auto make_getter() {
  return overloads{[]<typename T, typename Tag>(StrongType<T, Tag> value) {
                     return std::optional<float>{value.v};
                   },
                   [](Ms value) { return std::optional<float>{value.count()}; },
                   [](bool value) { return std::optional<float>{value ? 1.0f : 0.0f}; },
                   [](float value) { return std::optional<float>{value}; },
                   [](nullptr_t) { return std::optional<float>{}; }};
};

auto make_setter(float value) {
  return overloads{[value]<typename T, typename Tag>(StrongType<T, Tag> &setting) {
                     setting = StrongType<T, Tag>{value};
                     return true;
                   },
                   [value](Ms &setting) {
                     setting = Ms{static_cast<int>(value)};
                     return true;
                   },
                   [value](bool &setting) {
                     setting = value > 0.0f;
                     return true;
                   },
                   [value](float &setting) {
                     setting = value;
                     return true;
                   },
                   [](nullptr_t) { return false; }};
}
} // namespace

bool set(MotorSettings &settings, std::string_view s, float value) {
  return visit(settings, s, make_setter(value));
}

std::optional<float> get(const MotorSettings &settings, std::string_view s) {
  return visit(const_cast<MotorSettings &>(settings), s, make_getter());
}

bool set(WheelSettings &settings, std::string_view s, float value) {
  return visit(settings, s, make_setter(value));
}

std::optional<float> get(const WheelSettings &settings, std::string_view s) {
  return visit(const_cast<WheelSettings &>(settings), s, make_getter());
}

bool set(RobotSettings &settings, std::string_view s, float value) {
  return visit(settings, s, make_setter(value));
}

std::optional<float> get(const RobotSettings &settings, std::string_view s) {
  return visit(const_cast<RobotSettings &>(settings), s, make_getter());
}
} // namespace robot