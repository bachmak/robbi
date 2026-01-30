#include "robot/robot.h"

#include "utils/io.h"
#include "utils/str.h"
#include "utils/time.h"

namespace robot {
namespace {

struct MotorSpeeds {
  DegSec left;
  DegSec right;
};

struct MotorPositions {
  Degree left;
  Degree right;
};

MotorSpeeds to_motor_speeds(const utils::geometry::Twist &twist, const RobotSettings &settings) {
  const auto v = twist.linear.x;
  const auto w = twist.angular.z;

  const auto v_l = v - settings.width.v / 2 * w;
  const auto v_r = v + settings.width.v / 2 * w;

  const auto w_l = v_l / settings.left.radius.v;
  const auto w_r = -v_r / settings.right.radius.v;

  const auto w_l_deg = DegSec{w_l * 180 / M_PI};
  const auto w_r_deg = DegSec{w_r * 180 / M_PI};

  return {
      .left = w_l_deg,
      .right = w_r_deg,
  };
}

MotorPositions to_motor_positions(Meter left, Meter right, const RobotSettings &settings) {
  auto calc_dist = [](Meter distance, Meter radius) {
    return Degree{distance.v / (2 * M_PI * radius.v) * 360.0f};
  };

  const auto dist_left = calc_dist(left, settings.left.radius);
  const auto dist_right = calc_dist(right, settings.right.radius);

  return {
      .left = dist_left,
      .right = dist_right,
  };
}

MotorPositions to_motor_positions(Degree rotation, const RobotSettings &settings) {
  const auto radius = settings.width / 2;
  const auto arc_length = radius * M_PI * rotation.v / 180.0f;

  // TODO: figure out the correct signs
  return to_motor_positions(-arc_length, -arc_length, settings);
}
} // namespace

Robot::Robot(const RobotSettings &settings)
    : settings_(settings), left_(settings_.left.motor), right_(settings_.right.motor) {}

void Robot::update(Us dt) {
  left_.update(dt);
  right_.update(dt);

  utils::time::delay(settings_.delay);
}

bool Robot::completed() const { return left_.completed() && right_.completed(); }

void Robot::set_target_speed(const utils::geometry::Twist &twist) {
  const auto speeds = to_motor_speeds(twist, settings_);

  utils::io::debug("Setting new speed: left = %.2f deg/sec, right = %.2f deg/sec", speeds.left.v,
                   speeds.right.v);

  left_.set_target_speed(speeds.left);
  right_.set_target_speed(speeds.right);
}

void Robot::set_target_distance(Meter distance, Us duration) {
  const auto positions = to_motor_positions(distance, -distance, settings_);

  utils::io::info("Setting new positions: left = %.2f deg, right = %.2f deg, "
                  "duration = %.2f s",
                  positions.left.v, positions.right.v, utils::time::to_sec(duration));

  left_.set_target_distance(positions.left, duration);
  right_.set_target_distance(positions.right, duration);
}

void Robot::set_target_rotation(Degree rotation, Us duration) {
  const auto positions = to_motor_positions(rotation, settings_);

  utils::io::info("Setting new positions: left = %.2f deg, right = %.2f deg, "
                  "duration = %.2f s",
                  positions.left.v, positions.right.v, utils::time::to_sec(duration));

  left_.set_target_distance(positions.left, duration);
  right_.set_target_distance(positions.right, duration);
}

void Robot::set_stop(bool value) {
  utils::io::info("Setting stop: stop = %s", value ? "true" : "false");

  left_.set_stop(value);
  right_.set_stop(value);
}

void Robot::configure(std::string_view setting, float value) {
  using utils::str::substr_after;

  auto configure_wheel = [&](WheelSettings &settings, robot::Motor &motor,
                             std::string_view subsetting) {
    if (auto motor_setting = substr_after(subsetting, "motor.")) {
      return motor.configure(*motor_setting, value);
    }
    if (subsetting == "radius") {
      settings.radius = Meter{value};
    }
  };

  if (auto subsetting = utils::str::substr_after(setting, "left.")) {
    configure_wheel(settings_.left, left_, *subsetting);
  } else if (auto subsetting = utils::str::substr_after(setting, "right.")) {
    configure_wheel(settings_.right, right_, *subsetting);
  } else if (setting == "width") {
    settings_.width = Meter{value};
  } else if (setting == "delay") {
    settings_.delay = Ms{static_cast<int>(value)};
  } else if (setting == "speed") {
    auto speed = DegSec{value};
    left_.set_target_speed(speed);
    right_.set_target_speed(-speed);
  } else {
    utils::io::error("Robot: unknown setting: %s", setting.data());
  }
}
} // namespace robot