#include "robot/configurator.h"

#include "robot/robot.h"
#include "utils/io.h"
#include "utils/str.h"

namespace robot {
namespace {

constexpr auto header = std::string_view{"RobotConfiguration"};

struct ConfigCommand {
  std::string_view setting;
  float value;
};

void process_command(Robot &robot, std::string_view str) {
  utils::io::info("%s: received message: %s", header.data(), str.data());

  const auto tokens = utils::str::split(str);
  if (tokens.size() != 2) {
    return;
  }

  const auto setting = tokens[0];
  if (tokens[1] == "get") {
    if (const auto value = robot.get_setting(setting)) {
      utils::io::info("%s: get %s = %f", header.data(), setting.data(), *value);
      return;
    }
    utils::io::info("%s: no such setting: %s", header.data(), setting.data());
    return;
  }

  const auto value = utils::str::to_float(tokens[1]);
  if (!value.has_value()) {
    utils::io::info("%s: invalid value for %s", header.data(), setting.data());
    return;
  }

  if (robot.set_setting(setting, *value)) {
    utils::io::info("%s: applied setting: %s = %f", header.data(), setting.data(), *value);
    return;
  }
  utils::io::info("%s: no such setting: %s", header.data(), setting.data());
}
} // namespace

Configurator::Configurator(Robot &robot, ros::Node &node, std::string_view topic_name)
    : robot_(robot), //
      subscription_(node, topic_name.data(),
                    [this](std::string_view str) { process_command(robot_, str); }) {}
} // namespace robot