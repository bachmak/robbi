#include "robot/configurator.h"

#include "robot/robot.h"
#include "utils/io.h"
#include "utils/str.h"

namespace robot {
namespace {

struct ConfigCommand {
  std::string_view setting;
  float value;
};

std::optional<ConfigCommand> process_command(Configurator &configurator, std::string_view str) {
  utils::io::debug("RobotConfiguration: received message: %.*s", static_cast<int>(str.size()),
                   str.data());

  const auto tokens = utils::str::split(str);
  if (tokens.size() != 2) {
    return {};
  }

  const auto setting = tokens[0];
  const auto value = utils::str::to_float(tokens[1]);
  if (!value.has_value()) {
    return {};
  }

  utils::io::info("RobotConfiguration: applying setting: %.*s = %f",
                  static_cast<int>(setting.size()), setting.data(), *value);

  return ConfigCommand{setting, *value};
}
} // namespace

Configurator::Configurator(Robot &robot, ros::Node &node, std::string_view topic_name)
    : robot_(robot), //
      subscription_(node, topic_name.data(), [this](std::string_view str) {
        if (auto cmd = process_command(*this, str); cmd.has_value()) {
          configure(cmd->setting, cmd->value);
        }
      }) {}

void Configurator::configure(std::string_view setting, float value) {
  robot_.configure(setting, value);
}
} // namespace robot