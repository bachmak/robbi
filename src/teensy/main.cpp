#include "robot/configurator.h"
#include "robot/controller.h"
#include "robot/robot.h"
#include "ros/connection.h"
#include "ros/executor.h"
#include "ros/node.h"
#include "ros/publisher.h"
#include "ros/subscription.h"
#include "ros/support.h"
#include "utils/debug.h"
#include "utils/io.h"
#include "utils/time.h"

struct Config {
  utils::io::Settings io_setings{
      .serial_baud = 115200,
      .log_level = utils::io::LogLevel::DEBUG,
      .serial_redirect = utils::io::SerialRedirect::MICRO_ROS,
      .delay_after_init = Ms{1000},
  };

  robot::RobotSettings robot_settings = {
      .left =
          {
              .motor =
                  {
                      .name = "left-wheel",
                      .control_pin = Pin{5},
                      .feedback_pin = Pin{6},
                  },
          },
      .right =
          {
              .motor =
                  {
                      .name = "right-wheel",
                      .control_pin = Pin{7},
                      .feedback_pin = Pin{8},
                  },
          },
  };

  std::string_view node_name = "robot";

  std::string_view cmd_vel_topic = "/robot/cmd_vel";
  std::string_view cmd_action_topic = "/robot/cmd_action";
  std::string_view logs_topic = "/robot/logs";
  std::string_view config_topic = "/robot/config";
  std::string_view echo_sub_topic = "/robot/echo_request";
  std::string_view echo_pub_topic = "/robot/echo_response";

  Ms ping_interval{500};
  Ms ping_timeout{100};

  Ms spin_timeout{100};
  Ms connection_check_period{500};

  Pin debug_pin{13};
};

void do_loop(const Config &config) {
  auto ping_timer = utils::time::Timer{
      config.ping_interval,
      // adjusted start point for timer to avoid waiting the first time
      -config.ping_interval,
  };
  while (!ros::is_connected(ping_timer, config.ping_timeout)) {
  }

  auto support = ros::Support{};
  auto node = ros::Node{support, config.node_name};

  auto log_publisher = ros::Publisher<std::string_view>{node, config.logs_topic};
  utils::io::redirect_to(log_publisher);

  auto echo_publisher = ros::Publisher<std::string_view>{node, config.echo_pub_topic};

  auto robot = robot::Robot{config.robot_settings};
  auto configurator = robot::Configurator{robot, node, config.config_topic};
  auto controller = robot::Controller{robot, node, config.cmd_vel_topic, config.cmd_action_topic};

  auto echo_sub = ros::Subscription<std::string_view>{
      node, config.echo_sub_topic,
      [&echo_publisher](std::string_view echo) { echo_publisher.publish(echo); }};

  auto executables = std::vector<ros::Executable>{
      &configurator.subscription().base(),
      &controller.cmd_vel_subscription().base(),
      &controller.cmd_action_subscription().base(),
      &echo_sub.base(),
  };

  auto executor = ros::Executor{support, executables};
  auto blinker = utils::debug::Blinker{config.debug_pin};

  auto last = Us{micros() - 1000}; // to prevent first dt == 0
  while (!ros::is_disconnected(ping_timer, config.ping_timeout)) {
    const auto now = Us{micros()};
    const auto dt = now - std::exchange(last, now);

    blinker.update();
    utils::debug::crash_report();

    executor.spin_some(config.spin_timeout);
    robot.update(dt);
  }

  utils::io::redirect_reset();
}

void run() {
  const auto config = Config{};
  utils::io::init(config.io_setings);

  while (true) {
    do_loop(config);
  }
}

void setup() {}

void loop() { run(); }