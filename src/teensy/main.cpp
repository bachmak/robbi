#include "ros_oop/support.h"
#include "ros_oop/node.h"
#include "ros_oop/subscription.h"
#include "ros_oop/executor.h"
#include "ros_oop/publisher.h"
#include "ros_oop/timer.h"
#include "io_utils.h"
#include "time_utils.h"
#include "wheely.h"
#include "wheely_configurator.h"
#include "utils/connection.h"

#include <CrashReport.h>

struct Config
{
    io_utils::Settings io_setings{
        .serial_baud = 115200,
        .log_level = io_utils::LogLevel::DEBUG,
        .serial_redirect = io_utils::SerialRedirect::MICRO_ROS,
        .delay_after_init = Ms{1000},
    };

    WheelySettings wheely_settings = {
        .left = {
            .motor = {
                .name = "left-wheel",
                .control_pin = Pin{5},
                .feedback_pin = Pin{6},
            },
        },
        .right = {
            .motor = {
                .name = "right-wheel",
                .control_pin = Pin{7},
                .feedback_pin = Pin{8},
            },
        },
    };

    const char *node_name = "wheely";

    const char *cmd_vel_topic = "cmd_vel";
    const char *cmd_action_topic = "cmd_action";
    const char *cmd_vel_echo_topic = "cmd_vel_echo";
    const char *logs_topic = "logs";
    const char *config_topic = "wheely_config";
    const char *echo_sub_topic = "echo_request";
    const char *echo_pub_topic = "echo_response";

    Ms ping_interval{500};
    Ms ping_timeout{100};

    Ms spin_timeout{100};
    Ms connection_check_period{500};

    Pin main_loop_pin{13};
};

void do_loop(const Config &config)
{
    auto ping_timer = time_utils::Timer{
        config.ping_interval,
        // adjusted start point for timer to avoid waiting the first time
        -config.ping_interval,
    };
    while (!utils::connection::is_connected(ping_timer, config.ping_timeout))
    {
    }

    auto support = ros::Support{};
    auto node = ros::Node{support, config.node_name};
    auto cmd_vel_echo_publisher = ros::Publisher<geo_utils::Twist>{
        node,
        config.cmd_vel_echo_topic,
    };

    auto log_publisher = ros::Publisher<std::string_view>{node, config.logs_topic};
    io_utils::redirect_to(log_publisher);

    auto echo_publisher = ros::Publisher<std::string_view>{node, config.echo_pub_topic};

    auto wheely = Wheely{config.wheely_settings};
    auto main_loop_delay = Ms{0};

    auto cfg_sub = ros::Subscription<std::string_view>{
        node,
        config.config_topic,
        [&](std::string_view str)
        {
            io_utils::debug(
                "WheelyConfiguration: received message: %.*s",
                static_cast<int>(str.size()),
                str.data());

            const auto tokens = common_utils::split(str);
            if (tokens.size() != 2)
            {
                return;
            }

            const auto setting = tokens[0];
            const auto value = common_utils::str_to_float(tokens[1]);
            if (!value.has_value())
            {
                return;
            }

            if (setting == "delay")
            {
                main_loop_delay = Ms{static_cast<int>(*value)};
            }
            else
            {
                wheely.configure(setting, *value);
            }

            io_utils::info(
                "WheelyConfiguration: applying setting: %.*s = %f",
                static_cast<int>(setting.size()),
                setting.data(),
                *value);

            wheely.configure(setting, *value);
        },
    };

    auto cmd_vel_sub = ros::Subscription<geo_utils::Twist>{
        node,
        config.cmd_vel_topic,
        [&](const geo_utils::Twist &twist)
        {
            cmd_vel_echo_publisher.publish(twist);
            wheely.set_target_speed(twist);
        }};

    auto cmd_action_sub = ros::Subscription<std::string_view>{
        node,
        config.cmd_action_topic,
        [&wheely](std::string_view cmd)
        {
            const auto tokens = common_utils::split(cmd);
            if (tokens.size() < 0)
            {
                return;
            }

            const auto action = tokens[0];
            if (action == "stop")
            {
                return wheely.set_stop(true);
            }
            if (action == "go")
            {
                return wheely.set_stop(false);
            }
            if (tokens.size() < 2)
            {
                return;
            }

            const auto param_1 = common_utils::str_to_float(tokens[1]);
            const auto param_2 = common_utils::str_to_float(tokens[2]);
            if (!param_1.has_value() || !param_2.has_value())
            {
                return;
            }

            const auto duration = Us{static_cast<int64_t>(*param_2 * 1'000'000)};
            if (action == "move")
            {
                return wheely.set_target_distance(Meter{*param_1}, duration);
            }
            if (action == "rotate")
            {
                return wheely.set_target_rotation(Degree{*param_1}, duration);
            }
        }};

    auto echo_sub = ros::Subscription<std::string_view>{
        node,
        config.echo_sub_topic,
        [&echo_publisher](std::string_view echo)
        {
            echo_publisher.publish(echo);
        }};

    auto executables = std::vector<ros::Executable>{
        &cmd_vel_sub.base(),
        &cmd_action_sub.base(),
        &cfg_sub.base(),
        &echo_sub.base(),
    };

    auto executor = ros::Executor{support, executables};

    bool led_switch = false;

    auto last = Us{micros() - 1000}; // to prevent first dt == 0
    while (!utils::connection::is_disconnected(ping_timer, config.ping_timeout))
    {
        const auto now = Us{micros()};
        const auto dt = now - std::exchange(last, now);

        digitalWrite(config.main_loop_pin.v, std::exchange(led_switch, !led_switch));

        if (CrashReport)
        {
            auto p = io_utils::StringPrint{};
            CrashReport.printTo(p);
            io_utils::debug("there is a crash report:\n%s", p.buffer.c_str());
        }

        executor.spin_some(config.spin_timeout);
        wheely.update(dt);

        delay(main_loop_delay.count());
    }

    io_utils::redirect_reset();
}

void run()
{
    const auto config = Config{};
    io_utils::init(config.io_setings);
    pinMode(config.main_loop_pin.v, OUTPUT);

    while (true)
    {
        do_loop(config);
    }
}

void setup() {}

void loop()
{
    run();
}