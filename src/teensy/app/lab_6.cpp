#include "app/lab_6.h"

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

namespace lab_6
{
    struct Config
    {
        io_utils::Settings io_setings{
            .serial_baud = 115200,
            .serial_redirect = io_utils::SerialRedirect::MICRO_ROS,
            .delay_after_init = Ms{2000},
        };

        WheelySettings wheely_settings = {
            .left = {
                .motor = {
                    .control_pin = Pin{5},
                    .feedback_pin = Pin{6},
                },
                .radius = Meter{0.0326},
            },
            .right = {
                .motor = {
                    .control_pin = Pin{7},
                    .feedback_pin = Pin{8},
                },
                .radius = Meter{0.0326},
            },
            .width = Meter{0.102},
        };

        const char *node_name = "wheely";
        const char *cmd_vel_topic = "cmd-vel";
        const char *cmd_vel_echo_topic = "cmd-vel-echo";
        const char *logs_topic = "logs";
        const char *config_topic = "wheely-config";

        Ms ping_interval{500};
        Ms ping_timeout{100};

        Ms spin_timeout{100};
        Ms connection_check_period{500};
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

        auto log_publisher = ros::Publisher<std::string>{node, config.logs_topic};
        io_utils::redirect_to(log_publisher);

        auto wheely = Wheely{config.wheely_settings};
        auto wheely_configurator = WheelyConfigurator{wheely, node, config.config_topic};

        auto subscription_callback = [&](const geo_utils::Twist &twist)
        {
            cmd_vel_echo_publisher.publish(twist);
            wheely.set_target_speed(twist);
        };

        auto subscription = ros::Subscription<geo_utils::Twist>{
            node,
            config.cmd_vel_topic,
            subscription_callback};

        auto executables = std::vector<ros::Executable>{
            &subscription.base(),
            &wheely_configurator.subscription().base(),
        };

        auto executor = ros::Executor{support, executables};

        auto last = Us{micros() - 1000}; // to prevent first dt == 0
        while (!utils::connection::is_disconnected(ping_timer, config.ping_timeout))
        {
            const auto now = Us{micros()};
            const auto dt = now - std::exchange(last, now);

            executor.spin_some(config.spin_timeout);
            wheely.update(dt);
        }

        io_utils::redirect_reset();
    }

    void run()
    {
        const auto config = Config{};
        io_utils::init(config.io_setings);

        while (true)
        {
            do_loop(config);
        }
    }
}