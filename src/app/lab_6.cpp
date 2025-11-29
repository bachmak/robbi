#include "app/lab_6.h"

#include "ros_oop/support.h"
#include "ros_oop/node.h"
#include "ros_oop/subscription.h"
#include "ros_oop/executor.h"
#include "ros_oop/publisher.h"
#include "ros_oop/timer.h"
#include "io_utils.h"
#include "time_utils.h"
#include "robot_utils.h"
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

        RobotSettings robot_settings = {
            .left_wheel_settings = {
                .control_pin = Pin{5},
                .feedback_pin = Pin{6},
                .speed_dead_range = Speed{4},
                .circumference = Meter{0.205},
                .label = "right-wheel",
            },
            .right_wheel_settings = {
                .control_pin = Pin{7},
                .feedback_pin = Pin{8},
                .speed_dead_range = Speed{4},
                .circumference = Meter{0.205},
                .label = "left-wheel",
            },
            .width = Meter{0.102},
        };

        const char *node_name = "wheels";
        const char *topic_name = "move";

        Ms ping_interval{500};
        Ms ping_timeout{100};

        Ms spin_timeout{100};
        Ms connection_check_period{500};
    };

    static const auto config = Config{};

    void do_loop(Wheels &wheels, const Config &config)
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

        auto wa_left = wheels.left.attach();
        auto wa_right = wheels.right.attach();

        auto motors_move = [&](Speed left, Speed right)
        {
            wheels.left.rotate(wa_left, left);
            wheels.right.rotate(wa_right, right);
        };

        auto subscription_callback = [&](int32_t data)
        {
            switch (data)
            {
            case 0:
                motors_move(Speed{0}, Speed{0});
                return;
            case 1:
                motors_move(Speed{1}, Speed{-1});
                return;
            case 2:
                motors_move(Speed{-1}, Speed{1});
                return;
            case 3:
                motors_move(Speed{-1}, Speed{-1});
                return;
            case 4:
                motors_move(Speed{1}, Speed{1});
                return;
            }
        };

        auto subscription = ros::Subscription<int32_t>{
            node,
            config.topic_name,
            subscription_callback};

        auto executables = std::vector<ros::Executable>{
            &subscription.base(),
        };

        auto executor = ros::Executor{support, executables};

        while (!utils::connection::is_disconnected(ping_timer, config.ping_timeout))
        {
            executor.spin_some(config.spin_timeout);
        }
    }

    void run()
    {
        const auto config = Config{};
        auto wheels = Wheels{
            config.robot_settings.left_wheel_settings,
            config.robot_settings.right_wheel_settings,
        };

        io_utils::init(config.io_setings);

        while (true)
        {
            do_loop(wheels, config);
        }
    }
}