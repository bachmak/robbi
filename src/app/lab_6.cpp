#include "app/lab_6.h"

#include "ros_oop/support.h"
#include "ros_oop/node.h"
#include "ros_oop/publisher.h"
#include "ros_oop/executor.h"
#include "ros_oop/timer.h"
#include "io_utils.h"

#include <Arduino.h>

namespace lab_6
{
    struct Config
    {
        io_utils::Settings io_setings{
            .serial_baud = 115200,
            .serial_redirect = io_utils::SerialRedirect::MICRO_ROS,
            .delay_after_init = Ms{2000},
        };

        const char *node_name = "micro_ros_platformio_node";
        const char *topic_name = "micro_ros_platformio_node_publisher";

        Ms timer_period{1000};
        Ms loop_delay{100};
        Ms spin_timeout{100};
    };

    void do_loop(ros::Executor &executor, const Config &config)
    {
        delay(config.loop_delay.count());
        executor.spin_some(config.spin_timeout);
    }

    void run()
    {
        const auto config = Config{};

        io_utils::init(config.io_setings);

        auto &support = ros::Support::get_instance();
        auto node = ros::Node{support, config.node_name};
        auto publisher = ros::Publisher<int32_t>{node, config.topic_name};
        auto timer_callback = [&](int64_t last_call_time)
        {
            static auto counter = 0;
            publisher.publish(counter++);
        };

        auto timer = ros::Timer{support, config.timer_period, timer_callback};

        auto executables = std::vector<ros::Executable>{
            &timer,
        };

        auto executor = ros::Executor{support, executables};

        while (true)
        {
            do_loop(executor, config);
        }
    }
}