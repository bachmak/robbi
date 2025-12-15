#!/usr/bin/env python3

import rclpy
import math
import time

from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import String

class Robbi(Node):
    def __init__(self):
        super().__init__('robbi')

        # Parameters
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('turn_angular_speed', 2.0)

        # Constants
        self.front_obstacle = 0.4
        self.forward_speed = 0.1
        self.kp = 2.5
        self.turn_duration = 4
        self.turn_angle = 180
        range_topic = '/range'
        cmd_vel_topic = '/cmd_vel'
        cmd_action_topic = '/cmd_action'
        update_interval = 1 / 20.0

        # State
        self.wall_side = None
        self.target_distance = None
        self.state = "SEARCH"
        self.ranges = { "angle_0": None, "angle_90": None, "angle_180": None }
        self.turn_start_time = 0

        # ROS2 entities
        self.create_subscription(Range, range_topic, self.on_range, 10)
        self.twist_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.action_pub = self.create_publisher(String, cmd_action_topic, 10)
        self.create_timer(update_interval, self.update)

        self.get_logger().info("Robbi: SEARCHING for wall first.")


    # ---------------------------- RANGE CALLBACK ----------------------------
    def on_range(self, msg: Range):
        if self.state == "TURN":
            return

        fid = msg.header.frame_id
        if fid in self.ranges:
            self.ranges[fid] = msg.range / 100.0  # cm → m
        else:
            self.get_logger().warn(f"Unexpected frame_id: {fid}")

    # ---------------------------- MAIN LOOP --------------------------------
    def update(self):
        if any(v is None for v in self.ranges.values()):
            return

        left = self.ranges["angle_0"]
        front = self.ranges["angle_90"]
        right = self.ranges["angle_180"]

        if self.state == "SEARCH":
            self.target_distance = min(left, right)
            
            if left < right:
                self.wall_side = "left"
            else
                self.wall_side = "right"

            self.state = "FOLLOW"
            return

        if self.state == "TURN":
            if time.time() - self.turn_start_time < self.turn_duration:
                return

            if self.wall_side == "left":
                self.wall_side = "right"
            else:
                self.wall_side = "left"

            self.ranges["angle_0"], self.ranges["angle_180"] = \
                self.ranges["angle_180"], self.ranges["angle_0"]

            self.ranges["angle_90"] = None
            self.state = "FOLLOW"
            return


        if self.state == "FOLLOW":
            if 0 < front < self.front_obstacle:
                self.twist_pub.publish(Twist())
                time.sleep(1)

                action = String()
                action.data = f"rotate {self.turn_angle} {self.turn_duration}"
                self.action_pub.publish(action)

                self.turn_start_time = time.time()
                self.state = "TURN"
                return

                time.sleep(self.turn_duration + 0.5)
                return

        # Select wall side
        if self.wall_side == "left":
            side = left
            direction = +1.0
        else:
            side = right
            direction = -1.0

        error = side - self.desired_distance
        angular = max(-1.5, min(1.5, direction * (self.kp * error)))

        self.get_logger().info(f"error={error}, angular={angular}, wall_side={self.wall_side} side={side}")

        twist = Twist()
        twist.linear.x = self.forward_speed
        twist.angular.z = angular
        self.twist_pub.publish(twist)

    def destroy(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Robbi()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.twist_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()