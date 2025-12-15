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
        self.declare_parameter('range_topic', '/range')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('cmd_action_topic', '/cmd_action')
        self.declare_parameter('desired_distance_m', 0.2)
        self.declare_parameter('front_obstacle_m', 0.40)
        self.declare_parameter('forward_speed', 0.1)
        self.declare_parameter('kp_angular', 2.5)
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('turn_angular_speed', 2.0)

        # ---- Load parameters ----
        range_topic = self.get_parameter('range_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        cmd_action_topic = self.get_parameter('cmd_action_topic').value
        self.front_obstacle = self.get_parameter('front_obstacle_m').value
        self.forward_speed = self.get_parameter('forward_speed').value
        self.kp = self.get_parameter('kp_angular').value
        rate = self.get_parameter('control_rate').value
        self.turn_speed = self.get_parameter('turn_angular_speed').value

        # Sensor storage
        self.ranges = { "angle_0": None, "angle_90": None, "angle_180": None }

        self.create_subscription(Range, range_topic, self.range_callback, 10)
        self.twist_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.action_pub = self.create_publisher(String, cmd_action_topic, 10)

        # State
        self.wall_side = None
        self.target_distance = None
        self.state = "SEARCH"
        self.turn_duration = 4
        self.turn_angle = 180

        self.turn_start_time = 0

        self.min_left = float('inf')
        self.min_right = float('inf')

        self.create_timer(1.0 / rate, self.update)
        self.get_logger().info("Robbi: SEARCHING for wall first.")


    # ---------------------------- RANGE CALLBACK ----------------------------
    def range_callback(self, msg: Range):
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