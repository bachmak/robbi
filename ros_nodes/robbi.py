#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
import math
import time


class Robbi(Node):
    def __init__(self):
        super().__init__('robbi')

        # ---- Parameters ----
        self.declare_parameter('range_topic', '/range')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('wall_side', 'left')
        self.declare_parameter('desired_distance_m', 0.20)
        self.declare_parameter('distance_tolerance_m', 0.02)
        self.declare_parameter('front_obstacle_m', 0.40)
        self.declare_parameter('forward_speed', 0.1)
        self.declare_parameter('kp_angular', 1.0)
        self.declare_parameter('control_rate', 2.0)
        self.declare_parameter('turn_angular_speed', 1.5)
        self.declare_parameter('turn_safe_margin_s', 0.5)

        # ---- Load parameters ----
        range_topic = self.get_parameter('range_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.wall_side = self.get_parameter('wall_side').value
        self.desired_distance = self.get_parameter('desired_distance_m').value
        self.dist_tolerance = self.get_parameter('distance_tolerance_m').value
        self.front_obstacle = self.get_parameter('front_obstacle_m').value
        self.forward_speed = self.get_parameter('forward_speed').value
        self.kp = self.get_parameter('kp_angular').value
        rate = self.get_parameter('control_rate').value
        self.turn_speed = self.get_parameter('turn_angular_speed').value
        self.turn_margin = self.get_parameter('turn_safe_margin_s').value

        # Sensor storage
        self.ranges = { "angle_0": None, "angle_90": None, "angle_180": None }

        self.create_subscription(Range, range_topic, self.range_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        self.state = "SEARCH"    # <<< NEW >>>
        self.turn_duration = (2 * math.pi) / self.turn_speed

        self.min_left = float('inf')
        self.min_right = float('inf')

        self.rotation_is_done = False

        self.create_timer(1.0 / rate, self.update)
        self.get_logger().info("Robbi: SEARCHING for wall first.")


    # ---------------------------- RANGE CALLBACK ----------------------------
    def range_callback(self, msg: Range):
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

        twist = Twist()

        # ===============================================================
        # STATE: SEARCH
        # ===============================================================
        if self.state == "SEARCH":
            # Keep track of closest left/right readings
            if left < self.min_left:
                self.min_left = left

            if right < self.min_right:
                self.min_right = right

            # Choose side
            if self.min_left < self.min_right:
                self.wall_side = "left"
            else:
                self.wall_side = "right"

            self.state = "FOLLOW"
            return

        if self.state == "FOLLOW":
            if 0 < front < self.front_obstacle:
                if not self.rotation_is_done:
                    twist.angular.z = self.turn_speed
                    self.cmd_pub.publish(twist)
                    time.sleep(self.turn_duration / 2)
                    self.rotation_is_done = True

                self.cmd_pub.publish(Twist())
                if self.wall_side == "left":
                    self.wall_side = "right"
                else:
                    self.wall_side = "left"

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
        if angular < 0.0:
            angular *= 1.5

        self.get_logger().info(f"angular={angular}")

        twist.linear.x = self.forward_speed
        twist.angular.z = angular
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = Robbi()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()