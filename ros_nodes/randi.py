#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range


class Randi(Node):
    def __init__(self):
        super().__init__('randi')

        # Topics
        self.declare_parameter('range_topic', '/range')
        self.declare_parameter('cmd_vel_topic', '/robot/cmd_vel')

        range_topic = self.get_parameter('range_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        # Store incoming ranges (converted to meters)
        self.ranges = {
            "angle_0": None,     # left
            "angle_90": None,    # front
            "angle_180": None    # right
        }

        self.create_subscription(Range, range_topic, self.range_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        # Timer
        self.create_timer(0.1, self.update)

        self.get_logger().info("Randi simple logic started.")

    # ---------------------------- INPUT ----------------------------
    def range_callback(self, msg: Range):
        fid = msg.header.frame_id
        if fid in self.ranges:
            # Convert cm → m (sensor sends centimeters)
            self.ranges[fid] = msg.range / 100.0
        else:
            self.get_logger().warn(f"Unexpected frame_id: {fid}")

    # ---------------------------- MAIN LOOP ----------------------------
    def update(self):
        # Make sure we have all sensor data
        if any(v is None for v in self.ranges.values()):
            return

        left = self.ranges["angle_0"]
        front = self.ranges["angle_90"]
        right = self.ranges["angle_180"]

        self.get_logger().info(f"Left={left:.3f}  Front={front:.3f}  Right={right:.3f}")

        twist = Twist()

        # -----------------------------------------------------------
        # 1. Left obstacle → move forward until cleared
        # -----------------------------------------------------------
        if left < 0.4:
            twist.linear.x = +0.10
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            return

        # -----------------------------------------------------------
        # 2. Front obstacle distance control (0.2–0.8)
        # -----------------------------------------------------------
        if 0.0 < front < 0.8:

            # Too close → move backward
            if front < 0.45:
                twist.linear.x = -0.05

            # Too far → move forward
            elif front > 0.55:
                twist.linear.x = +0.05

            # Desired range reached → stop
            else:
                twist.linear.x = 0.0

            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            return

        # -----------------------------------------------------------
        # 3. Right obstacle → move backward until cleared
        # -----------------------------------------------------------
        if right < 0.4:
            twist.linear.x = -0.10
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            return

        # -----------------------------------------------------------
        # Default → stop
        # -----------------------------------------------------------
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

    # -----------------------------------------------------------
    # Shutdown
    # -----------------------------------------------------------
    def destroy(self):
        stop = Twist()
        self.cmd_pub.publish(stop)
        super().destroy_node()


def main(args=None):
        rclpy.init(args=args)
        node = Randi()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
