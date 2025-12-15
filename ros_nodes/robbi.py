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
        range_topic = '/sensi/us'
        cmd_vel_topic = '/cmd_vel'
        cmd_action_topic = '/cmd_action'
        update_interval = 1 / 20.0

        # State
        self.wall_side = None
        self.target_distance = None
        self.state = None
        self.ranges = None
        self.curr_state_time = None

        # ROS2 entities
        self.create_subscription(Range, range_topic, self.on_range, 10)
        self.twist_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.action_pub = self.create_publisher(String, cmd_action_topic, 10)
        self.create_timer(update_interval, self.update)

        self.get_logger().info("Robbi: SEARCHING for wall first.")

        self.switch_state("SEARCH")


    def on_range(self, msg: Range):
        if self.state == "TURN":
            return

        fid = msg.header.frame_id
        if fid in self.ranges:
            self.ranges[fid] = msg.range
        else:
            self.get_logger().warn(f"Unexpected frame_id: {fid}")
    
    def switch_state(self, state):
        self.state = state
        self.curr_state_time = time.perf_counter()

        if state == "SEARCH":
            return self.enter_search()
        if state == "FOLLOW":
            return self.enter_follow()
        if state == "TURN":
            return self.enter_turn()


    def update(self):
        if any(v is None for v in self.ranges.values()):
            return

        if self.state == "SEARCH":
            return self.update_search()
        if self.state == "FOLLOW":
            return self.update_follow()
        if self.state == "TURN":
            return self.update_turn()
    

    def enter_search(self):
        self.wall_side = None
        self.target_distance = None
        self.ranges = { "angle_0": None, "angle_90": None, "angle_180": None }

    
    def update_search(self):
        if self.is_waiting_for_data():
            return

        self.init_wall_side()

        self.switch_state("FOLLOW")


    def enter_follow(self):
        return
    

    def update_follow(self):
        if self.is_waiting_for_data():
            return
        
        left, front, right = self.get_angles()
        if 0 < front < self.front_obstacle:
            return self.switch_state("TURN")

        if self.wall_side == "left":
            side = left
            direction = +1.0
        else:
            side = right
            direction = -1.0

        error = side - self.desired_distance
        angular = direction * (self.kp * error)

        self.get_logger().info(f"error={error}, angular={angular}, wall_side={self.wall_side} side={side}")
        self.cmd_vel(self.forward_speed, angular)
    

    def enter_turn(self):
        self.cmd_vel(0.0, 0.0)
        time.sleep(0.5)
        self.cmd_rotate(self.turn_angle, self.turn_duration)


    def update_turn(self):
        elapsed = time.perf_counter() - self.curr_state_time
        if elapsed < self.turn_duration:
            return
        
        self.invert_orientation()
        self.switch_state("FOLLOW")
    

    def is_waiting_for_data(self):
        return any(v is None for v in self.ranges.values()):

    
    def init_wall_side(self):
        left, _, right = self.get_angles()
        self.target_distance = min(left, right)
        if left < right:
            self.wall_side = "left"
        else:
            self.wall_side = "right"

    
    def invert_orientation(self):
        if left < right:
            self.wall_side = "left"
        else
            self.wall_side = "right"

        self.ranges["angle_0"], self.ranges["angle_180"] = \
            self.ranges["angle_180"], self.ranges["angle_0"]

        self.ranges["angle_90"] = None

    
    def get_angles(self):
        left = self.ranges["angle_0"]
        front = self.ranges["angle_90"]
        right = self.ranges["angle_180"]
        return left, front, right
    

    def cmd_vel(self, x, z):
        z = max(-1.5, min(1.5, z))

        twist = Twist()
        twist.linear.x = x
        twist.angular.z = z
        self.twist_pub.publish(twist)


    def cmd_rotate(self, degrees, duration):
        action = String()
        action.data = f"rotate {degrees} {duration}"
        self.action_pub.publish(action)


    def destroy(self):
        self.cmd_vel(0.0, 0.0)

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Robbi()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()


if __name__ == "__main__":
    main()