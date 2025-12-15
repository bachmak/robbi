from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String
import math
import rclpy
import time

class AngleEMA:
    def __init__(self, alpha):
        self.alpha = alpha
        self.has_value = False
        self.ema_x = 0.0
        self.ema_y = 0.0
        self.angle = 0.0

    def update(self, angle_deg):
        angle_rad = math.radians(angle_deg)

        x = math.cos(angle_rad)
        y = math.sin(angle_rad)

        if not self.has_value:
            # initialize EMA
            self.ema_x = x
            self.ema_y = y
            self.has_value = True
        else:
            # EMA on the circle
            self.ema_x = (1 - self.alpha) * self.ema_x + self.alpha * x
            self.ema_y = (1 - self.alpha) * self.ema_y + self.alpha * y

        # reconstruct filtered angle
        self.angle = math.degrees(math.atan2(self.ema_y, self.ema_x))
        self.angle = (self.angle + 360) % 360
        return self.angle

    def current(self):
        return self.angle if self.has_value else None


def ang_diff(a, b):
    d = (a - b + 180) % 360 - 180
    return d

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

class LongRunny(Node):
    def __init__(self):
        super().__init__("long_runny")

        # Constants
        self.imu_topic = "/sensi/imu"
        self.cmd_action_topic = "/cmd_action"
        self.control_period = 1 / 50.0
        self.fwd_vel = 0.08
        self.cmd_vel_topic = "/cmd_vel"
        self.cmd_action_topic = "/cmd_action"
        self.angle_alpha = 0.8
        self.localize_time = 2.0
        self.k_p_left = 0.1
        self.k_p_right = 0.05
        self.k_i = 0
        self.max_angular_speed = 1.5
        self.stop_err = 100 # degree

        # State
        self.state = None
        self.curr_state_time = time.perf_counter()
        self.angle_ema = AngleEMA(self.angle_alpha)
        self.target_angle = None
        self.last_time = None
        self.integrator = 0.0

        # ROS2 entities
        self.create_subscription(String, self.imu_topic, self.immy_callback, 10)
        self.twist_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.action_pub = self.create_publisher(Twist, self.cmd_action_topic, 10)
        self.timer = self.create_timer(self.control_period, self.update)

        self.switch_state("LOCALIZE")
        self.get_logger().info("Long-Runny: started")

    def immy_callback(self, msg):
        yaw, pitch, roll = [float(val) for val in msg.data.split()]
        angle = self.angle_ema.update(yaw)
        return

    def cmd_vel(self, x, z):
        twist = Twist()
        twist.linear.x = x
        twist.angular.z = z
        self.twist_pub.publish(twist)

    def switch_state(self, state):
        self.state = state
        self.curr_state_time = time.perf_counter()
        self.last_time = self.curr_state_time

        if state == "LOCALIZE":
            return self.enter_localize()
        if state == "MOVE":
            return self.enter_move()
        if state == "STOP":
            return self.enter_stop()

    def update(self):
        t = time.perf_counter()
        dt = t - self.last_time
        self.last_time = t

        if self.state == "LOCALIZE":
            return self.update_localize(dt)
        if self.state == "MOVE":
            return self.update_move(dt)
        if self.state == "STOP":
            return self.update_stop(dt)

    def enter_localize(self):
        return

    def update_localize(self, dt):
        angle = self.angle_ema.current()
        if angle is None:
            return

        elapsed = time.perf_counter() - self.curr_state_time
        if elapsed < self.localize_time:
            return

        return self.switch_state("MOVE")

    def enter_move(self):
        self.target_angle = self.angle_ema.current()
        self.cmd_vel(self.fwd_vel, 0.0)
        return

    def update_move(self, dt):
        curr_angle = self.angle_ema.current()
        err = ang_diff(self.target_angle, curr_angle)

        if abs(err) > self.stop_err:
            return self.switch_state("STOP")

        k_p = self.k_p_right if err > 0 else self.k_p_left
        p = k_p * err

        self.integrator += err * dt
        i = self.integrator * self.k_i

        angular = p + i
        angular = clamp(angular, -self.max_angular_speed, self.max_angular_speed)

        k_p_str = "k_p_left" if k_p == self.k_p_left else "k_p_right"
        print(f"MOVE: curr={curr_angle}, target={self.target_angle}, err={err}, ang={angular}, k_p={k_p_str}")

        self.cmd_vel(self.fwd_vel, angular)
        return

    def enter_stop(self):
        self.cmd_vel(0.0, 0.0)
        return
    
    def update_stop(self, dt):
        pass

    def destroy(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LongRunny()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()