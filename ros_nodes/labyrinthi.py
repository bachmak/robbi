import rclpy
import time

from collections import deque
from dataclasses import dataclass
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import String

@dataclass
class Section:
    distance: float
    move_duration: float
    expected_us_fwd: float # can be None
    expected_us_left: float # can be None
    expected_us_right: float # can be None
    rotation_angle: float
    rotation_duration: float

class Labyrinti(Node):
    def __init__(self):
        super().__init__('labyrinthi')

        # Constants
        self.wait_duration = 0.5 # sec
        range_topic = '/sensi/us'
        cmd_action_topic = '/robot/cmd_action'
        action_complete_topic = '/robot/action_complete'
        update_interval = 1 / 20.0

        # State
        self.state = None
        self.us_measurements = None
        self.current_section = None
        self.action_complete = False
        self.curr_state_time = None

        # ROS2 entities
        self.create_subscription(Range, range_topic, self.on_range, 10)
        self.create_timer(update_interval, self.update)
        self.action_pub = self.create_publisher(String, cmd_action_topic, 10)

        self.sections = deque[
            # 1
            Section(
                distance=2.60,
                move_duration=10.0,
                rotation_angle=-90.0,
                rotation_duration=2.0,
            ),
            # 2
            Section(
                distance=0.98,
                move_duration=5.0,
                rotation_angle=-90.0,
                rotation_duration=2.0,
            ),
            # 3
            Section(
                distance=0.75,
                move_duration=4.0,
                rotation_angle=-90.0,
                rotation_duration=2.0,
            ),
            # 4
            Section(
                distance=0.70,
                move_duration=4.0,
                rotation_angle=90.0,
                rotation_duration=2.0,
            ),
            # 5
            Section(
                distance=0.456,
                move_duration=3.0,
                rotation_angle=90.0,
                rotation_duration=2.0,
            ),
            # 6
            Section(
                distance=0.775,
                move_duration=4.0,
                rotation_angle=-90.0,
                rotation_duration=2.0,
            ),
            # 7
            Section(
                distance=1.375,
                move_duration=5.0,
                rotation_angle=-180.0,
                rotation_duration=4.0,
            ),
            # 8
            Section(
                distance=0.55,
                move_duration=3.0,
                rotation_angle=90.0,
                rotation_duration=2.0,
            ),
            # 9
            Section(
                distance=0.65,
                move_duration=3.0,
                rotation_angle=90.0,
                rotation_duration=2.0,
            ),
            # 10
            Section(
                distance=0.475,
                move_duration=3.0,
                rotation_angle=75.0,
                rotation_duration=2.0,
            ),
            # 11
            Section(
                distance=0.415,
                move_duration=3.0,
                rotation_angle=105.0,
                rotation_duration=2.0,
            ),
        ]

        self.switch_state("MOVE")


    def on_range(self, msg):
        if self.state != "WAIT":
            return

        fid = msg.header.frame_id
        if fid in self.us_measurements:
            self.us_measurements[fid] = msg.range
    

    def on_action_complete(self, msg):
        if self.state == "WAIT":
            self.get_logger().info(f"Unexpected action complete in {self.state} state")
        
        self.action_complete = True


    def switch_state(self, state):
        self.state = state
        self.curr_state_time = time.perf_counter()
        
        if state == "MOVE":
            return self.enter_move()
        if state == "ROTATE":
            return self.enter_rotate()
        if state == "WAIT":
            return self.enter_wait()


    def update(self):
        self.update_any()

        if self.state == "MOVE":
            return self.update_move()
        if self.state == "ROTATE":
            return self.update_rotate()
        if self.state == "WAIT":
            return self.update_wait()
    

    def update_any(self):
        if any(v and v[-1].value < 0.05 for v in self.us_measurements.values()):
            self.switch_state("STOP")


    def enter_move(self):
        self.current_section = self.sections.popleft()
        self.cmd_move(
            self.current_section.distance,
            self.current_section.move_duration,
        )
    

    def update_move(self):
        if not self.action_complete:
            return

        self.switch_state("WAIT")
    

    def enter_rotate(self):
        self.cmd_rotate(
            self.current_section.rotation_angle,
            self.current_section.rotation_duration,
        )
    

    def update_rotate(self):
        if not self.action_complete:
            return
        
        self.switch_state("MOVE")


    def enter_wait(self):
        self.ranges = { "angle_0": None, "angle_90": None, "angle_180": None }


    def update_wait(self):
        elapsed = time.perf_counter() - self.curr_state_time
        if elapsed < self.wait_duration:
            return
        
        self.switch_state("ROTATE")


    def cmd_move(self, meters, duration):
        action = String(data=f"move {meters} {duration}")
        self.action_pub.publish(action)
        self.action_complete = False


    def cmd_rotate(self, degrees, duration):
        action = String(data=f"rotate {degrees} {duration}")
        self.action_pub.publish(action)
        self.action_complete = False


    def destroy(self):
        self.cmd_move(0.0, 0.0)

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Labyrinti()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()


if __name__ == "__main__":
    main()
