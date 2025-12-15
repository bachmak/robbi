import math
import rclpy
import serial

from rclpy.node import Node
from sensor_msgs.msg import Range

class Sensi(Node):
    def __init__(self):
        super().__init__('sensi')

        self.declare_parameter('serial_port', '/dev/ttyACM1')
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('topic_name', 'range')

        port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud = self.get_parameter('baud_rate').get_parameter_value().integer_value
        topic = self.get_parameter('topic_name').get_parameter_value().string_value

        self.publisher_ = self.create_publisher(Range, topic, 10)

        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f"Connected to serial port {port} at {baud} baud.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")

        self.timer = self.create_timer(0.05, self.read_serial)

    def read_serial(self):
        if self.ser.in_waiting:
            line = self.ser.readline().decode('utf-8').strip()
            if line:
                try:
                    angle_str, distance_str = line.split()
                    angle = int(angle_str)
                    distance = float(distance_str)

                    self.get_logger().info(f"{angle}: {distance} cm")

                    msg = Range()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = f"angle_{angle}"
                    msg.radiation_type = Range.ULTRASOUND
                    msg.field_of_view = math.radians(15)
                    msg.min_range = 0.02
                    msg.max_range = 4.0
                    msg.range = distance

                    self.publisher_.publish(msg)

                except ValueError:
                    self.get_logger().warn(f"Could not parse line: {line}")

def main(args=None):
    rclpy.init(args=args)
    node = Sensi()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down node.")
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
