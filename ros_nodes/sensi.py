import math
import rclpy
import serial

from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import String

class Sensi(Node):
    def __init__(self):
        super().__init__('sensi')

        self.declare_parameter('serial_port', '/dev/ttyACM1')

        port = self.get_parameter('serial_port').get_parameter_value().string_value

        baud = 115200
        us_topic = '/sensi/us'
        imu_topic = '/sensi/imu'

        self.us_pub = self.create_publisher(Range, us_topic, 10)
        self.imu_pub = self.create_publisher(String, imu_topic, 10)

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
                    tokens = line.split()
                    if tokens:
                        channel = tokens[0]
                        if channel == "us":
                            return self.read_us(tokens[1:])
                        if channel == "imu":
                            return self.read_imu(tokens[1:])
                except ValueError:
                    self.get_logger().warn(f"Could not parse line: {line}")
    
    
    def read_imu(self, tokens):
        yaw, pitch, roll = tokens
        yaw = float(yaw)
        pitch = float(pitch)
        roll = float(roll)

        msg = String()
        msg.data = f"{yaw} {pitch} {roll}"
        self.get_logger().info(f"IMU: {msg.data}")

        self.imu_pub.publish(msg)


    def read_us(self, tokens):
        angle_str, distance_str = tokens
        angle = int(angle_str)
        distance = float(distance_str)

        self.get_logger().info(f"US: {angle} {distance} m")

        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f"angle_{angle}"
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = math.radians(15)
        msg.min_range = 0.02
        msg.max_range = 4.0
        msg.range = distance

        self.us_pub.publish(msg)


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
