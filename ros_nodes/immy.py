import math
import rclpy
import serial

from rclpy.node import Node
from std_msgs.msg import String

class Immy(Node):
    def __init__(self):
        super().__init__('immy')

        self.declare_parameter('serial_port', '/dev/ttyACM1')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('topic_name', 'immy')

        port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud = self.get_parameter('baud_rate').get_parameter_value().integer_value
        topic = self.get_parameter('topic_name').get_parameter_value().string_value

        self.publisher_ = self.create_publisher(String, topic, 10)
        self.ser = None

        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f"Connected to serial port {port} at {baud} baud.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")

        self.timer = self.create_timer(0.005, self.read_serial)

    def read_serial(self):
        if self.ser.in_waiting:
            line = self.ser.readline().decode('utf-8').strip()
            if line:
                try:
                    yaw, pitch, roll = line.split()
                    yaw = float(yaw)
                    pitch = float(pitch)
                    roll = float(roll)

                    msg = String()
                    msg.data = f"{yaw} {pitch} {roll}"
                    self.get_logger().info(f"Data: {msg.data}")

                    self.publisher_.publish(msg)

                except ValueError:
                    self.get_logger().warn(f"Could not parse line: {line}")

    def destroy(self):
        if self.ser is not None:
            self.ser.close()

        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = Immy()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down node.")
        node.destroy()

if __name__ == '__main__':
    main()
