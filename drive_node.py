import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from dynamixel_drive.driver import DynamixelDrive
import serial
import time

class DriveNode(Node):
    def __init__(self):
        super().__init__('dynamixel_drive')

        self.get_logger().info("Starting Dynamixel drive node")

        # ---- Dynamixels ----
        self.drive = DynamixelDrive(
            port="/dev/ttyUSB0",
            baudrate=57600,
            left_id=1,
            right_id=2
        )

        # ---- Arduino over USB ----
        self.arduino = None
        try:
            self.arduino = serial.Serial(
                '/dev/ttyACM0',   # confirm with: ls /dev/ttyACM*
                115200,
                timeout=0.1
            )
            time.sleep(2)  # Arduino reset delay
            self.get_logger().info("Arduino connected")
        except Exception as e:
            self.get_logger().warn(f"No Arduino: {e}")

        self.moving = False

        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.get_logger().info("Drive node ready")

    def send_led(self, value: str):
        if self.arduino:
            try:
                self.arduino.write(value.encode())
            except Exception:
                pass

    def cmd_vel_callback(self, msg: Twist):
        linear = msg.linear.x
        angular = msg.angular.z

        # YOUR mounting (already verified)
        left  = int(linear * 100 - angular * 50)
        right = int(-(linear * 100 + angular * 50))

        self.drive.set_velocity(left, right)

        # Determine motion state
        now_moving = abs(left) > 5 or abs(right) > 5

        # Send ONLY on change
        if now_moving and not self.moving:
            self.send_led('1')   # turn police lights ON
            self.get_logger().info("LED ON")
        elif not now_moving and self.moving:
            self.send_led('0')   # lights OFF
            self.get_logger().info("LED OFF")

        self.moving = now_moving

        self.get_logger().info(
            f"cmd_vel linear={linear:.2f} angular={angular:.2f} -> L={left} R={right}"
        )

def main():
    rclpy.init()
    node = DriveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
