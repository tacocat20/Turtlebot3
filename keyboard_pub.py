#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select


def get_key(timeout=0.1):
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            return sys.stdin.read(1)
        return None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_pub')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.loop)

        self.linear = 0.0
        self.angular = 0.0

        self.get_logger().info(
            "Keyboard teleop started\n"
            "W/S: forward/back\n"
            "A/D: turn\n"
            "SPACE: stop\n"
            "CTRL+C: quit"
        )

    def loop(self):
        key = get_key()

        # default: STOP if no key
        linear = 0.0
        angular = 0.0

        if key == 'w':
            linear = 0.3
        elif key == 's':
            linear = -0.3
        elif key == 'a':
            angular = 1.0
        elif key == 'd':
            angular = -1.0
        elif key == ' ':
            pass
        elif key == '\x03':  # Ctrl-C
            rclpy.shutdown()
            sys.exit(0)

        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = KeyboardPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
