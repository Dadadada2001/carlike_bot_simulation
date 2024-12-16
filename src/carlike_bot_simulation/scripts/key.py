#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty

class KeyboardControl(Node):
    def __init__(self):
        super().__init__('keyboard_control')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.linear_speed = 0.5
        self.angular_speed = 1.0
        self.get_logger().info("Keyboard control ready:\n"
                               "W/S: Forward/Backward\n"
                               "A/D: Turn Left/Right\n"
                               "X: Stop\n"
                               "Z/C: Decrease/Increase Angular Speed\n"
                               "Q: Quit")

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def run(self):
        twist = Twist()
        while rclpy.ok():
            key = self.get_key()
            if key == 'w':  # Increase forward velocity
                twist.linear.x = self.linear_speed
                twist.angular.z = 0.0
            elif key == 's':  # Decrease backward velocity
                twist.linear.x = -self.linear_speed
                twist.angular.z = 0.0
            elif key == 'a':  # Turn left
                twist.linear.x = 0.0
                twist.angular.z = self.angular_speed
            elif key == 'd':  # Turn right
                twist.linear.x = 0.0
                twist.angular.z = -self.angular_speed
            elif key == 'x':  # Stop the robot
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            elif key == 'z':  # Decrease angular speed
                self.angular_speed = max(0.1, self.angular_speed - 0.1)
                self.get_logger().info(f"Angular speed: {self.angular_speed}")
                continue
            elif key == 'c':  # Increase angular speed
                self.angular_speed += 0.1
                self.get_logger().info(f"Angular speed: {self.angular_speed}")
                continue
            elif key == '\x03' or key == 'q':  # Ctrl+C or 'q' to quit
                self.get_logger().info("Exiting...")
                break
            else:
                continue

            self.publisher.publish(twist)

def main():
    rclpy.init()
    node = KeyboardControl()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
