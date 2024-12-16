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

        # Constant forward velocity
        self.constant_linear_speed = 0.05  

        # Predefined angular velocities (5 steering angles)
        self.angular_actions = [-1.0, -0.5, 0.0, 0.5, 1.0]  
        self.current_angular_action = 2  # Default: "Straight" action index

        # Key controls
        self.key_mapping = {
            '1': 0,  # Full Left
            '2': 1,  # Slight Left
            '3': 2,  # Straight
            '4': 3,  # Slight Right
            '5': 4   # Full Right
        }

        self.get_logger().info(
            "Keyboard control ready:\n"
            "1: Full Left, 2: Slight Left, 3: Straight, 4: Slight Right, 5: Full Right\n"
            "Q: Quit\n"
            "Velocity is constant; choose a steering action."
        )

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
            if key in self.key_mapping:
                # Update current angular action based on key press
                self.current_angular_action = self.key_mapping[key]
                angular_velocity = self.angular_actions[self.current_angular_action]

                # Set twist message with constant linear velocity and selected angular velocity
                twist.linear.x = self.constant_linear_speed
                twist.angular.z = angular_velocity

                self.get_logger().info(
                    f"Selected Action: {key} -> Angular Velocity: {angular_velocity:.2f}"
                )

            elif key == 'q' or key == '\x03':  # Quit with 'q' or Ctrl+C
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
