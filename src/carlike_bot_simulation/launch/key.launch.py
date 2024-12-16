from launch import LaunchDescription
from launch_ros.actions import Node

from geometry_msgs.msg import Twist
import sys
import termios
import tty
import rclpy
from rclpy.node import Node as ROSNode

class KeyboardControl(ROSNode):
    def __init__(self):
        super().__init__('keyboard_control')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.linear_speed = 0.5  # Default linear speed
        self.angular_speed = 1.0  # Default angular speed
        self.get_logger().info("Control the robot using the keyboard:\n"
                               "W/S: Increase/Decrease linear velocity\n"
                               "A/D: Turn left/right\n"
                               "X: Stop\n"
                               "Z/C: Decrease/Increase angular speed\n"
                               "Q: Quit")

    def get_key(self):
        """Capture a single key press from the keyboard."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def run(self):
        """Continuously read key presses and publish Twist messages."""
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

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='',  # No package since it's self-contained
            executable='keyboard_control',
            name='keyboard_control',
            output='screen',
            parameters=[{'python_script': KeyboardControl}]
        )
    ])
