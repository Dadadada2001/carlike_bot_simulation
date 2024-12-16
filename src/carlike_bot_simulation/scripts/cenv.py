import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
import numpy as np
import cv2
from cv_bridge import CvBridge
from math import sqrt

class RobotEnv(Node):
    def __init__(self):
        super().__init__('robot_env')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.odom_subscription = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        self.bridge = CvBridge()
        self.current_image = None
        self.current_position = None
        self.done = False

        # Constant forward velocity
        self.constant_linear_speed = 0.5
        self.angular_actions = [-1.0, -0.5, 0.0, 0.5, 1.0]

    def image_callback(self, msg):
        """Convert the incoming ROS Image message to OpenCV format."""
        self.current_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def odom_callback(self, msg):
        """Extract robot's position from the /odom topic."""
        position = msg.pose.pose.position
        self.current_position = position

    def reset(self):
        """Reset the environment."""
        self.done = False
        self.current_image = None
        self.current_position = None
        # Optionally reset the robot in Gazebo
        self.send_velocity(0.0, 0.0)
        return self.wait_for_image()

    def step(self, action):
        """Execute the selected action and return the next state, reward, and done flag."""
        angular_velocity = self.angular_actions[action]
        self.send_velocity(self.constant_linear_speed, angular_velocity)

        # Wait for the next image and odometry data
        image = self.wait_for_image()
        reward = self.compute_reward()
        self.done = self.check_if_done()

        return image, reward, self.done

    def send_velocity(self, linear_x, angular_z):
        """Publish the velocity command."""
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.publisher.publish(twist)

    def wait_for_image(self):
        """Wait until the next image is available."""
        while self.current_image is None:
            rclpy.spin_once(self)
        return self.preprocess_image(self.current_image)

    def preprocess_image(self, image):
        """Resize and normalize the image for the neural network."""
        image = cv2.resize(image, (84, 84))  # Resize to 84x84
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)  # Convert to grayscale
        return np.expand_dims(image, axis=-1) / 255.0  # Normalize to [0, 1]

    def compute_reward(self):
        """Calculate the reward based on the robot's distance from the origin."""
        if self.current_position is None:
            return 0.0  # No position data available yet

        distance = sqrt(self.current_position.x**2 + self.current_position.y**2)

        # Reward based on distance from the origin
        if distance < 6.0:
            return -1.0 # Positive reward for being within 6 meters of the origin
        elif 6.0 <= distance <= 7.5:
            return 0.5 
        else:
            return -1.0 
    def check_if_done(self):
        """Check if the episode is done (robot moves too far off track)."""
        # You can add conditions for episode termination if needed
        return False
