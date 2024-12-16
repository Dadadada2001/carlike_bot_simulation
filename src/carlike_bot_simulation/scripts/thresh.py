import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ThresholdImageNode(Node):
    def __init__(self):
        super().__init__('threshold_image_node')
        
        # Create a subscription to the raw image topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # Replace with your raw image topic
            self.image_callback,
            10
        )
        
        # Create a publisher for the thresholded image
        self.publisher = self.create_publisher(
            Image,
            '/camera/image_threshold',  # Output topic for thresholded image
            10
        )
        
        # Initialize CvBridge for ROS <-> OpenCV image conversion
        self.bridge = CvBridge()
        self.get_logger().info("Threshold Image Node has been started.")

    def image_callback(self, msg):
        # Convert the ROS image message to an OpenCV image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # Convert the image to grayscale
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # Apply binary thresholding
        _, bw_image = cv2.threshold(gray_image, 127, 255, cv2.THRESH_BINARY)

        # Debug: Log shape and type of image
        self.get_logger().info(f"Thresholding applied, shape: {bw_image.shape}")

        # Convert the thresholded image back to a ROS Image message
        try:
            thresholded_msg = self.bridge.cv2_to_imgmsg(bw_image, encoding='mono8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert thresholded image: {e}")
            return

        # Publish the thresholded image
        self.publisher.publish(thresholded_msg)
        self.get_logger().info("Published thresholded image.")

def main(args=None):
    rclpy.init(args=args)
    node = ThresholdImageNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
