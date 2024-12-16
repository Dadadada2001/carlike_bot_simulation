import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageProcessor:
    def __init__(self):
        self.bridge = CvBridge()
        self.image = None
        rclpy.Subscriber("/camera/image_raw", Image, self.callback)

    def callback(self, msg):
        # Convert ROS Image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        resized_image = cv2.resize(gray_image, (84, 84))
        self.image = resized_image / 255.0  # Normalize
