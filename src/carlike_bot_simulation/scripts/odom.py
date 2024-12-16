import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomListener(Node):
    def __init__(self):
        super().__init__('odom_listener')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',  # Change to your odometry topic
            self.odom_callback,
            10
        )

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        x, y, z = position.x, position.y, position.z
        self.get_logger().info(f"Position: x={x}, y={y}, z={z}")

def main(args=None):
    rclpy.init(args=args)
    node = OdomListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
