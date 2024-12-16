import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class CarControlNode(Node):
    def __init__(self):
        super().__init__('car_control_node')
        
        # Create publishers for the left and right wheels
        self.left_wheel_pub = self.create_publisher(Float64, '/left_wheel_controller/command', 10)
        self.right_wheel_pub = self.create_publisher(Float64, '/right_wheel_controller/command', 10)
        
        # Example control values (replace with actual logic or inputs)
        self.control_speed(0.5, 0.5)  # Both wheels move forward at 0.5 m/s
        
    def control_speed(self, left_speed, right_speed):
        # Create messages for the left and right wheels
        left_msg = Float64()
        right_msg = Float64()
        
        left_msg.data = left_speed
        right_msg.data = right_speed
        
        # Publish the control commands to both wheels
        self.left_wheel_pub.publish(left_msg)
        self.right_wheel_pub.publish(right_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CarControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
