cmake_minimum_required(VERSION 3.5)
project(carlike_bot_simulation)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(robot_state_publisher REQUIRED)

# Install Python scripts
install(
  PROGRAMS
    src/camera_subscriber.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install other resources
install(
  DIRECTORY launch urdf scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
