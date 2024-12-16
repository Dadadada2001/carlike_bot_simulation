import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_description',
            default_value=os.path.join(
                dir('carlike_bot_simulation'), 'urdf', 'carlike_bot.urdf.xacro'),
            description='P'
        ),
        
        # Robot State Publisher to publish robot state
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': LaunchConfiguration('robot_description')}]
        ),
        
        # Joint State Publisher GUI to interact with the robot (optional)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        
        # RViz to visualize the robot
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(
                dir('carlike_bot_simulation'), 'rviz', 'default.rviz')],
        ),
    ])
