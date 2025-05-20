from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='azure_kinect_ros_driver',
            executable='azure_kinect_driver_node',
            name='azure_kinect_driver',
            output='screen',
            parameters=[{'color_enabled': True, 'depth_enabled': False}]
        ),
        Node(
            package='azure_data_capture',
            executable='rgb_to_bag',
            name='rgb_to_bag_recorder',
            output='screen'
        )
    ])