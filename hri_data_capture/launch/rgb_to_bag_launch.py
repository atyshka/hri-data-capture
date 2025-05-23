from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    azure_kinect_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('azure_kinect_ros_driver'),
                'launch',
                'driver.launch.py'
            )
        ]),
        launch_arguments={
            'color_enabled': 'True',
            'depth_enabled': 'False'
        }.items()
    )
    return LaunchDescription([
        azure_kinect_launch,
        Node(
            package='hri_data_capture',
            executable='rgb_to_bag',
            name='rgb_to_bag_recorder',
            output='screen'
        )
    ])