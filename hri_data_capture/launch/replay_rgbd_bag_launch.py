from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def launch_setup(context, *args, **kwargs):
    bag_path = LaunchConfiguration('bag_path').perform(context)
    compressed = LaunchConfiguration('compressed').perform(context).lower() == 'true'
    cam_names = [s.strip() for s in LaunchConfiguration('cameras').perform(context).split(',') if s.strip()]
    actions = []

    # # Play the bag file
    # actions.append(
    #     ExecuteProcess(
    #         cmd=['ros2', 'bag', 'play', bag_path, '--loop', '--clock'],
    #         output='screen' )
    # )

    # For each camera, run a republisher for ffmpeg data
    for cam_name in cam_names:
        actions.append(
            Node(
                package='image_transport',
                executable='republish',
                name=f'ffmpeg_republisher_{cam_name}',
                output='screen',
                remappings=[
                    ('in', f'/{cam_name}/hue_encoded_depth'),
                    ('out', f'/{cam_name}/hue_encoded_depth')
                ],
                parameters=[{
                    'in_transport': 'ffmpeg',
                    'out_transport': 'raw',
                    '.hue_encoded_depth.ffmpeg.map.h264_nvenc': 'h264_nvdec',
                    'use_sim_time': True
                }]
            )
        )

    # Launch the rviz2 with config
    actions.append(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(get_package_share_directory('hri_data_capture'), 'rviz', 'replay_rgbd.rviz')],
            parameters=[{'use_sim_time': True}]
        )
    )
    return actions

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('bag_path', description='Path to the bag file to replay'),
        DeclareLaunchArgument('cameras', default_value='camera', description='Comma-separated list of camera namespaces'),
        DeclareLaunchArgument('compressed', default_value='true', description='Use compressed depth and color images'),
        OpaqueFunction(function=launch_setup)
    ])
