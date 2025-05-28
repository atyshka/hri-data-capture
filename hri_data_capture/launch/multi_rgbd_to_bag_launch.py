from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def launch_setup(context, *args, **kwargs):
    serials = LaunchConfiguration('serials').perform(context)
    bag_base_name = LaunchConfiguration('bag_base_name').perform(context)
    record = LaunchConfiguration('record').perform(context).lower() == 'true'
    compressed = LaunchConfiguration('compressed').perform(context).lower() == 'true'
    
    if not serials.strip():
        serial_list = ['']  # Default to a single camera with no serial specified
        cam_names = ['camera']
    else:
        serial_list = [s.strip() for s in serials.split(',') if s.strip()]
        cam_names = [f'camera_{serial}' for serial in serial_list]
    
    actions = []
    # Launch camera drivers and hue encoders
    for serial, cam_name in zip(serial_list, cam_names):
        namespace = cam_name
        driver_args = {
            'color_enabled': 'True',
            'depth_enabled': 'True',
            'color_format': 'jpeg' if compressed else 'bgra',
            'color_resolution': '2160P',
            'depth_mode': 'NFOV_UNBINNED',
            'fps': '30',
            'point_cloud': 'False',
            'rgb_point_cloud': 'False',
            'camera_name': cam_name
        }
        if serial:
            driver_args['serial_number'] = serial
        actions.append(
            GroupAction([
                PushRosNamespace(namespace),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory('azure_kinect_ros_driver'),
                            'launch',
                            'driver.launch.py'
                        )
                    ),
                    launch_arguments=driver_args.items()
                ),
                Node(
                    package='hri_data_capture',
                    executable='hue_encode_depth',
                    name='hue_encode_depth',
                    output='screen',
                    parameters=[{
                        'input_topic': 'depth/image_raw',
                        'output_topic': 'depth/hue_encoded',
                        'min_depth': 0.5,
                        'max_depth': 2.0,
                    }]
                ),
                Node(
                    package='image_transport',
                    executable='republish',
                    name='ffmpeg_republisher',
                    output='screen',
                    remappings=[
                        ('in', 'depth/hue_encoded'),
                        ('out', 'hue_encoded_depth')
                    ],
                    parameters=[{
                        '.hue_encoded_depth.ffmpeg.encoding': 'h264_nvenc',
                        '.hue_encoded_depth.ffmpeg.pix_fmt': 'gbrp',
                        '.hue_encoded_depth.ffmpeg.tune': 'lossless',
                    }]
                )
            ])
        )
    
    # Only add the recorder node if recording is enabled
    if record:
        actions.append(
            Node(
                package='hri_data_capture',
                executable='multi_rgbd_to_bag',
                name='multi_rgbd_to_bag_recorder',
                output='screen',
                arguments=['--cameras', ','.join(cam_names), '--bag_base_name', bag_base_name, '--compressed' if compressed else ''],
            )
        )
    return actions

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('serials', default_value='', description='Comma-separated list of camera serial numbers'),
        DeclareLaunchArgument('bag_base_name', default_value='rgbd_bag', description='Base name for bag files'),
        DeclareLaunchArgument('record', default_value='true', description='Enable recording to bag file'),
        DeclareLaunchArgument('compressed', default_value='true', description='Use compressed depth and color images'),
        OpaqueFunction(function=launch_setup)
    ])
