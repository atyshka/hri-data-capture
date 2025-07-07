from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from datetime import datetime

def launch_setup(context, *args, **kwargs):
    serials = LaunchConfiguration('serials').perform(context)
    bag_base_name = LaunchConfiguration('bag_base_name').perform(context)
    bag_file_name = f"{bag_base_name}_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    record = LaunchConfiguration('record').perform(context).lower() == 'true'
    compressed = LaunchConfiguration('compressed').perform(context).lower() == 'true'
    record_audio = LaunchConfiguration('audio').perform(context).lower() == 'true'

    if not serials.strip():
        serial_list = ['']  # Default to a single camera with no serial specified
        cam_names = ['camera']
    else:
        serial_list = [s.strip() for s in serials.split(',') if s.strip()]
        cam_names = [f'camera_{serial}' for serial in serial_list]
    
    actions = []
    os.makedirs(bag_file_name, exist_ok=True) 

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
                    on_exit=Shutdown(),
                    remappings=[
                        ('in', 'depth/hue_encoded'),
                        ('out', 'depth/hue_encoded')
                    ],
                    parameters=[{
                        'in_transport': 'raw',
                        'out_transport': 'ffmpeg',
                        '.depth.hue_encoded.ffmpeg.encoding': 'h264_nvenc',
                        '.depth.hue_encoded.ffmpeg.pix_fmt': 'gbrp',
                        '.depth.hue_encoded.ffmpeg.tune': 'lossless',
                    }]
                )
            ])
        )
    
    # Only add the recorder node if recording is enabled
    if record:
        topics = ['/clock', '/tf', '/tf_static']
        for cam_name in cam_names:
            if compressed:
                topics.append(f'/{cam_name}/depth/hue_encoded/ffmpeg')
                topics.append(f'/{cam_name}/rgb/image_raw/compressed')
            else:
                topics.append(f'/{cam_name}/depth/image_raw')
                topics.append(f'/{cam_name}/rgb/image_raw')
            topics.append(f'/{cam_name}/rgb/camera_info')
            topics.append(f'/{cam_name}/depth/camera_info')
        actions.append(
            ExecuteProcess(
                cmd=['ros2', 'bag', 'record', '-o', bag_file_name + '/bag', '--topics'] + topics,
                output='screen',
                name='multi_rgbd_to_bag_recorder'
            )
        )
        # actions.append(
        #     Node(
        #         package='hri_data_capture',
        #         executable='multi_rgbd_to_bag',
        #         name='multi_rgbd_to_bag_recorder',
        #         output='screen',
        #         arguments=['--cameras', ','.join(cam_names), '--bag_base_name', bag_base_name, '--compressed' if compressed else ''],
        #     )
        # )
    if record_audio:
        actions.append(
            ExecuteProcess(
                cmd=['arecord', '-D', 'plughw:CARD=Array', '-f', 'S32_LE', '-c', '7', '-r', '48000', bag_file_name + '/cam0_$(date +%Y-%m-%d_%H-%M-%S-%3N).wav'],
                output='screen',
                name='audio_recorder',
                shell=True,
            )
        )
    return actions

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('serials', default_value='', description='Comma-separated list of camera serial numbers'),
        DeclareLaunchArgument('bag_base_name', default_value='rgbd_bag', description='Base name for bag files'),
        DeclareLaunchArgument('record', default_value='true', description='Enable recording to bag file'),
        DeclareLaunchArgument('audio', default_value='true', description='Enable audio recording'),
        DeclareLaunchArgument('compressed', default_value='true', description='Use compressed depth and color images'),
        OpaqueFunction(function=launch_setup)
    ])
