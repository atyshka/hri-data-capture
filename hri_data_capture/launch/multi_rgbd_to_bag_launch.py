from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import IncludeLaunchDescription, GroupAction, TimerAction
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
    compressed_video = LaunchConfiguration('compressed_video').perform(context).lower() == 'true'
    record_audio = LaunchConfiguration('audio').perform(context).lower() == 'true'
    min_depth = float(LaunchConfiguration('min_depth').perform(context))
    max_depth = float(LaunchConfiguration('max_depth').perform(context))

    if not serials.strip():
        serial_list = ['']  # Default to a single camera with no serial specified
        cam_names = ['camera']
    else:
        serial_list = [s.strip() for s in serials.split(',') if s.strip()]
        cam_names = [f'camera_{serial}' for serial in serial_list]
    
    actions = []
    os.makedirs(bag_file_name, exist_ok=True) 

    # Launch camera drivers and hue encoders
    for i, (serial, cam_name) in enumerate(zip(serial_list, cam_names)):
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
            'camera_name': cam_name,
            'tf_prefix': cam_name,
        }
        if serial:
            driver_args['sensor_sn'] = serial
        
        driver_action = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory('azure_kinect_ros_driver'),
                            'launch',
                            'driver.launch.py'
                        )
                    ),
                    launch_arguments=driver_args.items()
                )

        group = [
            PushRosNamespace(namespace),
            TimerAction(period=5.0*i, actions=[driver_action]),
            Node(
                package='hri_data_capture',
                executable='hue_encode_depth',
                name='hue_encode_depth',
                output='screen',
                on_exit=Shutdown(),
                parameters=[{
                    'input_topic': 'depth/image_raw',
                    'output_topic': 'depth/hue_encoded',
                    'min_depth': min_depth,
                    'max_depth': max_depth,
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
        ]
        if compressed_video:
            group.append(
                Node(
                    package='image_transport',
                    executable='republish',
                    name='compressed_video_republisher',
                    output='screen',
                    on_exit=Shutdown(),
                    remappings=[
                        ('in', 'rgb/image_raw'),
                        ('out', 'rgb/image_raw')
                    ],
                    parameters=[{
                        'in_transport': 'compressed',
                        'out_transport': 'ffmpeg',
                        '.rgb.image_raw.ffmpeg.encoding': 'h264_nvenc',
                        '.rgb.image_raw.ffmpeg.preset': 'll',
                    }]
                )
            )
        actions.append(GroupAction(group))
        actions.append(
            Node(
                package='rqt_image_view',
                executable='rqt_image_view',
                name='rqt_image_view',
                arguments=['/foo'],
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        )
    
    # Only add the recorder node if recording is enabled
    if record:
        topics = ['/clock', '/tf', '/tf_static']
        for cam_name in cam_names:
            if compressed:
                topics.append(f'/{cam_name}/depth/hue_encoded/ffmpeg')
                if compressed_video:
                    topics.append(f'/{cam_name}/rgb/image_raw/ffmpeg')
                else:
                    topics.append(f'/{cam_name}/rgb/image_raw/compressed')
            else:
                topics.append(f'/{cam_name}/depth/image_raw')
                topics.append(f'/{cam_name}/rgb/image_raw')
            topics.append(f'/{cam_name}/rgb/camera_info')
            topics.append(f'/{cam_name}/depth/camera_info')
        actions.append(
            ExecuteProcess(
                cmd=['ros2', 'bag', 'record', '-o', bag_file_name + '/bag', '--custom-data', f'min_depth={min_depth}', f'max_depth={max_depth}', '--topics'] + topics,
                output='screen',
                name='multi_rgbd_to_bag_recorder',
                on_exit=Shutdown(),
            )
        )

    if record_audio:
        actions.append(
            ExecuteProcess(
                cmd=['arecord', '-D', 'plughw:CARD=Kinect_00048392', '-f', 'S32_LE', '-c', '7', '-r', '48000', bag_file_name + '/cam0_$(date +%Y-%m-%d_%H-%M-%S-%3N).wav'],
                output='screen',
                name='audio_recorder',
                shell=True,
            )
        )
        actions.append(
            ExecuteProcess(
                cmd=['arecord', '-D', 'plughw:CARD=Kinect_00050092', '-f', 'S32_LE', '-c', '7', '-r', '48000', bag_file_name + '/cam1_$(date +%Y-%m-%d_%H-%M-%S-%3N).wav'],
                output='screen',
                name='audio_recorder_2',
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
        DeclareLaunchArgument('compressed_video', default_value='true', description='Use compressed video for RGB images'),
        DeclareLaunchArgument('min_depth', default_value='0.5', description='Minimum depth for hue encoding'),
        DeclareLaunchArgument('max_depth', default_value='2.5', description='Maximum depth for hue encoding'),
        OpaqueFunction(function=launch_setup)
    ])
