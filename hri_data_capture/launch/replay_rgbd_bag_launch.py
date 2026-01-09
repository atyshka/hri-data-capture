from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os

def launch_setup(context, *args, **kwargs):
    bag_path = LaunchConfiguration('bag_path').perform(context)

    compressed = LaunchConfiguration('compressed').perform(context).lower() == 'true'
    compressed_video = LaunchConfiguration('compressed_video').perform(context).lower() == 'true'
    serials = LaunchConfiguration('serials').perform(context)
    if not serials.strip():
        serial_list = ['']  # Default to a single camera with no serial specified
        cam_names = ['camera']
    else:
        serial_list = [s.strip() for s in serials.split(',') if s.strip()]
        cam_names = [f'camera_{serial}' for serial in serial_list]
    actions = []

    # Play the bag file
    actions.append(
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag_path + '/bag', '--loop', '--clock', '--progress-bar-update-rate', '0'],
            output='screen', on_exit=Shutdown() )
    )

    # For each camera, run a republisher for ffmpeg data
    for cam_name in cam_names:
        actions.append(
            Node(
                package='hri_data_capture',
                executable='hue_decode_depth',
                name=f'hue_decode_depth_{cam_name}',
                output='screen',
                parameters=[{
                    'input_topic': f'/{cam_name}/depth/hue_encoded',
                    'output_topic': f'/{cam_name}/depth/image_raw',
                    'min_depth': 0.5,
                    'max_depth': 2.0,
                }]
            )
        )

    # Launch depth image processor nodes for producing a point cloud
    for cam_name in cam_names:
        actions.append(
            ComposableNodeContainer(
                name=f'depth_image_processor_container_{cam_name}',
                namespace=cam_name,
                package='rclcpp_components',
                executable='component_container_mt',
                composable_node_descriptions=[
                    ComposableNode(
                        package='image_transport',
                        plugin='image_transport::Republisher',
                        name=f'ffmpeg_republisher_{cam_name}',
                        remappings=[
                            ('in', f'/{cam_name}/depth/hue_encoded'),
                            ('out', f'/{cam_name}/depth/hue_encoded')
                        ],
                        parameters=[{
                            'in_transport': 'ffmpeg',
                            'out_transport': 'raw',
                            f'{cam_name}.depth.hue_encoded.ffmpeg.map.h264_nvenc': 'h264_cuvid',
                            'use_sim_time': True
                        }]
                    ),
                    ComposableNode(
                        package='image_transport',
                        plugin='image_transport::Republisher',
                        name=f'ffmpeg_republisher_{cam_name}_rgb',
                        remappings=[
                            ('in', f'/{cam_name}/rgb/image_raw'),
                            ('out', f'/{cam_name}/rgb/image_raw')
                        ],
                        parameters=[{
                            'in_transport': 'ffmpeg',
                            'out_transport': 'raw',
                            f'{cam_name}.rgb.image_raw.ffmpeg.map.h264_nvenc': 'h264_cuvid',
                            'use_sim_time': True
                        }]
                    ),
                    # Rectify the depth image and rgb image
                    ComposableNode(
                        package="image_proc",
                        plugin="image_proc::RectifyNode",
                        name=f'rectify_{cam_name}',
                        remappings=[
                            ('image', f'/{cam_name}/depth/image_raw'),
                            ('camera_info', f'/{cam_name}/depth/camera_info'),
                            ('image_rect', f'/{cam_name}/depth/image_rect'),
                        ],
                        parameters=[{
                            'use_sim_time': True,
                            'queue_size': 30
                        }],
                    ),
                    ComposableNode(
                        package="image_proc",
                        plugin="image_proc::RectifyNode",
                        name=f'rectify_rgb_{cam_name}',
                        remappings=[
                            ('image', f'/{cam_name}/rgb/image_raw'),
                            ('camera_info', f'/{cam_name}/rgb/camera_info'),
                            ('image_rect', f'/{cam_name}/rgb/image_rect'),
                        ],
                        parameters=[{
                            'use_sim_time': True,
                            'queue_size': 30
                        }],
                    ),
                ],
                output='screen'
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
        DeclareLaunchArgument('compressed_video', default_value='true', description='Use compressed video for RGB images'),
        OpaqueFunction(function=launch_setup)
    ])
