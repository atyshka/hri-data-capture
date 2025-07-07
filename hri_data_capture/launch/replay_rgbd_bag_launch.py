from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
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
        # actions.append(
        #     Node(
        #                 package='image_transport',
        #                 executable='republish',
        #                 name=f'rgb_republisher_{cam_name}',
        #                 remappings=[
        #                     ('in', f'/{cam_name}/rgb/image_raw'),
        #                     ('out', f'/{cam_name}/rgb/image_raw')
        #                 ],
        #                 parameters=[{
        #                     'in_transport': 'compressed',
        #                     'out_transport': 'raw',
        #                     'use_sim_time': True
        #                 }]
        #             ),
        # )
        # actions.append(Node(
        #                 package="image_proc",
        #                 executable="rectify_node",
        #                 name="rectify_rgb_node",
        #                 remappings=[
        #                     ('image', f'/{cam_name}/rgb/image_raw'),
        #                     ('camera_info', f'/{cam_name}/rgb/camera_info'),
        #                     ('image_rect', f'/{cam_name}/rgb/image_rect'),
        #                 ],
        #                 parameters=[{
        #                     'image_transport': 'compressed',
        #                     'use_sim_time': True,
        #                 }],
        #             ),)

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
                            'camera.depth.hue_encoded.ffmpeg.map.h264_nvenc': 'h264_cuvid',
                            'use_sim_time': True
                        }]
                    ),
                    # Rectify the depth image and rgb image
                    ComposableNode(
                        package="image_proc",
                        plugin="image_proc::RectifyNode",
                        name="rectify_node",
                        remappings=[
                            ('image', f'/{cam_name}/depth/image_raw'),
                            ('camera_info', f'/{cam_name}/depth/camera_info'),
                            ('image_rect', f'/{cam_name}/depth/image_rect'),
                        ],
                        parameters=[{
                            'use_sim_time': True,
                        }],
                    ),
                    ComposableNode(
                        package="image_proc",
                        plugin="image_proc::RectifyNode",
                        name="rectify_rgb_node",
                        remappings=[
                            ('image', f'/{cam_name}/rgb/image_raw'),
                            ('camera_info', f'/{cam_name}/rgb/camera_info'),
                            ('image_rect', f'/{cam_name}/rgb/image_rect'),
                        ],
                        parameters=[{
                            'image_transport': 'compressed',
                            'use_sim_time': True,
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
        OpaqueFunction(function=launch_setup)
    ])
