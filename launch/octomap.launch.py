#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
)
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PythonExpression,
    PathJoinSubstitution,
    IfElseSubstitution,
)
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch.actions import LogInfo



def generate_launch_description():
    """
    Generates the launch description for the Octomap_Server node.

    This launch file is the ROS2 equivalent of the provided ROS1 XML launch file.
    It launches the Octomap_Server as a composable node inside a container,
    and handles parameters and remappings in the ROS2 way.
    """
    # LaunchDescription object
    ld = LaunchDescription()

    # Package and path
    pkg_name = "mrs_octomap_server"
    this_pkg_path = get_package_share_directory(pkg_name)

    # Arguments

    ld.add_action(DeclareLaunchArgument(
        'UAV_NAME',
        default_value=EnvironmentVariable('UAV_NAME', default_value='uav30'),
        description="Name of the UAV, used for namespacing"
    ))
    uav_name = LaunchConfiguration('UAV_NAME')


    ld.add_action(
        DeclareLaunchArgument(
            'RUN_TYPE',
            default_value=os.getenv('RUN_TYPE', ''),
            description="Type of run (simulation or real).",
        )
    )
    run_type = LaunchConfiguration('RUN_TYPE')


    ld.add_action(
        DeclareLaunchArgument(
            'debug',
            default_value="false",
            description="Enable debug mode.",
        )
    )
    debug = LaunchConfiguration('debug')

    ld.add_action(
        DeclareLaunchArgument(
            'standalone',
            default_value='false',
            description="Enable debug mode.",
        )
    )
    standalone = LaunchConfiguration('standalone')


    ld.add_action(
        DeclareLaunchArgument(
            'custom_config',
            default_value='',
            description="Path to the custom configuration file. The path can be absolute, starting with '/' or relative to the current working directory",
        )
    )
    custom_config = LaunchConfiguration('custom_config')



    processed_custom_config = IfElseSubstitution(
            condition=PythonExpression(['"', custom_config, '" != "" and ', 'not "', custom_config, '".startswith("/")']),
            if_value=PathJoinSubstitution([EnvironmentVariable('PWD'), custom_config]),
            else_value=custom_config
            )

    run_type=os.getenv('RUN_TYPE', 'realworld')

    if run_type == "simulation":
        simulation = True
    else:
        simulation = False



    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description="Whether to use simulation time."
    ))
    use_sim_time = LaunchConfiguration('use_sim_time')\

    # ld.add_action(DeclareLaunchArgument(
    #     'custom_config',
    #     default_value="",
    #     description="Path to a custom YAML configuration file. Can be absolute or relative.",
    # ))


    # ld.add_action(LogInfo(msg=["Remapping lidar_3d_0_in to: ", LaunchConfiguration('lidar_3d_topic_0_in')]))



    # ########################
    # ## Frame ID Arguments ##
    # ########################

    world_frame = PythonExpression(['"', uav_name, '/world_origin"']) # Corrected based on your log output
    robot_frame = PythonExpression(['"', uav_name, '/fcu"'])

    # ###################################
    # ## Composable Node and Container ##
    # ###################################
    config_files = [
        os.path.join(this_pkg_path, 'config', 'default.yaml'),
    ]

    ld.add_action(LogInfo(msg=["Config file exists: ", str(os.path.exists(config_files[0]))]))
    ld.add_action(LogInfo(msg=["Config file path: ", str(config_files)]))


    ld.add_action(ComposableNodeContainer(
        namespace=uav_name,
        # name= uav_name_str +'_octomap_server_container',
        name='octomap_server_container',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        composable_node_descriptions=[
            ComposableNode(
                package=pkg_name,
                plugin='mrs_octomap_server::OctomapServer',
                name='octomap_server',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'config_files': config_files,
                    'custom_config': processed_custom_config,
                    'uav_name': uav_name,
                    'simulation': simulation,
                    'world_frame_id': world_frame,
                    'robot_frame_id': robot_frame,
                    'map_path': '/tmp/',
                    'vizualization/frame_id': world_frame,
                }],
                remappings=[
                        # 3D Lidar
                        #("~/lidar_3d_0_in", '~/lidar/points'),
                        ("~/lidar_3d_0_in", '/uav30/ouster/points'),
                        ("~/lidar_3d_1_in", '~/lidar_3d_topic_1'),          #need modifications
                        ("~/lidar_3d_2_in", '~/lidar_3d_topic_2'),
                        ("~/lidar_3d_0_over_max_range_in", '/uav30/ouster/lidar_3d_topic_0_over_max_range'),
                        ("~/lidar_3d_1_over_max_range_in", '/uav30/ouster/lidar_3d_topic_1_over_max_range'),
                        ("~/lidar_3d_2_over_max_range_in", '/uav30/ouster/lidar_3d_topic_2_over_max_range'),

                        # 2D Lidar
                        ("~/lidar_2d_0_in", '~/lidar_2d_topic_0'),
                        ("~/lidar_2d_1_in", '~/lidar_2d_topic_1'),
                        ("~/lidar_2d_2_in", '~/lidar_2d_topic_2'),

                        # Depth Camera
                        ("~/depth_camera_0_in", '~/depth_camera_topic_0'),
                        ("~/depth_camera_1_in", '~/depth_camera_topic_1'),
                        ("~/depth_camera_2_in", '~/depth_camera_topic_2'),
                        ("~/depth_camera_0_over_max_range_in", '~/depth_camera_topic_0_over_max_range'),
                        ("~/depth_camera_1_over_max_range_in", '~/depth_camera_topic_1_over_max_range'),
                        ("~/depth_camera_2_over_max_range_in", '~/depth_camera_topic_2_over_max_range'),

                        # Camera Info
                        ("~/camera_info_0_in", '~/camera_info_topic_0'),
                        ("~/camera_info_1_in", '~/camera_info_topic_1'),
                        ("~/camera_info_2_in", '~/camera_info_topic_2'),

                        # Other remappings
                        ("~/control_manager_diagnostics_in", "~/control_manager/diagnostics"),
                        ("~/height_in", "~/odometry/height"),
                        ("~/clear_box_in", "~/uav_pose_estimator/clear_box"),

                        # Topics out
                        ("~/octomap_global_full_out", "~/octomap_global_full"),
                        ("~/octomap_global_binary_out", "~/octomap_global_binary"),
                        ("~/octomap_local_full_out", "~/octomap_local_full"),
                        ("~/octomap_local_binary_out", "~/octomap_local_binary"),

                        # Services
                        ("~/reset_map_in", "~/reset_map"),
                        ("~/save_map_in", "~/save_map"),
                        ("~/load_map_in", "~/load_map"),
                    ]
            )

        ],
    ))
    return ld


# Octomap_node = ComposableNode(
#         package=pkg_name,
#         plugin='mrs_octomap_server::OctomapServer',
#         namespace=uav_name,
#         name='octomap_server',
#         parameters=[{
#             'use_sim_time': use_sim_time,
#             'config_files': config_files,
#             'custom_config': processed_custom_config,
#             'uav_name': uav_name,
#             'simulation': simulation,
#             'world_frame_id': world_frame,
#             'robot_frame_id': robot_frame,
#             'map_path': '/tmp/',
#             'vizualization/frame_id': world_frame,
#             'topic_prefix': [uav_name, '/'],
#         }],
#         remappings=[
#             # 3D Lidar
#             #("~/lidar_3d_0_in", '~/lidar/points'),
#             ("~/lidar_3d_0_in", '~/lidar_3d_topic_0'),
#             ("~/lidar_3d_1_in", '~/lidar_3d_topic_1'),
#             ("~/lidar_3d_2_in", '~/lidar_3d_topic_2'),
#             ("~/lidar_3d_0_over_max_range_in", '~/lidar_3d_topic_0_over_max_range'),
#             ("~/lidar_3d_1_over_max_range_in", '~/lidar_3d_topic_1_over_max_range'),
#             ("~/lidar_3d_2_over_max_range_in", '~/lidar_3d_topic_2_over_max_range'),

#             # 2D Lidar
#             ("~/lidar_2d_0_in", '~/lidar_2d_topic_0'),
#             ("~/lidar_2d_1_in", '~/lidar_2d_topic_1'),
#             ("~/lidar_2d_2_in", '~/lidar_2d_topic_2'),

#             # Depth Camera
#             ("~/depth_camera_0_in", '~/depth_camera_topic_0'),
#             ("~/depth_camera_1_in", '~/depth_camera_topic_1'),
#             ("~/depth_camera_2_in", '~/depth_camera_topic_2'),
#             ("~/depth_camera_0_over_max_range_in", '~/depth_camera_topic_0_over_max_range'),
#             ("~/depth_camera_1_over_max_range_in", '~/depth_camera_topic_1_over_max_range'),
#             ("~/depth_camera_2_over_max_range_in", '~/depth_camera_topic_2_over_max_range'),

#             # Camera Info
#             ("~/camera_info_0_in", '~/camera_info_topic_0'),
#             ("~/camera_info_1_in", '~/camera_info_topic_1'),
#             ("~/camera_info_2_in", '~/camera_info_topic_2'),

#             # Other remappings
#             ("~/control_manager_diagnostics_in", "~/control_manager/diagnostics"),
#             ("~/height_in", "~/odometry/height"),
#             ("~/clear_box_in", "~/uav_pose_estimator/clear_box"),

#             # Topics out
#             ("~/octomap_global_full_out", "~/octomap_global_full"),
#             ("~/octomap_global_binary_out", "~/octomap_global_binary"),
#             ("~/octomap_local_full_out", "~/octomap_local_full"),
#             ("~/octomap_local_binary_out", "~/octomap_local_binary"),

#             # Services
#             ("~/reset_map_in", "~/reset_map"),
#             ("~/save_map_in", "~/save_map"),
#             ("~/load_map_in", "~/load_map"),
#         ]
#     )

#     load_into_existing = LoadComposableNodes(
#         target_container=container_name,
#         composable_node_descriptions=[Octomap_node],
#         condition=UnlessCondition(standalone)
#     )
#     ld.add_action(load_into_existing)




#     standalone_container = ComposableNodeContainer(
#         namespace=uav_name,
#         #name= uav_name + '_octomap_server_container',
#         name = 'octomap_server_container',
#         #name='octomap_server_container',
#         package='rclcpp_components',
#         executable='component_container_mt',
#         output='screen',
#         arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
#         composable_node_descriptions=[Octomap_node],
#         parameters=[
#             {'use_intra_process_comms': True},
#             {'thread_num': os.cpu_count()},
#             {'use_sim_time': use_sim_time},
#         ],
#         condition=IfCondition(standalone)
#     )
#     ld.add_action(standalone_container)

#     return ld
