#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PythonExpression,
    PathJoinSubstitution,
    IfElseSubstitution,
)
from launch_ros.actions import Node, ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # LaunchDescription object
    ld = LaunchDescription()

    # Package and path
    pkg_name = "mrs_octomap_server"
    this_pkg_path = get_package_share_directory(pkg_name)

    # Arguments
    uav_name = LaunchConfiguration("UAV_NAME")

    ld.add_action(
        DeclareLaunchArgument(
            "UAV_NAME",
            default_value=os.getenv("UAV_NAME", "uav1"),
            description="Name of the UAV used for namespacing.",
        )
    )

    run_type = LaunchConfiguration("RUN_TYPE")
    ld.add_action(
        DeclareLaunchArgument(
            "RUN_TYPE",
            default_value=os.getenv("RUN_TYPE", ""),
            description="Type of run (simulation or real).",
        )
    )

    debug = LaunchConfiguration("debug")
    ld.add_action(
        DeclareLaunchArgument(
            "debug",
            default_value="false",
            description="Enable debug mode.",
        )
    )

    standalone = LaunchConfiguration("standalone")

    declare_standalone = DeclareLaunchArgument(
        'standalone',
        default_value='true',
        description='Whether to start a as a standalone or load into an existing container.'
    )
    ld.add_action(declare_standalone)

    custom_config = LaunchConfiguration("custom_config")
    ld.add_action(
        DeclareLaunchArgument(
            "custom_config",
            default_value="",
            description="Path to the custom configuration file. The path can be absolute, starting with '/' or relative to the current working directory",
        )
    )

    custom_config = IfElseSubstitution(
            condition=PythonExpression(['"', custom_config, '" != "" and ', 'not "', custom_config, '".startswith("/")']),
            if_value=PathJoinSubstitution([EnvironmentVariable('PWD'), custom_config]),
            else_value=custom_config
            )
    
    run_type=os.getenv('RUN_TYPE', "realworld")

    if run_type == "simulation":
        simulation = True
    else:
        simulation = False

    ld.add_action(DeclareLaunchArgument(name='log_level', default_value='info'))

    map_path = LaunchConfiguration("map_path")
    ld.add_action(
        DeclareLaunchArgument(
            "map_path",
            default_value=PathJoinSubstitution([EnvironmentVariable("HOME"), "maps"]),
            description="Path to the maps directory",
        )
    )

    world_frame_id = LaunchConfiguration("world_frame_id")
    ld.add_action(
        DeclareLaunchArgument(
            "world_frame_id",
            default_value=PythonExpression([uav_name, "'/gps_origin'"]),
            description="World frame ID.",
        )
    )

    robot_frame_id = LaunchConfiguration("robot_frame_id")
    ld.add_action(
        DeclareLaunchArgument(
            "robot_frame_id",
            default_value=PythonExpression([uav_name, "'/fcu'"]),
            description="Robot frame ID.",
        )
    )

    container_name = LaunchConfiguration('container_name')

    declare_container_name = DeclareLaunchArgument(
        'container_name',
        default_value='',
        description='Name of an existing container to load into (if standalone is false)'
    )

    ld.add_action(declare_container_name)


    platform_config = LaunchConfiguration('platform_config')

    # this adds the args to the list of args available for this launch files
    # these args can be listed at runtime using -s flag
    # default_value is required to if the arg is supposed to be optional at launch time
    ld.add_action(DeclareLaunchArgument(
        'platform_config',
        default_value="",
        description="Path to the platform configuration file. The path can be absolute, starting with '/' or relative to the current working directory",
        ))

    # behaviour:
    #     platform_config == "" => platform_config: ""
    #     platform_config == "/<path>" => platform_config: "/<path>"
    #     platform_config == "<path>" => platform_config: "$(pwd)/<path>"
    platform_config = IfElseSubstitution(
            condition=PythonExpression(['"', platform_config, '" != "" and ', 'not "', platform_config, '".startswith("/")']),
            if_value=PathJoinSubstitution([EnvironmentVariable('PWD'), platform_config]),
            else_value=platform_config
            )

    use_sim_time = LaunchConfiguration('use_sim_time')

    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value=os.getenv('USE_SIM_TIME', "false"),
        description="Should the node subscribe to sim time?",
    ))


    # Define the composable node for octomap_server
    octomap_server_node = ComposableNode(
        package=pkg_name,
        plugin="mrs_octomap_server::MrsOctomapServer",  # Replace with your plugin name
        name="octomap_server",
        namespace=uav_name,
        parameters=[
            {"uav_name": uav_name},
            {"simulation": simulation},
            {"world_frame_id": world_frame_id},
            {"robot_frame_id": robot_frame_id},
            {"map_path": map_path},
            {"private_config": this_pkg_path + "/config/default.yaml"},
            {"public_config": this_pkg_path + "/config/default.yaml"},
            {"custom_config": custom_config},
            {"platform_config": platform_config},
            {"debug": debug},
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            # 3D Lidar
            ("~lidar_3d_0_in", "lidar_3d_topic_0_in"),
            ("~lidar_3d_1_in", "lidar_3d_topic_1_in"),
            ("~lidar_3d_2_in", "lidar_3d_topic_2_in"),
            ("~lidar_3d_0_over_max_range_in", "lidar_3d_topic_0_over_max_range_in"),
            ("~lidar_3d_1_over_max_range_in", "lidar_3d_topic_1_over_max_range_in"),
            ("~lidar_3d_2_over_max_range_in", "lidar_3d_topic_2_over_max_range_in"),
            # 2D Lidar
            ("~lidar_2d_0_in", "lidar_2d_topic_0_in"),
            ("~lidar_2d_1_in", "lidar_2d_topic_1_in"),
            ("~lidar_2d_2_in", "lidar_2d_topic_2_in"),
            # Depth Camera
            ("~depth_camera_0_in", "depth_camera_topic_0_in"),
            ("~depth_camera_1_in", "depth_camera_topic_1_in"),
            ("~depth_camera_2_in", "depth_camera_topic_2_in"),
            ("~depth_camera_0_over_max_range_in", "depth_camera_topic_0_over_max_range_in"),
            ("~depth_camera_1_over_max_range_in", "depth_camera_topic_1_over_max_range_in"),
            ("~depth_camera_2_over_max_range_in", "depth_camera_topic_2_over_max_range_in"),
            # Camera Info
            ("~camera_info_0_in", "camera_info_topic_0_in"),
            ("~camera_info_1_in", "camera_info_topic_1_in"),
            ("~camera_info_2_in", "camera_info_topic_2_in"),
            # Other remappings
            ("~control_manager_diagnostics_in", "control_manager/diagnostics"),
            ("~height_in", "odometry/height"),
            ("~clear_box_in", "uav_pose_estimator/clear_box"),
            # Topics out
            ("~octomap_global_full_out", "~octomap_global_full"),
            ("~octomap_global_binary_out", "~octomap_global_binary"),
            ("~octomap_local_full_out", "~octomap_local_full"),
            ("~octomap_local_binary_out", "~octomap_local_binary"),
            # Services
            ("~reset_map_in", "~reset_map"),
            ("~save_map_in", "~save_map"),
            ("~load_map_in", "~load_map"),
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    # Load into existing container if not standalone
    load_octomap_server = LoadComposableNodes(
        target_container=container_name,
        composable_node_descriptions=[octomap_server_node],
        condition=UnlessCondition(standalone),
    )
    ld.add_action(load_octomap_server)


    standalone_container = ComposableNodeContainer(
        namespace = uav_name
        name= namespace + '_container',
        package='rclcpp_components',
        executable='component_container_mt',
        output="screen",
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        composable_node_descriptions=[octomap_server_node],
        # prefix=['debug_roslaunch ' + os.ttyname(sys.stdout.fileno())],
        parameters=[
            {'use_intra_process_comms': True},
            {'thread_num': os.cpu_count()},
            {'use_sim_time': use_sim_time},
        ],
        condition=IfCondition(standalone)
    )

    ld.add_action(standalone_container)


    return ld