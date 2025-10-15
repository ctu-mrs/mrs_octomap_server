#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch.substitutions import IfElseSubstitution
from launch.substitutions import PythonExpression


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

    # ###########################
    # ## General Arguments ##
    # ###########################

    ld.add_action(DeclareLaunchArgument(
        'UAV_NAME',
        default_value=EnvironmentVariable('UAV_NAME', default_value='uav1'),
        description="Name of the UAV, used for namespacing and frame IDs."
    ))
    uav_name = LaunchConfiguration('UAV_NAME')


    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description="Whether to use simulation time."
    ))
    use_sim_time = LaunchConfiguration('use_sim_time')


    ld.add_action(
        DeclareLaunchArgument(
            'custom_config',
            default_value='',
            description="Path to the custom configuration file. The path can be absolute, starting with '/' or relative to the current working directory",
        )
    )
    custom_config_path = LaunchConfiguration('custom_config')


    processed_custom_config = IfElseSubstitution(
            condition=PythonExpression(['"', custom_config_path, '" != "" and ', 'not "', custom_config_path, '".startswith("/")']),
            if_value=PathJoinSubstitution([EnvironmentVariable('PWD'), custom_config_path]),
            else_value=custom_config_path
            )
    
    # ############################
    # ## Sensor Topic Arguments ##
    # ############################

    ld.add_action(DeclareLaunchArgument(
    'lidar_3d_topic_0_in',
    default_value=PathJoinSubstitution([uav_name, 'lidar/points']),
    description='Input topic for 3D Lidar 0 point cloud.'
    ))

    #ld.add_action(DeclareLaunchArgument('lidar_3d_topic_0_in', default_value=uav_name+ '/lidar/points', description='Input topic for 3D Lidar 0 point cloud.'))
    ld.add_action(DeclareLaunchArgument('lidar_3d_topic_0_over_max_range_in', default_value='lidar_3d_0_over_max_range_in', description='Input topic for 3D Lidar 0 points over max range.'))

    # ########################
    # ## Frame ID Arguments ##
    # ########################
    
    world_frame = PathJoinSubstitution([uav_name, 'os_lidar'])
    robot_frame = PathJoinSubstitution([uav_name, 'fcu'])

    # ###################################
    # ## Composable Node and Container ##
    # ###################################
    
    config_files = [
        os.path.join(this_pkg_path, 'config', 'default.yaml'),
    ]

    ld.add_action(ComposableNodeContainer(
        namespace=uav_name,
        # name= uav_name_str +'_octomap_server_container',
        name='octomap_server_container',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        arguments=['--ros-args', '--log-level', 'INFO'],  # Keep INFO level
        composable_node_descriptions=[
            ComposableNode(
                package=pkg_name,
                plugin='mrs_octomap_server::OctomapServer',
                name='octomap_server',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'simulation': use_sim_time,
                    'uav_name': uav_name,
                    'world_frame_id': world_frame,
                    'robot_frame_id': robot_frame,
                    'visualization/frame_id': world_frame,
                    'map_path': '/tmp/',
                    # Topic parameters (for direct topic access)
                    'lidar_3d_topic_0_in': LaunchConfiguration('lidar_3d_topic_0_in'),
                    'lidar_3d_topic_0_over_max_range_in': LaunchConfiguration('lidar_3d_topic_0_over_max_range_in'),
                    # Legacy parameters expected by the node
                    'custom_config': '',
                    'config_files': config_files,
                    # Logging configuration
                    'ros.logging.severity_threshold': 'INFO',  # Keep INFO level
                }],
                remappings=[

                        ("lidar_3d_0_in", LaunchConfiguration('lidar_3d_topic_0_in')),
                        ("lidar_3d_0_over_max_range_in", LaunchConfiguration('lidar_3d_topic_0_over_max_range_in')),
                        # Other remappings
                        ("~/control_manager_diagnostics_in", "control_manager/diagnostics"),
                        ("~/height_in", "odometry/height"),
                        ("~/clear_box_in", "uav_pose_estimator/clear_box"),

                        # Topics out
                        ("octomap_global_full_out", "octomap_global_full"),
                        ("octomap_global_binary_out", "octomap_global_binary"),
                        ("octomap_local_full_out", "octomap_local_full"),
                        ("octomap_local_binary_out", "octomap_local_binary"),

                        # Services
                        ("~/reset_map_in", "~/reset_map"),
                        ("~/save_map_in", "~/save_map"),
                        ("~/load_map_in", "~/load_map"),
                    ]
            )

        ],
    ))
    return ld