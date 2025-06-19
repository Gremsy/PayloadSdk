#!/usr/bin/env python3
"""
Gremsy Gimbal Heading Follower Launch File
==========================================
Launches the Gremsy gimbal node with drone heading following capability.
Automatically handles robot namespacing based on ROBOT_NAME environment variable.

Usage Examples:
  # Basic launch (uses ROBOT_NAME from environment)
  ros2 launch PayloadSDK gremsy_gimbal.launch.py

  # With custom gimbal IP
  ros2 launch PayloadSDK gremsy_gimbal.launch.py gimbal_ip:=192.168.1.100

  # With heading follow enabled
  ros2 launch PayloadSDK gremsy_gimbal.launch.py enable_heading_follow:=true

  # Override robot name
  ros2 launch PayloadSDK gremsy_gimbal.launch.py robot_name:=robot_3

  # Disable auto-namespacing
  ros2 launch PayloadSDK gremsy_gimbal.launch.py auto_namespace:=false

  # Use specific config file
  ros2 launch PayloadSDK gremsy_gimbal.launch.py config_file:=/path/to/config.yaml
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PythonExpression
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the package directory
    package_dir = get_package_share_directory('PayloadSDK')

    # Path to the default config file
    default_config_file = os.path.join(package_dir, 'config', 'config_uav2.yaml')

    # Get robot name from environment with fallback
    robot_name_default = EnvironmentVariable('ROBOT_NAME', default_value='robot_1')

    # Declare launch arguments with descriptions
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='Path to the gremsy configuration YAML file'
    )

    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value=robot_name_default,
        description='Robot name for topic namespacing (defaults to ROBOT_NAME env var)'
    )

    auto_namespace_arg = DeclareLaunchArgument(
        'auto_namespace',
        default_value='true',
        description='Automatically namespace drone topics with robot name (true/false)'
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Additional namespace for the gremsy node (e.g., gimbal)'
    )

    gimbal_ip_arg = DeclareLaunchArgument(
        'gimbal_ip',
        default_value='10.4.1.33',
        description='IP address of the Gremsy gimbal'
    )

    gimbal_port_arg = DeclareLaunchArgument(
        'gimbal_port',
        default_value='14566',
        description='UDP port of the Gremsy gimbal'
    )

    enable_heading_follow_arg = DeclareLaunchArgument(
        'enable_heading_follow',
        default_value='false',
        description='Enable heading follow mode on startup (true/false)'
    )

    heading_offset_arg = DeclareLaunchArgument(
        'heading_offset',
        default_value='0.0',
        description='Heading offset in degrees (0=forward, 90=right, -90=left, 180=backward)'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='INFO',
        description='Logging level (DEBUG, INFO, WARN, ERROR)'
    )

    # Parameter overrides that will be passed to the node
    parameter_overrides = {
        # Network settings
        'network.udp_ip_target': LaunchConfiguration('gimbal_ip'),
        'network.udp_port_target': LaunchConfiguration('gimbal_port'),

        # Heading follow settings
        'heading_follow.enable': LaunchConfiguration('enable_heading_follow'),
        'heading_follow.offset': LaunchConfiguration('heading_offset'),

        # Environment settings
        'environment.robot_name': LaunchConfiguration('robot_name'),
        'environment.auto_namespace': LaunchConfiguration('auto_namespace'),
    }

    # Create the main gremsy node
    gremsy_node = Node(
        package='PayloadSDK',
        executable='gremsysdk',
        name='gremsy_gimbal_node',
        namespace=LaunchConfiguration('namespace'),
        parameters=[
            LaunchConfiguration('config_file'),
            parameter_overrides
        ],
        output='screen',
        emulate_tty=True,
        respawn=True,
        respawn_delay=3.0,
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )

    # Log startup information
    startup_log = LogInfo(
        msg=[
            'Starting Gremsy Heading Follower with:',
            '\n  - Robot Name: ', LaunchConfiguration('robot_name'),
            '\n  - Auto Namespace: ', LaunchConfiguration('auto_namespace'),
            '\n  - Gimbal IP: ', LaunchConfiguration('gimbal_ip'),
            '\n  - Gimbal Port: ', LaunchConfiguration('gimbal_port'),
            '\n  - Heading Follow: ', LaunchConfiguration('enable_heading_follow'),
            '\n  - Heading Offset: ', LaunchConfiguration('heading_offset'), '°',
            '\n  - Node Namespace: "', LaunchConfiguration('namespace'), '"',
            '\n  - Config: ', LaunchConfiguration('config_file')
        ]
    )

    return LaunchDescription([
        # Arguments
        config_arg,
        robot_name_arg,
        auto_namespace_arg,
        namespace_arg,
        gimbal_ip_arg,
        gimbal_port_arg,
        enable_heading_follow_arg,
        heading_offset_arg,
        log_level_arg,

        # Log startup info
        startup_log,

        # Main node
        gremsy_node
    ])