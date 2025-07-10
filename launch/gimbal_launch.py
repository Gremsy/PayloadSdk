#!/usr/bin/env python3

import yaml
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get package share directory
    pkg_share = FindPackageShare('PayloadSDK')
    
    # Config file path
    default_config_path = PathJoinSubstitution([pkg_share, 'config', 'config.yaml'])
    
    # Declare arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_path,
        description='Path to the configuration YAML file'
    )
    
    # For now, we'll use a simple approach
    # In production, you'd use OpaqueFunction to load YAML dynamically
    
    # Create main gimbal node
    gimbal_node = Node(
        package='PayloadSDK',
        executable='gremsysdk',  # You can override this with launch argument
        name='gremsy_gimbal_node',  # Must match config file
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('config_file')],
        # The node will configure itself based on the YAML file
    )
    
    # Log what we're doing
    log_msg = LogInfo(
        msg=['Launching Gremsy gimbal with config: ', LaunchConfiguration('config_file')]
    )
    
    return LaunchDescription([
        config_file_arg,
        log_msg,
        gimbal_node
    ])