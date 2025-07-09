#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Get package share directory
    pkg_share = FindPackageShare('PayloadSDK')
    
    # Path to default config file
    default_config = PathJoinSubstitution([
        pkg_share, 
        'config', 
        'config.yaml'  # or 'config_uav.yaml' if that's your file
    ])
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_config,
        description='Full path to the ROS2 parameters file'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error)'
    )
    
    # Create the gimbal node
    gimbal_node = Node(
        package='PayloadSDK',
        executable='gremsysdk',
        name='gremsy_gimbal_node',  # This should match the node name in your cpp file
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('params_file')],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )
    
    return LaunchDescription([
        config_file_arg,
        log_level_arg,
        gimbal_node
    ])