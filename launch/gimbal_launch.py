#!/usr/bin/env python3
"""
Launch file for Gremsy Camera Stream Node with configurable namespace.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    EnvironmentVariable,
    TextSubstitution
)
from launch_ros.actions import Node


def generate_launch_description():
    """
    Generate launch description for Gremsy camera stream node.
    """
    
    # Declare launch arguments
    rgb_namespace_arg = DeclareLaunchArgument(
        'rgb_namespace',
        default_value=[
            TextSubstitution(text='/'),
            EnvironmentVariable('ROBOT_NAME', default_value='robot'),
            TextSubstitution(text='/base')
        ],
        description='Namespace for the Gremsy camera stream node'
    )

    # Define the Gremsy Camera Stream Node
    gremsy_camera_node = Node(
        package='PayloadSDK',
        executable='gremsy_camera_stream_node',
        name='gremsy_camera_stream_node',
        namespace=LaunchConfiguration('rgb_namespace'),
        output='screen',
        # FIXED: Add camera parameters to publish ONLY raw images
        parameters=[{
            # Compression settings - DISABLE COMPRESSION, ONLY RAW IMAGES
            'camera.compression.enable': False,
            'camera.compression.publish_raw': True,
            'camera.compression.publish_both': False,
            'camera.compression.jpeg_quality': 80,
            
            # Camera settings
            'camera.stream_mode': 'RGB',
            'camera.rgb_enabled': True,
            'camera.ir_enabled': False,
            'camera.auto_start_stream': True,
            
            # Network settings (adjust as needed)
            'network.udp_ip_target': '10.4.1.13',
            'network.udp_port_target': 14566,
            'connection.control_method': 1,
            
            # Publishing rate
            'node.publish_rate': 30.0
        }]
    )

    # Define the Gremsy Gimbal Node
    gremsy_gimbal_node = Node(
        package='PayloadSDK',
        executable='gremsy_gimbal_node',
        name='gremsy_gimbal_node',
        namespace=LaunchConfiguration('rgb_namespace'),
        output='screen',
        # Add any parameters here if needed
        # parameters=[{
        #     'param_name': 'param_value'
        # }]
    )

    return LaunchDescription([
        # Launch arguments
        rgb_namespace_arg,
        
        # Nodes
        gremsy_camera_node,
        gremsy_gimbal_node
    ])