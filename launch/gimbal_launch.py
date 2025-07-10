#!/usr/bin/env python3

import yaml
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import OpaqueFunction


def launch_setup(context, *args, **kwargs):
    pkg_share = FindPackageShare('PayloadSDK').perform(context)
    config_path = os.path.join(pkg_share, 'config', 'config_uav.yaml')  # hardcoded, or use LaunchConfiguration
    
    # Load YAML
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    # Extract executable name from YAML
    executable_name = config.get('node', {}).get('executable', 'gremsysdk')  # fallback to default

    # Extract node name (optional)
    node_name = config.get('node', {}).get('name', 'gremsy_gimbal_node')

    return [
        LogInfo(msg=f'Launching {executable_name} as node {node_name} using config {config_path}'),
        Node(
            package='PayloadSDK',
            executable=executable_name,
            name=node_name,
            namespace='',
            output='screen',
            emulate_tty=True,
            parameters=[config_path]
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
