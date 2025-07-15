from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('gremsy_ros2')  # Replace with your actual package name
    
    # Path to config file
    config_file = os.path.join(pkg_dir, 'config', 'config_uav.yaml')
    
    # Check if config file exists
    if not os.path.exists(config_file):
        raise FileNotFoundError(f"Config file not found: {config_file}")
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='INFO',
        description='Log level for all nodes'
    )
    
    enable_gimbal_arg = DeclareLaunchArgument(
        'enable_gimbal',
        default_value='true',
        description='Enable gimbal control node'
    )
    
    enable_camera_stream_arg = DeclareLaunchArgument(
        'enable_camera_stream',
        default_value='true',
        description='Enable camera streaming node'
    )
    
    enable_camera_move_arg = DeclareLaunchArgument(
        'enable_camera_move',
        default_value='true',
        description='Enable camera movement node'
    )
    
    # Common parameters for all nodes
    common_parameters = [
        config_file,
        {
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }
    ]
    
    # Node 1: Gimbal Control and Status
    gimbal_node = Node(
        package='gremsy_ros2',  # Replace with your actual package name
        executable='gremsy_gimbal_node',
        name='gremsy_gimbal_node',
        namespace='gremsy',
        parameters=common_parameters,
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        condition=IfCondition(LaunchConfiguration('enable_gimbal'))
    )
    
    # Node 2: Camera Streaming
    camera_stream_node = Node(
        package='gremsy_ros2',  # Replace with your actual package name
        executable='gremsy_camera_stream_node',
        name='gremsy_camera_stream_node',
        namespace='gremsy',
        parameters=common_parameters,
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        condition=IfCondition(LaunchConfiguration('enable_camera_stream'))
    )
    
    # Node 3: Camera Movement Control
    # camera_move_node = Node(
    #     package='gremsy_ros2',  # Replace with your actual package name
    #     executable='gremsy_camera_move_node',
    #     name='gremsy_camera_move_node',
    #     namespace='gremsy',
    #     parameters=common_parameters,
    #     output='screen',
    #     emulate_tty=True,
    #     arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    #     condition=IfCondition(LaunchConfiguration('enable_camera_move'))
    # )
    
    # Group all nodes together
    gremsy_group = GroupAction([
        gimbal_node,
        camera_stream_node,
        camera_move_node
    ])
    
    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        log_level_arg,
        enable_gimbal_arg,
        enable_camera_stream_arg,
        enable_camera_move_arg,
        
        # Nodes group
        gremsy_group
    ])