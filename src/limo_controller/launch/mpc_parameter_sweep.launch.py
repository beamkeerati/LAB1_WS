#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    TimerAction,
    OpaqueFunction,
)
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """Working parameter sweep launch file based on successful single evaluation approach"""
    
    # Declare arguments
    sweep_config_arg = DeclareLaunchArgument(
        'sweep_config',
        default_value='mpc_sweep_config.yaml',
        description='Parameter sweep configuration file'
    )
    
    results_dir_arg = DeclareLaunchArgument(
        'results_directory',
        default_value='mpc_parameter_sweep',
        description='Directory to save sweep results'
    )
    
    experiment_duration_arg = DeclareLaunchArgument(
        'experiment_duration',
        default_value='200.0',  # Reduced for faster testing
        description='Maximum duration of each experiment in seconds'
    )
    
    timeout_duration_arg = DeclareLaunchArgument(
        'timeout_duration',
        default_value='150.0',  # Reduced timeout
        description='Timeout duration for experiments that hang'
    )
    
    # Get package directories
    pkg_limo_controller = get_package_share_directory("limo_controller")
    pkg_limo_description = get_package_share_directory("limo_description")

    # Robot description and simulation launch (SAME AS WORKING VERSION)
    limo_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_limo_description, "launch", "ign", "limo_ackerman_gz_path.launch.py"
            )
        )
    )
    
    # Forward kinematics node (SAME AS WORKING VERSION)
    forward_kinematics = Node(
        package='limo_controller',
        executable='forward_kinematics.py',
        name='fk_node',
        output='screen'
    )
    
    # Inverse kinematics node (SAME AS WORKING VERSION)
    inverse_kinematics = Node(
        package='limo_controller',
        executable='inverse_kinematics.py',
        name='ik_node',
        output='screen',
        parameters=[{'mode': 'car'}]
    )
    
    # MPC controller with default parameters (SAME AS WORKING VERSION)
    mpc_controller = Node(
        package='limo_controller',
        executable='mpc.py',
        name='mpc_node',
        output='screen',
        parameters=[
            {'target_speed': 0.5},  # Default values that work
            {'horizon_length': 8},
            {'control_dt': 0.1},
            {'max_steer_deg': 10.0},
            {'position_weight': 10.0},
            {'yaw_weight': 15.0},
            {'control_weight': 0.1},
            {'path_type': 'yaml'},  # Start with working YAML path
            {'use_yaml_path': True},
            {'yaml_path_file': 'path.yaml'},
            {'mode': 'car'}
        ]
    )
    
    # Parameter sweep controller node
    parameter_sweep = Node(
        package='limo_controller',
        executable='parameter_sweep.py',
        name='mpc_parameter_sweep',
        output='screen',
        parameters=[
            {'sweep_config_file': LaunchConfiguration('sweep_config')},
            {'results_directory': LaunchConfiguration('results_directory')},
            {'experiment_duration': LaunchConfiguration('experiment_duration')},
            {'timeout_duration': LaunchConfiguration('timeout_duration')}
        ]
    )
    
    # Use SAME timing as working single evaluation
    delayed_kinematics = TimerAction(
        period=5.0,
        actions=[forward_kinematics, inverse_kinematics]
    )
    
    delayed_mpc = TimerAction(
        period=10.0,  # Same as working version
        actions=[mpc_controller]
    )
    
    delayed_sweep = TimerAction(
        period=15.0,  # Start sweep after MPC is ready
        actions=[parameter_sweep]
    )
    
    return LaunchDescription([
        sweep_config_arg,
        results_dir_arg,
        experiment_duration_arg,
        timeout_duration_arg,
        limo_description,
        delayed_kinematics,
        delayed_mpc,
        delayed_sweep
    ])