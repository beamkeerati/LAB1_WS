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
    """
    SIMPLIFIED parameter sweep launch file that only starts the sweep controller.
    
    The sweep controller itself handles launching and managing all simulation processes
    for each experiment individually to avoid multiple Gazebo instances.
    """
    
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
    
    # Parameter sweep controller node - this manages everything
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
    
    return LaunchDescription([
        sweep_config_arg,
        results_dir_arg,
        experiment_duration_arg,
        timeout_duration_arg,
        parameter_sweep
    ])