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
    """Launch file for automated parameter sweep"""
    
    # Declare arguments
    sweep_config_arg = DeclareLaunchArgument(
        'sweep_config',
        default_value='mpc_sweep_config.yaml',
        description='Parameter sweep configuration file'
    )
    
    results_dir_arg = DeclareLaunchArgument(
        'results_directory',
        default_value='~/mpc_parameter_sweep',
        description='Directory to save sweep results'
    )
    
    experiment_duration_arg = DeclareLaunchArgument(
        'experiment_duration',
        default_value='60.0',
        description='Duration of each experiment in seconds'
    )
    
    parallel_arg = DeclareLaunchArgument(
        'enable_parallel',
        default_value='false',
        description='Enable parallel experiment execution'
    )
    
    max_concurrent_arg = DeclareLaunchArgument(
        'max_concurrent',
        default_value='1',
        description='Maximum concurrent experiments'
    )
    
    # Robot simulation (will be restarted for each experiment)
    pkg_limo_controller = get_package_share_directory("limo_controller")
    pkg_limo_description = get_package_share_directory("limo_description")

    # Description launch
    limo_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_limo_description, "launch", "ign", "limo_ackerman_gz_path.launch.py"
            )
        )
    )

        
    # Forward and inverse kinematics
    forward_kinematics = Node(
        package='limo_controller',
        executable='forward_kinematics.py',
        name='fk_node',
        output='screen'
    )
    
    inverse_kinematics = Node(
        package='limo_controller',
        executable='inverse_kinematics.py', 
        name='ik_node',
        output='screen',
        parameters=[{'mode': 'car'}]
    )
    
    # Parameter sweep controller
    parameter_sweep = Node(
        package='limo_controller',
        executable='parameter_sweep.py',
        name='mpc_parameter_sweep',
        output='screen',
        parameters=[
            {'sweep_config_file': LaunchConfiguration('sweep_config')},
            {'results_directory': LaunchConfiguration('results_directory')},
            {'experiment_duration': LaunchConfiguration('experiment_duration')},
            {'enable_parallel': LaunchConfiguration('enable_parallel')},
            {'max_concurrent_experiments': LaunchConfiguration('max_concurrent')}
        ]
    )
    
    # Delayed start for parameter sweep (after simulation is ready)
    delayed_sweep = TimerAction(
        period=10.0,
        actions=[parameter_sweep]
    )
    
    return LaunchDescription([
        sweep_config_arg,
        results_dir_arg,
        experiment_duration_arg,
        parallel_arg,
        max_concurrent_arg,
        limo_description,
        forward_kinematics,
        inverse_kinematics,
        delayed_sweep
    ])