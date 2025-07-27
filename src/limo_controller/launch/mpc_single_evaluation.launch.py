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
    """Launch file for single MPC evaluation experiment"""
    
    # Declare arguments
    experiment_name_arg = DeclareLaunchArgument(
        'experiment_name',
        default_value='mpc_single_test',
        description='Name of the experiment'
    )
    
    duration_arg = DeclareLaunchArgument(
        'duration',
        default_value='120.0',
        description='Experiment duration in seconds'
    )
    
    target_speed_arg = DeclareLaunchArgument(
        'target_speed',
        default_value='0.8',
        description='Target speed for MPC controller'
    )
    
    horizon_arg = DeclareLaunchArgument(
        'horizon_length',
        default_value='10',
        description='MPC prediction horizon length'
    )
    
    control_dt_arg = DeclareLaunchArgument(
        'control_dt',
        default_value='0.1',
        description='MPC control time step'
    )
    
    save_dir_arg = DeclareLaunchArgument(
        'save_directory',
        default_value='~/mpc_evaluation',
        description='Directory to save results'
    )
    
    # Robot description and simulation launch
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
    
    # Forward kinematics node
    forward_kinematics = Node(
        package='limo_controller',
        executable='forward_kinematics.py',
        name='fk_node',
        output='screen'
    )
    
    # Inverse kinematics node  
    inverse_kinematics = Node(
        package='limo_controller',
        executable='inverse_kinematics.py',
        name='ik_node',
        output='screen',
        parameters=[{'mode': 'car'}]
    )
    
    # MPC controller with parameters
    mpc_controller = Node(
        package='limo_controller',
        executable='mpc.py',
        name='mpc_node',
        output='screen',
        parameters=[
            {'target_speed': LaunchConfiguration('target_speed')},
            {'horizon_length': LaunchConfiguration('horizon_length')},
            {'control_dt': LaunchConfiguration('control_dt')},
            {'mode': 'car'}
        ]
    )
    
    # Performance evaluator
    performance_evaluator = Node(
        package='limo_controller',
        executable='mpc_evaluator.py',
        name='mpc_evaluator',
        output='screen',
        parameters=[
            {'experiment_name': LaunchConfiguration('experiment_name')},
            {'evaluation_duration': LaunchConfiguration('duration')},
            {'save_directory': LaunchConfiguration('save_directory')},
            {'enable_plots': True},
            {'enable_real_time_plots': False}
        ]
    )
    
    # Delayed start for MPC controller (after simulation is ready)
    delayed_mpc = TimerAction(
        period=10.0,
        actions=[mpc_controller]
    )
    
    # Delayed start for evaluator (after MPC is ready)
    delayed_evaluator = TimerAction(
        period=15.0,
        actions=[performance_evaluator]
    )
    
    return LaunchDescription([
        experiment_name_arg,
        duration_arg,
        target_speed_arg,
        horizon_arg,
        control_dt_arg,
        save_dir_arg,
        limo_description,
        forward_kinematics,
        inverse_kinematics,
        delayed_mpc,
        delayed_evaluator
    ])