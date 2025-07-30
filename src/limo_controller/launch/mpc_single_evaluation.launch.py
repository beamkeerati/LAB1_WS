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
    """Launch file for single MPC evaluation experiment with dual path support"""
    
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
        default_value='1.1',
        description='Target speed for MPC controller'
    )
    
    horizon_arg = DeclareLaunchArgument(
        'horizon_length',
        default_value='50',
        description='MPC prediction horizon length'
    )
    
    control_dt_arg = DeclareLaunchArgument(
        'control_dt',
        default_value='0.1',
        description='MPC control time step'
    )
    
    save_dir_arg = DeclareLaunchArgument(
        'save_directory',
        default_value='mpc_evaluation',
        description='Directory to save results'
    )
    
    # Path configuration arguments - UPDATED with clearer options
    path_type_arg = DeclareLaunchArgument(
        'path_type',
        default_value='yaml',
        description='Path type: "yaml" for YAML file, "switch_back" for built-in switch back course'
    )
    
    yaml_path_file_arg = DeclareLaunchArgument(
        'yaml_path_file',
        default_value='path.yaml',
        description='YAML path file name (only used when path_type=yaml)'
    )
    
    # MPC tuning parameters
    position_weight_arg = DeclareLaunchArgument(
        'position_weight',
        default_value='10.0',
        description='Position tracking weight in MPC cost function'
    )
    
    yaw_weight_arg = DeclareLaunchArgument(
        'yaw_weight',
        default_value='15.0',
        description='Yaw tracking weight in MPC cost function'
    )
    
    control_weight_arg = DeclareLaunchArgument(
        'control_weight',
        default_value='0.1',
        description='Control cost weight in MPC cost function'
    )
    
    max_steer_deg_arg = DeclareLaunchArgument(
        'max_steer_deg',
        default_value='10.0',
        description='Maximum steering angle in degrees'
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
    
    # MPC controller with enhanced parameters
    mpc_controller = Node(
        package='limo_controller',
        executable='mpc.py',
        name='mpc_node',
        output='screen',
        parameters=[
            {'target_speed': LaunchConfiguration('target_speed')},
            {'horizon_length': LaunchConfiguration('horizon_length')},
            {'control_dt': LaunchConfiguration('control_dt')},
            {'max_steer_deg': LaunchConfiguration('max_steer_deg')},
            {'position_weight': LaunchConfiguration('position_weight')},
            {'yaw_weight': LaunchConfiguration('yaw_weight')},
            {'control_weight': LaunchConfiguration('control_weight')},
            {'path_type': LaunchConfiguration('path_type')},
            {'yaml_path_file': LaunchConfiguration('yaml_path_file')},
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
            {'enable_real_time_plots': True}
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
        path_type_arg,
        yaml_path_file_arg,
        position_weight_arg,
        yaw_weight_arg,
        control_weight_arg,
        max_steer_deg_arg,
        limo_description,
        forward_kinematics,
        inverse_kinematics,
        delayed_mpc,
        delayed_evaluator
    ])