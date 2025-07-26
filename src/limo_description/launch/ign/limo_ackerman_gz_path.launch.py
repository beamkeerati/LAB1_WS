# Copyright 2024 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.actions import AppendEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    robot_name = LaunchConfiguration('robot_name', default='limo')
    world = LaunchConfiguration('world', default='empty.sdf')

    # Pose where we want to spawn the robot
    spawn_x_val = '9.073496746393584'
    spawn_y_val = '0'
    spawn_z_val = '0.5'
    spawn_yaw_val = '1.57'
    
    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare('limo_description'),
            'rviz',
            'ign_path.rviz',
        ]
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('limo_description'),
                 'urdf','ign', 'limo_ackerman_gz.xacro']
            ),
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    set_env_vars_resources = AppendEnvironmentVariable(
            'IGN_GAZEBO_RESOURCE_PATH',
            os.path.join(get_package_share_directory('limo_description'),
                         'models'))

    robot_world = PathJoinSubstitution(
        [
            FindPackageShare('limo_description'),
            'worlds',
            'basic_gz.world',
        ]
    )
    
    # Controller configuration
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('limo_description'),
            'config',
            'limo_controller.yaml',
        ]
    )

    # Bridge configuration file path
    bridge_config_file = PathJoinSubstitution(
        [
            FindPackageShare('limo_description'),
            'config',
            'limo_bridge.yaml',
        ]
    )
    
    # Launch RViz
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}]
    )

    # Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    # Spawn robot entity at specific position
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description', 
            '-name', robot_name, 
            '-allow_renaming', 'true',
            '-x', spawn_x_val,
            '-y', spawn_y_val,
            '-z', spawn_z_val,
            '-Y', spawn_yaw_val
        ],
    )

    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Steering Controller
    steering_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['steering_controller', '--param-file', robot_controllers],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Velocity Controller  
    velocity_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['velocity_controller', '--param-file', robot_controllers],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Bridge for sensor topics - Following README approach
    # Based on: ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=path/to/config.yaml
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='limo_bridge',
        parameters=[{'config_file': bridge_config_file}],
        output='screen'
    )

    return LaunchDescription([
        set_env_vars_resources,
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='If true, use simulated clock'),
        DeclareLaunchArgument(
            'robot_name',
            default_value='limo',
            description='Name of the robot in simulation'),
        DeclareLaunchArgument(
            'world',
            default_value=robot_world,
            description='Gazebo world file'),
        DeclareLaunchArgument(
            'spawn_x',
            default_value=spawn_x_val,
            description='X position to spawn robot'),
        DeclareLaunchArgument(
            'spawn_y', 
            default_value=spawn_y_val,
            description='Y position to spawn robot'),
        DeclareLaunchArgument(
            'spawn_z',
            default_value=spawn_z_val,
            description='Z position to spawn robot'),
        DeclareLaunchArgument(
            'spawn_yaw',
            default_value=spawn_yaw_val,
            description='Yaw orientation to spawn robot'),
            
        # Launch Gazebo environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
            launch_arguments=[('gz_args', ['-r -v 4 ', world])]),
        
        # Bridge for sensors
        bridge,
        
        # Robot state publisher
        node_robot_state_publisher,
        
        # Spawn robot
        gz_spawn_entity,
        
        # Start controllers after robot is spawned
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[steering_controller_spawner, velocity_controller_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[start_rviz_cmd],
            )
        ),
    ])