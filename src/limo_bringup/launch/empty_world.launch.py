#!/usr/bin/env python3

import os
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable, 
    DeclareLaunchArgument, 
    IncludeLaunchDescription,
    RegisterEventHandler
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    # Simple world SDF content
    simple_world_sdf = """<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>
    
    <physics name="default_physics" default="true" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics">
    </plugin>
    
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands">
    </plugin>
    
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster">
    </plugin>
  </world>
</sdf>"""
    
    # Write the simple world to a temporary file
    temp_world = tempfile.NamedTemporaryFile(mode='w', suffix='.sdf', delete=False)
    temp_world.write(simple_world_sdf)
    temp_world.close()

    # Controller configuration
    robot_controllers = PathJoinSubstitution([
        FindPackageShare('limo_description'),
        'config',
        'limo_controller.yaml',
    ])

    # Set environment variables for model paths
    set_gz_resource_path1 = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(get_package_share_directory('limo_description'), 'models')
    )
    
    set_gz_resource_path2 = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        get_package_share_directory('limo_description')
    )

    # Launch Gazebo
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v4 ', temp_world.name], 'on_exit_shutdown': 'true'}.items()
    )
    
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v4 '}.items()
    )

    # Get the URDF file for robot_state_publisher (needed by gz_ros2_control)
    urdf_path = os.path.join(
        get_package_share_directory('limo_description'),
        'urdf',
        'limo_ackerman.xacro'
    )

    # Get the SDF file for spawning
    sdf_path = os.path.join(
        get_package_share_directory('limo_description'),
        'models',
        'limo_ackerman',
        'model.sdf'
    )

    # Robot state publisher (needed by gz_ros2_control plugin)
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', urdf_path])
        }],
    )

    # Spawn robot from SDF file
    spawn_turtlebot_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'limo_ackerman',
            '-file', sdf_path,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01'
        ],
        output='screen',
    )

    # Bridge for basic topics  
    bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/limo/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/limo/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen',
    )

    # Image bridge for camera topics
    image_bridge_cmd = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=[
            '/limo/color/image_raw',
            '/limo/depth/image_raw'
        ],
        output='screen',
    )

    # Controller spawners (using proper spawner nodes like the working example)
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    velocity_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'velocity_controller',
            '--param-file',
            robot_controllers,
        ],
        output='screen'
    )

    steering_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'steering_controller',
            '--param-file',
            robot_controllers,
        ],
        output='screen'
    )

    ld = LaunchDescription()

    # Add environment variables first
    ld.add_action(set_gz_resource_path1)
    ld.add_action(set_gz_resource_path2)
    
    # Add launch arguments
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true'))
    ld.add_action(DeclareLaunchArgument('x_pose', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('y_pose', default_value='0.0'))

    # Launch Gazebo, robot_state_publisher, and spawn robot
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(bridge_cmd)
    ld.add_action(image_bridge_cmd)
    ld.add_action(spawn_turtlebot_cmd)

    # Event handlers to ensure proper startup sequence (like the working example)
    ld.add_action(RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_turtlebot_cmd,
            on_exit=[joint_state_broadcaster_spawner],
        )
    ))
    
    ld.add_action(RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[velocity_controller_spawner, steering_controller_spawner],
        )
    ))

    return ld