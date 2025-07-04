import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_prefix, get_package_share_directory

def generate_launch_description():

    pkg_limo_controller = get_package_share_directory('limo_controller')
    pkg_limo_description = get_package_share_directory('limo_description')

    # Odometry launch
    limo_kinematics = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_limo_controller, 'launch', 'limo_kinematics.launch.py')
        )
    )
    
    # Description launch
    limo_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_limo_description, 'launch','classic', 'construct_kinematics.launch.py')
        )
    )
    
    #
    rqt_robot_steering = ExecuteProcess(
        cmd=['ros2', 'run', 'rqt_robot_steering', 'rqt_robot_steering', '--force-discover'],
        output='screen'
    )

    

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    # ld.add_action(limo_kinematics)
    ld.add_action(limo_description)
    ld.add_action(rqt_robot_steering)

    return ld