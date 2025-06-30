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

    
    # Description launch
    limo_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_limo_description, 'launch', 'construct_path.launch.py')
        )
    )
    
    #
    rqt_robot_steering = ExecuteProcess(
        cmd=['ros2', 'run', 'rqt_robot_steering', 'rqt_robot_steering', '--force-discover'],
        output='screen'
    )
    
    odom = Node(
        package="limo_controller",
        executable="forward_kinematics.py",
        name="fk_node",
        output="screen"
    )
    
    pure_pursuit = Node(
        package="limo_controller",
        executable="pure_pursuit.py",
        name="pure_pursuit_node",
        output="screen"
    )

    

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(odom)
    ld.add_action(limo_description)
    ld.add_action(pure_pursuit)

    return ld