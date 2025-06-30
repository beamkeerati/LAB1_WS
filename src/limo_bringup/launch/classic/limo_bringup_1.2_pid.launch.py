import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_prefix, get_package_share_directory

def generate_launch_description():

    pkg_limo_controller = get_package_share_directory('limo_controller')
    pkg_limo_description = get_package_share_directory('limo_description')

    # Description launch from the limo_description package
    limo_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_limo_description, 'launch','classic', 'construct_path.launch.py')
        )
    )

    # Launch rqt_robot_steering (currently defined but not added to ld; add if needed)
    rqt_robot_steering = ExecuteProcess(
        cmd=['ros2', 'run', 'rqt_robot_steering', 'rqt_robot_steering', '--force-discover'],
        output='screen'
    )

    # Node for forward kinematics
    odom = Node(
        package="limo_controller",
        executable="forward_kinematics.py",
        name="fk_node",
        output="screen"
    )

    # Node for PID controller (to be delayed)
    pid = Node(
        package="limo_controller",
        executable="pid.py",
        name="pid_node",
        output="screen"
    )

    # TimerAction to delay the PID node launch by 10 seconds.
    delayed_pid = TimerAction(
        period=10.0,
        actions=[pid]
    )

    # Create the launch description and populate it with the actions.
    ld = LaunchDescription()
    ld.add_action(odom)
    ld.add_action(limo_description)
    # Add delayed PID node action after a 10-second delay.
    ld.add_action(delayed_pid)
    # Optionally add rqt_robot_steering if needed:
    # ld.add_action(rqt_robot_steering)

    return ld
