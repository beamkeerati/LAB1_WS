import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_prefix, get_package_share_directory

def generate_launch_description():

    pkg_limo_controller = get_package_share_directory('limo_controller')
    pkg_limo_description = get_package_share_directory('limo_description')

    # Declare launch argument to choose controller type: pid, stanley, or pure_pursuit
    controller_arg = DeclareLaunchArgument(
        'controller',
        default_value='pid',
        description='Controller type: pid, stanley, or pure_pursuit'
    )

    # Include a description launch file from the limo_description package.
    limo_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_limo_description, 'launch', 'construct_ekf.launch.py')
        )
    )

    # Launch rqt_robot_steering (currently defined but not added; uncomment if needed)
    rqt_robot_steering = ExecuteProcess(
        cmd=['ros2', 'run', 'rqt_robot_steering', 'rqt_robot_steering'],
        output='screen'
    )
    # Node for EKF
    ekf = Node(
        package="limo_controller",
        executable="ekf.py",
        name="ekf_node",
        output="screen"
    )

    # Node for forward kinematics
    odom = Node(
        package="limo_controller",
        executable="forward_kinematics.py",
        name="fk_node",
        output="screen"
    )
    
    # Node for Pure Pursuit controller, with condition
    pure_pursuit = Node(
        package="limo_controller",
        executable="pure_pursuit.py",
        name="pure_pursuit_node",
        output="screen",
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('controller'), "' == 'pure_pursuit'"]))
    )
    
    # Node for Stanley controller, with condition
    stanley = Node(
        package="limo_controller",
        executable="stanley.py",
        name="stanley_node",
        output="screen",
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('controller'), "' == 'stanley'"]))
    )

    # Node for PID controller (to be delayed), with condition
    pid = Node(
        package="limo_controller",
        executable="pid.py",
        name="pid_node",
        output="screen",
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('controller'), "' == 'pid'"]))
    )

    # TimerAction to delay the PID node launch by 10 seconds.
    delayed_pid = TimerAction(
        period=10.0,
        actions=[pid]
    )

    # Create the launch description and populate it with the actions.
    ld = LaunchDescription()
    ld.add_action(controller_arg)
    ld.add_action(odom)
    ld.add_action(ekf)
    ld.add_action(limo_description)
    # Add the conditional controller nodes.
    ld.add_action(delayed_pid)
    ld.add_action(stanley)
    ld.add_action(pure_pursuit)
    # Optionally add rqt_robot_steering if needed:
    # ld.add_action(rqt_robot_steering)

    return ld
