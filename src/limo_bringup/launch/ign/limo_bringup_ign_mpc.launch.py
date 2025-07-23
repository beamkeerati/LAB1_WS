import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_prefix, get_package_share_directory


def generate_launch_description():

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
    odom = Node(
        package="limo_controller",
        executable="forward_kinematics.py",
        name="fk_node",
        output="screen",
    )

    # IMPROVED MPC controller with better tracking parameters
    mpc = Node(
        package="limo_controller",
        executable="mpc_4states.py",  # Use the improved version
        name="mpc_node",
        output="screen",
        # parameters=[
        #     {"target_speed": 0.8},
        #     {"horizon_length": 15},
        #     {"control_dt": 0.05},
        #     {"max_speed": 1.2},
        #     {"max_steer_deg": 8.0},
        #     {"position_weight": 10.0},
        #     {"yaw_weight": 5.0},
        #     {"mode": "car"},
        # ],
    )

    # Inverse kinematics node
    inv_kin = Node(
        package="limo_controller",
        executable="inverse_kinematics.py",
        name="ik_node",
        output="screen",
        parameters=[{"mode": "car"}],
    )

    # NEW: Path tracking measurement node
    tracking_measurement = Node(
        package="limo_controller",
        executable="path_tracking_measurement.py",
        name="tracking_measurement_node",
        output="screen",
        parameters=[
            {"save_interval": 5.0},  # Save data every 5 seconds
            {"max_data_points": 5000},
            {"output_file": "mpc_tracking_results.yaml"},
        ],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(
        DeclareLaunchArgument(
            "enable_tracking_measurement",
            default_value="false",
            description="Enable path tracking measurement node",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "mpc_target_speed",
            default_value="0.8",
            description="MPC target speed in m/s",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "mpc_horizon",
            default_value="15",
            description="MPC prediction horizon length",
        )
    )

    # Add all nodes
    ld.add_action(limo_description)
    ld.add_action(odom)
    ld.add_action(mpc)  # Now using improved MPC
    ld.add_action(inv_kin)

    # Conditionally add tracking measurement
    ld.add_action(
        Node(
            package="limo_controller",
            executable="path_tracking_measurement.py",
            name="tracking_measurement_node",
            output="screen",
            parameters=[
                {"save_interval": 5.0},
                {"max_data_points": 5000},
                {"output_file": "mpc_tracking_results.yaml"},
            ],
            condition=IfCondition(LaunchConfiguration("enable_tracking_measurement")),
        )
    )

    return ld
