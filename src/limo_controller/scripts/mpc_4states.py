#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterAlreadyDeclaredException
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import (
    Point,
    Pose,
    PoseStamped,
    PoseWithCovariance,
    Quaternion,
    Twist,
    TwistWithCovariance,
    Vector3,
    TransformStamped,
)
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64, Float64MultiArray
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import math
import numpy as np
import os
import yaml
import cvxpy
from ament_index_python.packages import get_package_share_directory

import numpy as np
import math
import cvxpy
import matplotlib.pyplot as plt
from PathPlanning.CubicSpline import cubic_spline_planner
from mpc_lib import *


# ============================================================================
# IMPROVED PARAMETER CONFIGURATION FOR BETTER TRACKING
# ============================================================================
class MPCConfig:
    """Improved configuration class for better MPC path tracking"""

    # State and Control Dimensions
    NX = 4  # x = x, y, v, yaw
    NU = 2  # a = [accel, steer]
    T = 15  # INCREASED horizon length for better lookahead

    # IMPROVED MPC Cost Matrices for better tracking
    R = np.diag([0.1, 0.1])  # INCREASED input cost to reduce aggressiveness
    Rd = np.diag([0.05, 0.5])  # BALANCED input difference cost
    Q = np.diag([10.0, 10.0, 1.0, 5.0])  # INCREASED position and yaw tracking weights
    Qf = np.diag([15.0, 15.0, 1.0, 10.0])  # HIGHER terminal cost for stability

    # Goal and Stopping Parameters
    GOAL_DIS = 1.0  # REDUCED for more precise goal reaching
    STOP_SPEED = 0.2 / 3.6  # REDUCED stop speed
    MAX_TIME = 500.0

    # Iterative Parameters
    MAX_ITER = 5  # INCREASED for better convergence
    DU_TH = 0.05  # REDUCED threshold for more precise convergence

    # Speed Parameters
    TARGET_SPEED = 0.8  # REDUCED target speed for better tracking
    N_IND_SEARCH = 15  # INCREASED search range

    # Time Step
    DT = 0.05  # INCREASED time step for more reasonable horizon

    # Vehicle Physical Parameters
    LENGTH = 4.5
    WIDTH = 2.0
    BACKTOWHEEL = 1.0
    WHEEL_LEN = 0.3
    WHEEL_WIDTH = 0.2
    TREAD = 0.7
    WB = 0.2

    # Robot-specific Parameters
    WHEEL_RADIUS = 0.045
    WHEEL_BASE = 0.2
    TRACK_WIDTH = 0.14

    # IMPROVED Control Constraints
    MAX_STEER = math.radians(8.0)  # REDUCED max steering for smoother control
    MAX_DSTEER = math.radians(3.0)  # REDUCED steering rate for smoother control
    MAX_SPEED = 1.2  # Slightly increased
    MIN_SPEED = -0.5  # Limited reverse speed
    MAX_ACCEL = 0.3  # REDUCED max acceleration for smoother control
    MAX_LINEAR_VEL = 1.2
    MIN_LINEAR_VEL = -0.5
    MAX_ANGULAR_VEL = math.radians(15)  # INCREASED for better turning
    MIN_ANGULAR_VEL = -math.radians(15)

    # Safety Parameters
    MAX_MPC_FAILURES = 5  # REDUCED for faster response to failures

    @classmethod
    def get_dict(cls):
        """Return all parameters as a dictionary for easy passing to other modules"""
        return {
            attr: getattr(cls, attr)
            for attr in dir(cls)
            if not attr.startswith("_") and not callable(getattr(cls, attr))
        }

    @classmethod
    def update_from_ros_params(cls, node):
        """Update parameters from ROS parameter server"""
        try:
            # Update target speed if provided
            if node.has_parameter("target_speed"):
                cls.TARGET_SPEED = (
                    node.get_parameter("target_speed")
                    .get_parameter_value()
                    .double_value
                )
                node.get_logger().info(f"Updated TARGET_SPEED to {cls.TARGET_SPEED}")

            # Update horizon length if provided
            if node.has_parameter("horizon_length"):
                cls.T = (
                    node.get_parameter("horizon_length")
                    .get_parameter_value()
                    .integer_value
                )
                node.get_logger().info(f"Updated horizon length T to {cls.T}")

            # Update control frequency if provided
            if node.has_parameter("control_dt"):
                cls.DT = (
                    node.get_parameter("control_dt").get_parameter_value().double_value
                )
                node.get_logger().info(f"Updated control DT to {cls.DT}")

            # Update max speed if provided
            if node.has_parameter("max_speed"):
                cls.MAX_SPEED = (
                    node.get_parameter("max_speed").get_parameter_value().double_value
                )
                cls.MAX_LINEAR_VEL = cls.MAX_SPEED
                node.get_logger().info(f"Updated MAX_SPEED to {cls.MAX_SPEED}")

            # Update max steering if provided
            if node.has_parameter("max_steer_deg"):
                max_steer_deg = (
                    node.get_parameter("max_steer_deg")
                    .get_parameter_value()
                    .double_value
                )
                cls.MAX_STEER = math.radians(max_steer_deg)
                node.get_logger().info(f"Updated MAX_STEER to {max_steer_deg} degrees")

            # Add tracking weight parameters
            if node.has_parameter("position_weight"):
                pos_weight = (
                    node.get_parameter("position_weight")
                    .get_parameter_value()
                    .double_value
                )
                cls.Q[0, 0] = pos_weight
                cls.Q[1, 1] = pos_weight
                node.get_logger().info(f"Updated position weight to {pos_weight}")

            if node.has_parameter("yaw_weight"):
                yaw_weight = (
                    node.get_parameter("yaw_weight").get_parameter_value().double_value
                )
                cls.Q[3, 3] = yaw_weight
                node.get_logger().info(f"Updated yaw weight to {yaw_weight}")

        except Exception as e:
            node.get_logger().warn(f"Error updating parameters from ROS: {e}")


# ============================================================================


class MPCNode(Node):
    def __init__(self):
        # Use a unique node name to avoid conflicts
        super().__init__("mpc_path_tracker")

        # Declare ROS parameters with defaults from config
        self._declare_parameter_if_not_exists("mode", "car")
        self._declare_parameter_if_not_exists("target_speed", MPCConfig.TARGET_SPEED)
        self._declare_parameter_if_not_exists("horizon_length", MPCConfig.T)
        self._declare_parameter_if_not_exists("control_dt", MPCConfig.DT)
        self._declare_parameter_if_not_exists("max_speed", MPCConfig.MAX_SPEED)
        self._declare_parameter_if_not_exists(
            "max_steer_deg", math.degrees(MPCConfig.MAX_STEER)
        )
        # Add new tuning parameters
        self._declare_parameter_if_not_exists("position_weight", 10.0)
        self._declare_parameter_if_not_exists("yaw_weight", 5.0)

        # Update config from ROS parameters
        MPCConfig.update_from_ros_params(self)

        # Get parameter values with fallback defaults
        try:
            self.mode = self.get_parameter("mode").get_parameter_value().string_value
        except:
            self.mode = "car"
            self.get_logger().warn("Using default mode: car")

        try:
            self.target_speed = (
                self.get_parameter("target_speed").get_parameter_value().double_value
            )
        except:
            self.target_speed = MPCConfig.TARGET_SPEED
            self.get_logger().warn(
                f"Using default target_speed: {self.target_speed} m/s"
            )

        # Set up subscriber and timer
        self.odom_sub = self.create_subscription(
            Odometry, "/odometry/ground_truth", self.odom_callback, 10
        )
        self.create_timer(MPCConfig.DT, self.timer_callback)  # MPC control loop
        self.robot_odom = None

        # Set up publishers
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.path_pub = self.create_publisher(Path, "/path", 10)

        # NEW: Add publisher for current reference point for tracking analysis
        self.reference_pub = self.create_publisher(
            PoseStamped, "/mpc/reference_pose", 10
        )

        # Load the path from YAML
        self.path = self.read_path()
        self.path_index = 0
        self.get_logger().info(
            f"Loaded path with {self.path.shape[0]} points."
            if self.path is not None
            else "Path not loaded!"
        )

        # Publish the full path
        self.publish_path()

        # Vehicle parameters (from config)
        self.wheel_radius = MPCConfig.WHEEL_RADIUS
        self.l = MPCConfig.WHEEL_BASE
        self.track = MPCConfig.TRACK_WIDTH

        # MPC state variables
        self.current_state = State()
        self.target_ind = 0
        self.prev_linear_vel = 0.0
        self.prev_angular_vel = 0.0
        self.linear_x = 0.0

        # Error handling and fallback variables
        self.mpc_failed_count = 0
        self.max_mpc_failures = MPCConfig.MAX_MPC_FAILURES
        self.emergency_stop = False

        # NEW: Variables for reference tracking
        self.current_reference = None

        # Initialize MPC library with our configuration
        if self.path is not None:
            self.init_path()
            self.init_mpc()
        else:
            self.get_logger().error("Failed to load path - controller inactive")

    def _declare_parameter_if_not_exists(self, name, default_value):
        """Helper method to declare parameter only if it doesn't exist"""
        try:
            self.declare_parameter(name, default_value)
        except ParameterAlreadyDeclaredException:
            self.get_logger().debug(
                f"Parameter '{name}' already declared, using existing value"
            )

    def read_path(self):
        """Load path from YAML file"""
        try:
            pkg = get_package_share_directory("limo_controller")
            yaml_path = os.path.join(pkg, "path", "path.yaml")

            with open(yaml_path, "r") as file:
                data = yaml.safe_load(file)

            path_points = []
            for point in data:
                x = point.get("x")
                y = point.get("y")
                yaw = point.get("yaw")
                if x is None or y is None or yaw is None:
                    self.get_logger().warn(
                        "Point missing required key: 'x', 'y', or 'yaw'."
                    )
                    continue
                path_points.append([x, y, yaw])

            if len(path_points) < 2:
                self.get_logger().error("Path must have at least 2 points")
                return None

            return np.array(path_points)

        except FileNotFoundError:
            self.get_logger().error(f"Path file not found: {yaml_path}")
            return None
        except Exception as e:
            self.get_logger().error(f"Error reading path file: {e}")
            return None

    def publish_path(self):
        """Publish path for visualization"""
        if self.path is None:
            return

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "world"

        for point in self.path:
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.header.frame_id = "world"
            pose_stamped.pose.position.x = point[0]
            pose_stamped.pose.position.y = point[1]
            pose_stamped.pose.position.z = 0.0
            q = quaternion_from_euler(0, 0, point[2])
            pose_stamped.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            path_msg.poses.append(pose_stamped)

        self.path_pub.publish(path_msg)

    def init_path(self):
        # Path variables
        self.cx = self.path[:, 0]
        self.cy = self.path[:, 1]
        self.cyaw = self.path[:, 2]
        self.ck = [0.0] * len(self.cx)

        # Pass config to library functions
        self.sp = calc_speed_profile(
            self.cx, self.cy, self.cyaw, MPCConfig.TARGET_SPEED, MPCConfig
        )
        self.dl = calculate_path_distance(self.cx, self.cy)
        self.cyaw = smooth_yaw(self.cyaw, MPCConfig)

        self.get_logger().info("MPC Controller initialized successfully")
        self.get_logger().info(
            f"IMPROVED Vehicle params: wheelbase={MPCConfig.WB:.3f}m, track={MPCConfig.TRACK_WIDTH:.3f}m"
        )
        self.get_logger().info(
            f"IMPROVED Target speed: {self.target_speed:.2f}m/s, Horizon: {MPCConfig.T}, DT: {MPCConfig.DT:.3f}s"
        )
        self.get_logger().info(
            f"IMPROVED Weights - Position: {MPCConfig.Q[0,0]}, Yaw: {MPCConfig.Q[3,3]}, Input: {MPCConfig.R[0,0]}"
        )

    def init_mpc(self):
        initial_state = State(x=self.cx[0], y=self.cy[0], yaw=self.cyaw[0], v=0.0)
        self.goal = [self.cx[-1], self.cy[-1]]
        self.state = initial_state

        # initial yaw compensation
        if self.state.yaw - self.cyaw[0] >= math.pi:
            self.state.yaw -= math.pi * 2.0
        elif self.state.yaw - self.cyaw[0] <= -math.pi:
            self.state.yaw += math.pi * 2.0

        self.target_ind, _ = calc_nearest_index(
            self.state, self.cx, self.cy, self.cyaw, 0, MPCConfig
        )

        self.odelta, self.oa = None, None

    def mpc_control(self):
        """IMPROVED MPC control function with better error handling"""
        if self.path is None:
            self.get_logger().warn("No path available for MPC control")
            return

        if not hasattr(self.robot_odom, "pose") or self.robot_odom.pose is None:
            self.get_logger().debug("Robot odometry not available yet")
            return

        if self.emergency_stop:
            self.pub_emergency_stop()
            return

        try:
            self.current_state = self.get_state(self.robot_odom)
        except Exception as e:
            self.get_logger().error(f"Error reading odometry: {e}")
            return

        try:
            xref, self.target_ind, dref = calc_ref_trajectory(
                self.state,
                self.cx,
                self.cy,
                self.cyaw,
                self.ck,
                self.sp,
                self.dl,
                self.target_ind,
                MPCConfig,
            )

            # Store current reference for tracking analysis
            self.current_reference = {
                "x": float(xref[0, 0]),
                "y": float(xref[1, 0]),
                "v": float(xref[2, 0]),
                "yaw": float(xref[3, 0]),
                "target_ind": self.target_ind,
            }

            # Publish reference pose for visualization
            self.publish_reference_pose()

            x0 = [
                self.state.x,
                self.state.y,
                self.state.v,
                self.state.yaw,
            ]

            self.oa, self.odelta, ox, oy, oyaw, ov = iterative_linear_mpc_control(
                xref, x0, dref, self.oa, self.odelta, MPCConfig
            )

            if self.oa is None or self.odelta is None:
                self.mpc_failed_count += 1
                self.get_logger().warn(
                    f"MPC control failed ({self.mpc_failed_count}/{self.max_mpc_failures})"
                )

                if self.mpc_failed_count >= self.max_mpc_failures:
                    self.get_logger().error(
                        "Too many MPC failures, entering emergency stop mode"
                    )
                    self.emergency_stop = True
                    self.pub_emergency_stop()
                    return

                di, ai = 0.0, -0.1
            else:
                self.mpc_failed_count = 0
                di, ai = self.odelta[0], self.oa[0]

            self.pub_cmd_vel(di, ai)
            self.state = self.get_state(self.robot_odom)

        except Exception as e:
            self.get_logger().error(f"Error in MPC control: {e}")
            self.pub_emergency_stop()

    def publish_reference_pose(self):
        """Publish current reference pose for tracking analysis"""
        if self.current_reference is None:
            return

        ref_pose = PoseStamped()
        ref_pose.header.stamp = self.get_clock().now().to_msg()
        ref_pose.header.frame_id = "world"
        ref_pose.pose.position.x = self.current_reference["x"]
        ref_pose.pose.position.y = self.current_reference["y"]
        ref_pose.pose.position.z = 0.0

        q = quaternion_from_euler(0, 0, self.current_reference["yaw"])
        ref_pose.pose.orientation.x = q[0]
        ref_pose.pose.orientation.y = q[1]
        ref_pose.pose.orientation.z = q[2]
        ref_pose.pose.orientation.w = q[3]

        self.reference_pub.publish(ref_pose)

    def timer_callback(self):
        """Timer callback for MPC control loop"""
        if (
            self.robot_odom is None
            or not hasattr(self.robot_odom, "pose")
            or self.robot_odom.pose is None
            or self.robot_odom.pose.pose is None
        ):
            self.get_logger().debug("Waiting for odometry data...")
            return

        self.publish_path()
        self.mpc_control()

    def odom_callback(self, msg: Odometry):
        """Odometry callback"""
        self.robot_odom = msg

    def get_state(self, robot_odom):
        """Extract state from odometry message"""
        x = robot_odom.pose.pose.position.x
        y = robot_odom.pose.pose.position.y
        orientation_q = robot_odom.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        )
        v = robot_odom.twist.twist.linear.x

        return State(x=x, y=y, yaw=yaw, v=v)

    def pub_cmd_vel(self, di, ai):
        """Publish command velocity with IMPROVED safety limits and smoother control"""
        # More gradual velocity changes
        target_linear_x = self.linear_x + ai * MPCConfig.DT

        # Smoother velocity transitions
        max_vel_change = 0.05  # Limit velocity change per timestep
        vel_change = target_linear_x - self.linear_x
        if abs(vel_change) > max_vel_change:
            vel_change = max_vel_change if vel_change > 0 else -max_vel_change

        self.linear_x = self.linear_x + vel_change

        # Apply safety limits
        self.linear_x = max(
            MPCConfig.MIN_LINEAR_VEL, min(MPCConfig.MAX_LINEAR_VEL, self.linear_x)
        )

        msg = Twist()
        msg.linear.x = self.linear_x

        # Calculate angular velocity with improved limits
        angular_z = self.linear_x * math.tan(di) / self.l
        angular_z = max(
            MPCConfig.MIN_ANGULAR_VEL, min(MPCConfig.MAX_ANGULAR_VEL, angular_z)
        )
        msg.angular.z = angular_z

        self.cmd_vel_pub.publish(msg)

    def pub_emergency_stop(self):
        """Publish emergency stop command"""
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)
        self.linear_x = 0.0


def main(args=None):
    try:
        rclpy.init(args=args)
        node = MPCNode()

        if node.path is None:
            node.get_logger().error(
                "Failed to initialize MPC controller - no valid path"
            )
            return

        node.get_logger().info("IMPROVED MPC controller started successfully")
        node.get_logger().info(
            f"IMPROVED Configuration: Horizon={MPCConfig.T}, DT={MPCConfig.DT}, Target_Speed={MPCConfig.TARGET_SPEED}"
        )
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("MPC controller stopped by user")
    except Exception as e:
        print(f"Error starting MPC controller: {e}")
        import traceback

        traceback.print_exc()
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except:
            pass


if __name__ == "__main__":
    main()
