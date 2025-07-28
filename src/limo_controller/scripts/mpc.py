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
from std_msgs.msg import Float64, Float64MultiArray, String
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import math
import numpy as np
import os
import sys
import yaml
import cvxpy
import json  # FIX: Add missing json import
from ament_index_python.packages import get_package_share_directory

# Add current directory and parent directories to Python path
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
package_dir = os.path.join(parent_dir, 'limo_controller')

sys.path.insert(0, current_dir)
sys.path.insert(0, parent_dir) 
sys.path.insert(0, package_dir)

# Try multiple import strategies
cubic_spline_planner = None
angle_mod = None

from limo_controller.PathPlanning.CubicSpline import cubic_spline_planner
from limo_controller.utils.angle import angle_mod
from limo_controller.mpc_lib import (
    State,
    calc_speed_profile,
    calculate_path_distance,
    smooth_yaw,
    calc_nearest_index,
    calc_ref_trajectory,
    iterative_linear_mpc_control,
    get_switch_back_course  # Import the switch_back path generator
)


# ============================================================================
# IMPROVED MPC CONFIGURATION WITH BETTER NUMERICAL STABILITY
# ============================================================================
class MPCConfig:
    """Configuration with improved numerical stability"""

    # State and Control Dimensions
    NX = 4  # x = x, y, v, yaw
    NU = 2  # a = [accel, steer]
    T = 10  # Increased horizon for better lookahead

    # MPC Cost Matrices - More aggressive tracking weights
    R = np.diag([0.1, 1.0])  # Higher control costs to smooth commands
    Rd = np.diag([0.1, 2.0])  # Higher rate penalties for smoothness
    Q = np.diag(
        [10.0, 10.0, 1.0, 15.0]  # MUCH higher position and yaw tracking weights
    )
    Qf = Q * 2.0  # Even higher terminal cost for better convergence

    # Goal and Stopping Parameters
    GOAL_DIS = 1.5
    STOP_SPEED = 0.5 / 3.6
    MAX_TIME = 500.0

    # Iterative Parameters
    MAX_ITER = 3
    DU_TH = 0.1

    # Speed Parameters - More conservative for better tracking
    TARGET_SPEED = 0.5  # Reduced for better path following
    N_IND_SEARCH = 30  # Increased search range

    # Time Step
    DT = 0.05 # Reduced for better stability

    # Vehicle Physical Parameters
    LENGTH = 0.13
    WIDTH = 0.12
    BACKTOWHEEL = 0.065
    WHEEL_LEN = 0.045
    WHEEL_WIDTH = 0.045
    TREAD = 0.14
    WB = 0.2

    # Robot-specific Parameters
    WHEEL_RADIUS = 0.045
    WHEEL_BASE = 0.2
    TRACK_WIDTH = 0.14

    # Control Constraints - FIXED to allow reverse speeds that match path requirements
    MAX_STEER = np.deg2rad(10.0)  # Matches inverse_kinematics.py constraint
    MAX_DSTEER = np.deg2rad(30.0)  # Reasonable steering rate limit
    MAX_SPEED = 1.5  # Conservative speed limit for small robot
    MIN_SPEED = -1.5  # FIXED: Allow full reverse speed to match path requirements
    MAX_ACCEL = 1.5  # Conservative acceleration limit

    # For cmd_vel output limits - matched to robot capabilities
    MAX_LINEAR_VEL = 1.0  # Conservative for small robot
    MIN_LINEAR_VEL = -1.0  # FIXED: Allow reverse to match path
    MAX_ANGULAR_VEL = 0.8  # Reasonable turning rate
    MIN_ANGULAR_VEL = -0.8

    # Safety Parameters
    MAX_MPC_FAILURES = 5

    @classmethod
    def get_dict(cls):
        """Return all parameters as a dictionary"""
        return {
            attr: getattr(cls, attr)
            for attr in dir(cls)
            if not attr.startswith("_") and not callable(getattr(cls, attr))
        }

    @classmethod
    def update_from_ros_params(cls, node):
        """Update parameters from ROS parameter server"""
        try:
            if node.has_parameter("target_speed"):
                cls.TARGET_SPEED = (
                    node.get_parameter("target_speed")
                    .get_parameter_value()
                    .double_value
                )
                node.get_logger().info(f"Updated TARGET_SPEED to {cls.TARGET_SPEED}")

            if node.has_parameter("horizon_length"):
                cls.T = (
                    node.get_parameter("horizon_length")
                    .get_parameter_value()
                    .integer_value
                )
                node.get_logger().info(f"Updated horizon length T to {cls.T}")

            if node.has_parameter("control_dt"):
                cls.DT = (
                    node.get_parameter("control_dt").get_parameter_value().double_value
                )
                node.get_logger().info(f"Updated control DT to {cls.DT}")

            if node.has_parameter("max_speed"):
                cls.MAX_SPEED = (
                    node.get_parameter("max_speed").get_parameter_value().double_value
                )
                node.get_logger().info(f"Updated MAX_SPEED to {cls.MAX_SPEED}")

            if node.has_parameter("max_steer_deg"):
                max_steer_deg = (
                    node.get_parameter("max_steer_deg")
                    .get_parameter_value()
                    .double_value
                )
                cls.MAX_STEER = math.radians(max_steer_deg)
                node.get_logger().info(f"Updated MAX_STEER to {max_steer_deg} degrees")

            # NEW: Update Q matrix weights
            if node.has_parameter("position_weight"):
                pos_weight = node.get_parameter("position_weight").get_parameter_value().double_value
                cls.Q[0, 0] = pos_weight  # x position weight
                cls.Q[1, 1] = pos_weight  # y position weight
                cls.Qf = cls.Q * 2.0  # Update terminal cost
                node.get_logger().info(f"Updated position weight to {pos_weight}")

            if node.has_parameter("yaw_weight"):
                yaw_weight = node.get_parameter("yaw_weight").get_parameter_value().double_value
                cls.Q[3, 3] = yaw_weight  # yaw weight
                cls.Qf = cls.Q * 2.0  # Update terminal cost
                node.get_logger().info(f"Updated yaw weight to {yaw_weight}")

            if node.has_parameter("control_weight"):
                ctrl_weight = node.get_parameter("control_weight").get_parameter_value().double_value
                cls.R[0, 0] = ctrl_weight  # acceleration weight
                cls.R[1, 1] = ctrl_weight * 10  # steering weight (higher)
                node.get_logger().info(f"Updated control weight to {ctrl_weight}")

        except Exception as e:
            node.get_logger().warn(f"Error updating parameters from ROS: {e}")


class MPCNode(Node):
    def __init__(self):
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
        
        # NEW: Parameter sweep related parameters
        self._declare_parameter_if_not_exists("position_weight", 10.0)
        self._declare_parameter_if_not_exists("yaw_weight", 15.0)
        self._declare_parameter_if_not_exists("control_weight", 0.1)
        
        # Path selection parameters - SIMPLIFIED to just path_type and yaml_path_file
        self._declare_parameter_if_not_exists("path_type", "yaml")  # "yaml" or "switch_back"
        self._declare_parameter_if_not_exists("yaml_path_file", "path.yaml")  # Custom YAML file

        # Update config from ROS parameters
        MPCConfig.update_from_ros_params(self)

        # Get parameter values
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

        # Get path configuration parameters
        try:
            self.path_type = self.get_parameter("path_type").get_parameter_value().string_value
        except:
            self.path_type = "yaml"

        try:
            self.yaml_path_file = self.get_parameter("yaml_path_file").get_parameter_value().string_value
        except:
            self.yaml_path_file = "path.yaml"

        # Set up subscribers and timer
        self.odom_sub = self.create_subscription(
            Odometry, "/odometry/ground_truth", self.odom_callback, 10
        )
        
        # NEW: Subscribe to parameter update commands
        self.param_update_sub = self.create_subscription(
            String, "/mpc/parameter_update", self.parameter_update_callback, 10
        )
        
        self.create_timer(MPCConfig.DT, self.timer_callback)
        self.robot_odom = None

        # Set up publishers
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.path_pub = self.create_publisher(Path, "/path", 10)
        self.reference_pub = self.create_publisher(
            PoseStamped, "/mpc/reference_pose", 10
        )
        
        # NEW: Publisher for experiment status
        self.status_pub = self.create_publisher(String, "/mpc/experiment_status", 10)

        # Initialize path based on configuration
        self.path_loaded = self.initialize_path()
        
        if not self.path_loaded:
            self.get_logger().error("Failed to load any path - controller inactive")
            return
        
        # Publish the full path
        self.publish_path()

        # Vehicle parameters
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

        # Variables for reference tracking
        self.current_reference = None

        # Control loop counter for the specific debug message you want to keep
        self.control_loop_count = 0
        self.stuck_counter = 0  # Track if robot gets stuck
        self.last_target_ind = 0

        # NEW: Goal tracking for parameter sweep
        self.goal_reached = False
        self.experiment_start_time = None

        # Initialize MPC
        if self.path_loaded:
            self.init_mpc()
            path_source = "YAML file" if self.path_type == "yaml" else f"built-in generator ({self.path_type})"
            self.get_logger().info(f"MPC Controller initialized with path from: {path_source}")
        else:
            self.get_logger().error("Failed to initialize MPC - no valid path")

    def _declare_parameter_if_not_exists(self, name, default_value):
        """Helper method to declare parameter only if it doesn't exist"""
        try:
            self.declare_parameter(name, default_value)
        except ParameterAlreadyDeclaredException:
            pass

    def initialize_path(self):
        """Initialize path based on configuration parameters"""
        if self.path_type == "yaml":
            return self.load_path_from_yaml()
        elif self.path_type == "switch_back":
            return self.generate_switch_back_path()
        else:
            self.get_logger().error(f"Unsupported path type: {self.path_type}. Supported types: 'yaml', 'switch_back'")
            return False

    def load_path_from_yaml(self):
        """Load path from YAML file"""
        try:
            pkg = get_package_share_directory("limo_controller")
            yaml_path = os.path.join(pkg, "path", self.yaml_path_file)

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
                return False

            path_array = np.array(path_points)
            self.cx = path_array[:, 0]
            self.cy = path_array[:, 1]
            self.cyaw = path_array[:, 2]
            self.ck = [0.0] * len(self.cx)

            # Initialize path parameters
            self.sp = calc_speed_profile(
                self.cx, self.cy, self.cyaw, MPCConfig.TARGET_SPEED, MPCConfig
            )
            self.dl = calculate_path_distance(self.cx, self.cy)
            self.cyaw = smooth_yaw(self.cyaw, MPCConfig)

            self.get_logger().info(f"Loaded path from YAML file: {yaml_path} with {len(self.cx)} points")
            return True

        except FileNotFoundError:
            self.get_logger().error(f"Path file not found: {yaml_path}")
            return False
        except Exception as e:
            self.get_logger().error(f"Error reading path file: {e}")
            return False

    def generate_switch_back_path(self):
        """Generate switch back path using built-in generator"""
        try:
            # Calculate path distance for interpolation
            dl = 0.1  # Path resolution
            
            # Generate switch_back path using the function from mpc_lib
            self.cx, self.cy, self.cyaw, self.ck = get_switch_back_course(dl)
            self.get_logger().info(f"Generated switch_back path with {len(self.cx)} points")
            
            # Initialize path parameters
            self.sp = calc_speed_profile(
                self.cx, self.cy, self.cyaw, MPCConfig.TARGET_SPEED, MPCConfig
            )
            self.dl = calculate_path_distance(self.cx, self.cy)
            self.cyaw = smooth_yaw(self.cyaw, MPCConfig)
            
            return True
                
        except Exception as e:
            self.get_logger().error(f"Error generating switch_back path: {e}")
            return False

    def parameter_update_callback(self, msg):
        """Handle parameter update requests from parameter sweep"""
        try:
            update_data = json.loads(msg.data)
            
            if update_data.get('action') == 'update_parameters':
                params = update_data.get('parameters', {})
                
                # Update ROS parameters
                param_updates = []
                for param_name, param_value in params.items():
                    param_updates.append(rclpy.parameter.Parameter(param_name, value=param_value))
                
                # Set parameters
                self.set_parameters(param_updates)
                
                # Update config
                MPCConfig.update_from_ros_params(self)
                
                # Handle path type changes
                if 'path_type' in params:
                    old_path_type = self.path_type
                    self.path_type = params['path_type']
                    
                    # Regenerate path if needed
                    if self.path_type != old_path_type:
                        if self.initialize_path():
                            self.publish_path()
                            self.init_mpc()
                            self.get_logger().info(f"Path changed from {old_path_type} to {self.path_type}")
                        else:
                            self.get_logger().error(f"Failed to load new path type: {self.path_type}")
                            return
                
                # Handle YAML path file changes
                if 'yaml_path_file' in params:
                    self.yaml_path_file = params['yaml_path_file']
                    if self.path_type == "yaml":
                        if self.initialize_path():
                            self.publish_path()
                            self.init_mpc()
                        else:
                            self.get_logger().error(f"Failed to load YAML path: {self.yaml_path_file}")
                            return
                
                # Reset experiment state
                self.goal_reached = False
                self.experiment_start_time = self.get_clock().now()
                self.emergency_stop = False
                self.mpc_failed_count = 0
                
                # Publish status
                status_msg = String()
                status_msg.data = json.dumps({
                    'status': 'parameters_updated',
                    'parameters': params
                })
                self.status_pub.publish(status_msg)
                
                self.get_logger().info(f"Parameters updated: {params}")
                
        except Exception as e:
            self.get_logger().error(f"Error updating parameters: {e}")

    def publish_path(self):
        """Publish path for visualization"""
        if not hasattr(self, 'cx') or self.cx is None:
            return

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "world"

        for i in range(len(self.cx)):
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.header.frame_id = "world"
            pose_stamped.pose.position.x = self.cx[i]
            pose_stamped.pose.position.y = self.cy[i]
            pose_stamped.pose.position.z = 0.0
            q = quaternion_from_euler(0, 0, self.cyaw[i])
            pose_stamped.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            path_msg.poses.append(pose_stamped)

        self.path_pub.publish(path_msg)

    def init_mpc(self):
        """Initialize MPC state"""
        initial_state = State(x=self.cx[0], y=self.cy[0], yaw=self.cyaw[0], v=0.0)
        self.goal = [self.cx[-1], self.cy[-1]]
        self.state = initial_state

        # Initial yaw compensation
        if self.state.yaw - self.cyaw[0] >= math.pi:
            self.state.yaw -= math.pi * 2.0
        elif self.state.yaw - self.cyaw[0] <= -math.pi:
            self.state.yaw += math.pi * 2.0

        self.target_ind, _ = calc_nearest_index(
            self.state, self.cx, self.cy, self.cyaw, 0, MPCConfig
        )
        self.odelta, self.oa = None, None

    def check_goal(self, state, goal, tind, nind):
        """Check if goal is reached"""
        dx = state.x - goal[0]
        dy = state.y - goal[1]
        d = math.hypot(dx, dy)

        isgoal = d <= MPCConfig.GOAL_DIS

        if abs(tind - nind) >= 5:
            isgoal = False

        isstop = abs(state.v) <= MPCConfig.STOP_SPEED

        if isgoal and isstop:
            return True

        return False

    def mpc_control(self):
        """MPC control function with improved error handling"""
        if not hasattr(self, 'cx') or self.cx is None:
            self.get_logger().warn("No path available for MPC control")
            return

        if not hasattr(self.robot_odom, "pose") or self.robot_odom.pose is None:
            return

        if self.emergency_stop:
            self.pub_emergency_stop()
            return

        try:
            self.current_state = self.get_state(self.robot_odom)
            # IMPORTANT: Update the MPC state with current robot state
            self.state = self.current_state
        except Exception as e:
            self.get_logger().error(f"Error reading odometry: {e}")
            return

        # Log current state every 10 iterations (ONLY debug message you wanted to keep)
        self.control_loop_count += 1
        if self.control_loop_count % 10 == 1:
            self.get_logger().info(
                f"Loop {self.control_loop_count}: Robot state: x={self.current_state.x:.3f}, y={self.current_state.y:.3f}, yaw={self.current_state.yaw:.3f}, v={self.current_state.v:.2f}"
            )

        # Check if goal reached
        if self.check_goal(self.state, self.goal, self.target_ind, len(self.cx)):
            if not self.goal_reached:
                self.goal_reached = True
                self.get_logger().info("Goal reached!")
                
                # Publish goal reached status
                status_msg = String()
                status_msg.data = json.dumps({'status': 'goal_reached'})
                self.status_pub.publish(status_msg)
                
            self.pub_emergency_stop()
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

            # Check if robot is stuck at same target index
            if self.target_ind == self.last_target_ind:
                self.stuck_counter += 1
                if self.stuck_counter > 20:  # Stuck for 2 seconds
                    self.target_ind = min(self.target_ind + 2, len(self.cx) - 1)
                    self.stuck_counter = 0
            else:
                self.stuck_counter = 0
                self.last_target_ind = self.target_ind

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
                    
                    # Publish failure status
                    status_msg = String()
                    status_msg.data = json.dumps({'status': 'experiment_failed'})
                    self.status_pub.publish(status_msg)
                    return

                # Use conservative fallback control
                di, ai = 0.0, 0.0
            else:
                self.mpc_failed_count = 0
                di, ai = self.odelta[0], self.oa[0]

            self.pub_cmd_vel(di, ai)

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
        """Publish command velocity with safety limits"""
        # Update linear velocity
        self.linear_x = self.linear_x + ai * MPCConfig.DT

        # Apply safety limits
        self.linear_x = max(
            MPCConfig.MIN_LINEAR_VEL, min(MPCConfig.MAX_LINEAR_VEL, self.linear_x)
        )

        msg = Twist()
        msg.linear.x = self.linear_x

        # Calculate angular velocity
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

        if not node.path_loaded:
            node.get_logger().error(
                "Failed to initialize MPC controller - no valid path"
            )
            return

        node.get_logger().info("MPC controller started successfully")
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