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

# ROBUST IMPORT HANDLING - Multiple fallback strategies
import sys

# Add current directory to Python path for imports
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, current_dir)

# Try multiple import strategies
cubic_spline_planner = None
angle_mod = None

# Strategy 1: Try normal package imports
try:
    from PathPlanning.CubicSpline import cubic_spline_planner
    from utils.angle import angle_mod

    print("✓ Successfully imported via package structure")
except ImportError as e1:
    print(f"Package import failed: {e1}")

    # Strategy 2: Try direct imports
    try:
        sys.path.insert(0, os.path.join(current_dir, "PathPlanning", "CubicSpline"))
        sys.path.insert(0, os.path.join(current_dir, "utils"))
        import cubic_spline_planner
        from angle import angle_mod

        print("✓ Successfully imported via direct imports")
    except ImportError as e2:
        print(f"Direct import failed: {e2}")

        # Strategy 3: Define minimal implementations
        print("⚠ Using fallback implementations")

        # Minimal angle_mod implementation
        def angle_mod(angle):
            import math

            while angle > math.pi:
                angle -= 2.0 * math.pi
            while angle < -math.pi:
                angle += 2.0 * math.pi
            return angle

        # Minimal cubic spline implementation
        class MockCubicSplinePlanner:
            @staticmethod
            def calc_spline_course(ax, ay, ds=0.1):
                # Simple linear interpolation fallback
                import numpy as np

                if len(ax) < 2:
                    return ax, ay, [0.0] * len(ax), [0.0] * len(ax), [0.0] * len(ax)

                rx, ry, ryaw, rk, s = [], [], [], [], []
                total_length = 0

                for i in range(len(ax) - 1):
                    dx = ax[i + 1] - ax[i]
                    dy = ay[i + 1] - ay[i]
                    dist = math.sqrt(dx * dx + dy * dy)
                    yaw = math.atan2(dy, dx)

                    # Interpolate points
                    num_points = max(1, int(dist / ds))
                    for j in range(num_points):
                        ratio = j / num_points if num_points > 0 else 0
                        x_interp = ax[i] + ratio * dx
                        y_interp = ay[i] + ratio * dy
                        rx.append(x_interp)
                        ry.append(y_interp)
                        ryaw.append(yaw)
                        rk.append(0.0)  # No curvature calculation
                        s.append(total_length + ratio * dist)

                    total_length += dist

                # Add final point
                rx.append(ax[-1])
                ry.append(ay[-1])
                ryaw.append(ryaw[-1] if ryaw else 0.0)
                rk.append(0.0)
                s.append(total_length)

                return rx, ry, ryaw, rk, s

        cubic_spline_planner = MockCubicSplinePlanner()

# Now try to import mpc_lib with the same robust approach
try:
    from mpc_lib import *

    print("✓ Successfully imported mpc_lib")
except ImportError as e:
    print(f"⚠ mpc_lib import failed: {e}")
    # We'll need to define minimal MPC library functions here
    # This is a more complex fallback, but for now let's try to continue


# ============================================================================
# SYNCED PARAMETER CONFIGURATION WITH ROBOT DESCRIPTION AND WORKING EXAMPLE
# ============================================================================
class MPCConfig:
    """Configuration synced with robot URDF and working example"""

    # State and Control Dimensions
    NX = 4  # x = x, y, v, yaw
    NU = 2  # a = [accel, steer]
    T = 5  # horizon length (same as working example)

    # MPC Cost Matrices - Balanced between tracking and smoothness
    R = np.diag([0.01, 0.01])  # Low control cost (from working example)
    Rd = np.diag([0.01, 1.0])  # Control rate change cost (from working example)
    Q = np.diag(
        [1.0, 1.0, 0.5, 0.5]
    )  # Moderate tracking weights (from working example)
    Qf = Q  # Terminal cost same as stage cost

    # Goal and Stopping Parameters
    GOAL_DIS = 1.5  # Same as working example
    STOP_SPEED = 0.5 / 3.6  # Same as working example (0.139 m/s)
    MAX_TIME = 500.0

    # Iterative Parameters
    MAX_ITER = 3  # Same as working example
    DU_TH = 0.1  # Same as working example

    # Speed Parameters - Adjusted for small robot
    TARGET_SPEED = 1.5  # 2.78 m/s from working example
    N_IND_SEARCH = 10  # Same as working example

    # Time Step - Balance between computation and lookahead
    DT = 0.2  # Same as working example for good lookahead

    # Vehicle Physical Parameters FROM ROBOT DESCRIPTION
    LENGTH = 0.13  # base_x_size from URDF
    WIDTH = 0.12  # base_y_size from URDF
    BACKTOWHEEL = 0.065  # Half of base length
    WHEEL_LEN = 0.045  # From URDF
    WHEEL_WIDTH = 0.045  # From URDF (wheel_length in URDF)
    TREAD = 0.14  # track width from URDF
    WB = 0.2  # wheelbase from URDF - KEEP THIS!

    # Robot-specific Parameters FROM URDF
    WHEEL_RADIUS = 0.045  # From URDF
    WHEEL_BASE = 0.2  # From URDF
    TRACK_WIDTH = 0.14  # From URDF

    # Control Constraints FROM ROBOT URDF AND WORKING EXAMPLE
    # URDF steering limit: ±0.523598767 radians = ±30 degrees
    MAX_STEER = 0.523598767  # 30 degrees from URDF steering joint limits
    MAX_DSTEER = np.deg2rad(30.0)  # From working example
    MAX_SPEED = 1.0  # From working example (15.28 m/s)
    MIN_SPEED = -0.3  # From working example (-5.56 m/s)
    MAX_ACCEL = 1.0  # From working example

    # For cmd_vel output limits
    MAX_LINEAR_VEL = 5.0  # Reasonable for small robot
    MIN_LINEAR_VEL = -2.0  # Limited reverse
    MAX_ANGULAR_VEL = 5.0  # Reasonable turning rate
    MIN_ANGULAR_VEL = -5.0

    # Safety Parameters
    MAX_MPC_FAILURES = 5

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

        except Exception as e:
            node.get_logger().warn(f"Error updating parameters from ROS: {e}")


# Minimal MPC library functions if imports failed
if "State" not in globals():

    class State:
        def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
            self.x = x
            self.y = y
            self.yaw = yaw
            self.v = v
            self.predelta = None


if "calc_speed_profile" not in globals():

    def calc_speed_profile(cx, cy, cyaw, target_speed, config):
        """Calculate speed profile with direction changes for sharp turns"""
        speed_profile = [target_speed] * len(cx)
        direction = 1.0  # forward

        # Set stop point
        for i in range(len(cx) - 1):
            dx = cx[i + 1] - cx[i]
            dy = cy[i + 1] - cy[i]

            move_direction = math.atan2(dy, dx)

            if dx != 0.0 and dy != 0.0:
                dangle = abs(angle_mod(move_direction - cyaw[i]))
                if dangle >= math.pi / 4.0:
                    direction = -1.0
                else:
                    direction = 1.0

            if direction != 1.0:
                speed_profile[i] = -target_speed
            else:
                speed_profile[i] = target_speed

        speed_profile[-1] = 0.0

        return speed_profile


if "calculate_path_distance" not in globals():

    def calculate_path_distance(cx, cy):
        if len(cx) < 2:
            return 1.0
        total_distance = 0.0
        for i in range(len(cx) - 1):
            dx = cx[i + 1] - cx[i]
            dy = cy[i + 1] - cy[i]
            total_distance += math.sqrt(dx * dx + dy * dy)
        return total_distance / (len(cx) - 1)


if "smooth_yaw" not in globals():

    def smooth_yaw(yaw, config):
        """Smooth yaw angles to prevent discontinuities"""
        for i in range(len(yaw) - 1):
            dyaw = yaw[i + 1] - yaw[i]

            while dyaw >= math.pi / 2.0:
                yaw[i + 1] -= math.pi * 2.0
                dyaw = yaw[i + 1] - yaw[i]

            while dyaw <= -math.pi / 2.0:
                yaw[i + 1] += math.pi * 2.0
                dyaw = yaw[i + 1] - yaw[i]

        return yaw


if "calc_nearest_index" not in globals():

    def calc_nearest_index(state, cx, cy, cyaw, pind, config):
        """Calculate nearest index with proper search range"""
        search_range = min(config.N_IND_SEARCH, len(cx) - pind)
        dx = [state.x - icx for icx in cx[pind : (pind + search_range)]]
        dy = [state.y - icy for icy in cy[pind : (pind + search_range)]]

        d = [idx**2 + idy**2 for (idx, idy) in zip(dx, dy)]

        mind = min(d)
        ind = d.index(mind) + pind

        mind = math.sqrt(mind)

        dxl = cx[ind] - state.x
        dyl = cy[ind] - state.y

        angle = angle_mod(cyaw[ind] - math.atan2(dyl, dxl))
        if angle < 0:
            mind *= -1

        return ind, mind


if "calc_ref_trajectory" not in globals():

    def calc_ref_trajectory(state, cx, cy, cyaw, ck, sp, dl, pind, config):
        """Calculate reference trajectory"""
        xref = np.zeros((config.NX, config.T + 1))
        dref = np.zeros((1, config.T + 1))
        ncourse = len(cx)

        ind, _ = calc_nearest_index(state, cx, cy, cyaw, pind, config)

        if pind >= ind:
            ind = pind

        xref[0, 0] = cx[ind]
        xref[1, 0] = cy[ind]
        xref[2, 0] = sp[ind]
        xref[3, 0] = cyaw[ind]
        dref[0, 0] = 0.0  # steer operational point should be 0

        travel = 0.0

        for i in range(config.T + 1):
            travel += abs(state.v) * config.DT
            dind = int(round(travel / dl))

            if (ind + dind) < ncourse:
                xref[0, i] = cx[ind + dind]
                xref[1, i] = cy[ind + dind]
                xref[2, i] = sp[ind + dind]
                xref[3, i] = cyaw[ind + dind]
                dref[0, i] = 0.0
            else:
                xref[0, i] = cx[ncourse - 1]
                xref[1, i] = cy[ncourse - 1]
                xref[2, i] = sp[ncourse - 1]
                xref[3, i] = cyaw[ncourse - 1]
                dref[0, i] = 0.0

        return xref, ind, dref


if "iterative_linear_mpc_control" not in globals():

    def iterative_linear_mpc_control(xref, x0, dref, oa, od, config):
        """MPC control with updating operational point iteratively"""
        ox, oy, oyaw, ov = None, None, None, None

        if oa is None or od is None:
            oa = [0.0] * config.T
            od = [0.0] * config.T

        for i in range(config.MAX_ITER):
            xbar = predict_motion(x0, oa, od, xref, config)
            poa, pod = oa[:], od[:]
            oa, od, ox, oy, oyaw, ov = linear_mpc_control(xref, xbar, x0, dref, config)

            if oa is None or od is None:
                print(f"MPC solver failed at iteration {i}")
                if i == 0:
                    oa = [0.0] * config.T
                    od = [0.0] * config.T
                    ox = [x0[0]] * (config.T + 1)
                    oy = [x0[1]] * (config.T + 1)
                    oyaw = [x0[3]] * (config.T + 1)
                    ov = [x0[2]] * (config.T + 1)
                else:
                    oa = poa
                    od = pod
                break

            # Convert to numpy arrays for arithmetic operations
            oa_array = np.array(oa)
            od_array = np.array(od)
            poa_array = np.array(poa)
            pod_array = np.array(pod)

            du = np.sum(np.abs(oa_array - poa_array)) + np.sum(
                np.abs(od_array - pod_array)
            )
            if du <= config.DU_TH:
                break
        else:
            print("Iterative is max iter")

        return oa, od, ox, oy, oyaw, ov


if "predict_motion" not in globals():

    def predict_motion(x0, oa, od, xref, config):
        """Predict motion"""
        xbar = xref * 0.0
        for i, _ in enumerate(x0):
            xbar[i, 0] = x0[i]

        state = State(x=x0[0], y=x0[1], yaw=x0[3], v=x0[2])
        for ai, di, i in zip(oa, od, range(1, config.T + 1)):
            state = update_state(state, ai, di, config)
            xbar[0, i] = state.x
            xbar[1, i] = state.y
            xbar[2, i] = state.v
            xbar[3, i] = state.yaw

        return xbar


if "update_state" not in globals():

    def update_state(state, a, delta, config):
        """Update state"""
        # input check
        if delta >= config.MAX_STEER:
            delta = config.MAX_STEER
        elif delta <= -config.MAX_STEER:
            delta = -config.MAX_STEER

        state.x = state.x + state.v * math.cos(state.yaw) * config.DT
        state.y = state.y + state.v * math.sin(state.yaw) * config.DT
        state.yaw = state.yaw + state.v / config.WB * math.tan(delta) * config.DT
        state.v = state.v + a * config.DT

        if state.v > config.MAX_SPEED:
            state.v = config.MAX_SPEED
        elif state.v < config.MIN_SPEED:
            state.v = config.MIN_SPEED

        return state


if "get_linear_model_matrix" not in globals():

    def get_linear_model_matrix(v, phi, delta, config):
        """Get linear model matrix"""
        A = np.zeros((config.NX, config.NX))
        A[0, 0] = 1.0
        A[1, 1] = 1.0
        A[2, 2] = 1.0
        A[3, 3] = 1.0
        A[0, 2] = config.DT * math.cos(phi)
        A[0, 3] = -config.DT * v * math.sin(phi)
        A[1, 2] = config.DT * math.sin(phi)
        A[1, 3] = config.DT * v * math.cos(phi)
        A[3, 2] = config.DT * math.tan(delta) / config.WB

        B = np.zeros((config.NX, config.NU))
        B[2, 0] = config.DT
        B[3, 1] = config.DT * v / (config.WB * math.cos(delta) ** 2)

        C = np.zeros(config.NX)
        C[0] = config.DT * v * math.sin(phi) * phi
        C[1] = -config.DT * v * math.cos(phi) * phi
        C[3] = -config.DT * v * delta / (config.WB * math.cos(delta) ** 2)

        return A, B, C


if "linear_mpc_control" not in globals():

    def linear_mpc_control(xref, xbar, x0, dref, config):
        """Linear MPC control"""
        try:
            x = cvxpy.Variable((config.NX, config.T + 1))
            u = cvxpy.Variable((config.NU, config.T))

            cost = 0.0
            constraints = []

            for t in range(config.T):
                cost += cvxpy.quad_form(u[:, t], config.R)

                if t != 0:
                    cost += cvxpy.quad_form(xref[:, t] - x[:, t], config.Q)

                A, B, C = get_linear_model_matrix(
                    xbar[2, t], xbar[3, t], dref[0, t], config
                )
                constraints += [x[:, t + 1] == A @ x[:, t] + B @ u[:, t] + C]

                if t < (config.T - 1):
                    cost += cvxpy.quad_form(u[:, t + 1] - u[:, t], config.Rd)
                    constraints += [
                        cvxpy.abs(u[1, t + 1] - u[1, t])
                        <= config.MAX_DSTEER * config.DT
                    ]

            cost += cvxpy.quad_form(xref[:, config.T] - x[:, config.T], config.Qf)

            constraints += [x[:, 0] == x0]
            constraints += [x[2, :] <= config.MAX_SPEED]
            constraints += [x[2, :] >= config.MIN_SPEED]
            constraints += [cvxpy.abs(u[0, :]) <= config.MAX_ACCEL]
            constraints += [cvxpy.abs(u[1, :]) <= config.MAX_STEER]

            prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)

            # Try multiple solvers
            solvers_to_try = [cvxpy.CLARABEL, cvxpy.OSQP, cvxpy.SCS, cvxpy.ECOS]

            solved = False
            for solver in solvers_to_try:
                try:
                    if solver == cvxpy.CLARABEL:
                        prob.solve(solver=solver, verbose=False, max_iter=1000)
                    elif solver == cvxpy.OSQP:
                        prob.solve(
                            solver=solver,
                            verbose=False,
                            max_iter=1000,
                            eps_abs=1e-4,
                            eps_rel=1e-4,
                        )
                    elif solver == cvxpy.SCS:
                        prob.solve(
                            solver=solver, verbose=False, max_iters=1000, eps=1e-4
                        )
                    else:
                        prob.solve(solver=solver, verbose=False)

                    if (
                        prob.status == cvxpy.OPTIMAL
                        or prob.status == cvxpy.OPTIMAL_INACCURATE
                    ):
                        solved = True
                        break
                except Exception:
                    continue

            if solved:
                ox = get_nparray_from_matrix(x.value[0, :])
                oy = get_nparray_from_matrix(x.value[1, :])
                ov = get_nparray_from_matrix(x.value[2, :])
                oyaw = get_nparray_from_matrix(x.value[3, :])
                oa = get_nparray_from_matrix(u.value[0, :])
                odelta = get_nparray_from_matrix(u.value[1, :])
            else:
                print("Error: Cannot solve mpc with any available solver")
                oa, odelta, ox, oy, oyaw, ov = None, None, None, None, None, None

        except Exception as e:
            print(f"MPC optimization error: {e}")
            oa, odelta, ox, oy, oyaw, ov = None, None, None, None, None, None

        return oa, odelta, ox, oy, oyaw, ov


if "get_nparray_from_matrix" not in globals():

    def get_nparray_from_matrix(x):
        return np.array(x).flatten()


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
            f"Robot params: wheelbase={MPCConfig.WB:.3f}m, track={MPCConfig.TRACK_WIDTH:.3f}m"
        )
        self.get_logger().info(
            f"Steering limits: ±{math.degrees(MPCConfig.MAX_STEER):.1f}° (from URDF)"
        )
        self.get_logger().info(
            f"Target speed: {self.target_speed:.2f}m/s, Horizon: {MPCConfig.T}, DT: {MPCConfig.DT:.3f}s"
        )
        self.get_logger().info(
            f"MPC Weights - Position: Q={MPCConfig.Q[0,0]}, Yaw: Q={MPCConfig.Q[3,3]}, Control: R={MPCConfig.R[0,0]}"
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

    def check_goal(self, state, goal, tind, nind):
        """Check if goal is reached"""
        # check goal
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
        """MPC control function with better error handling"""
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

        # Check if goal reached
        if self.check_goal(self.state, self.goal, self.target_ind, len(self.cx)):
            self.get_logger().info("Goal reached!")
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
        # v = robot_odom.twist.twist.linear.x
        if robot_odom.twist.twist.linear.x > 0:
            v = np.hypot(
                robot_odom.twist.twist.linear.x, robot_odom.twist.twist.linear.y
            )
        else:
            v = -np.hypot(
                robot_odom.twist.twist.linear.x, robot_odom.twist.twist.linear.y
            )

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

        if node.path is None:
            node.get_logger().error(
                "Failed to initialize MPC controller - no valid path"
            )
            return

        node.get_logger().info("MPC controller started successfully")
        node.get_logger().info(
            f"Configuration: Horizon={MPCConfig.T}, DT={MPCConfig.DT}, Target_Speed={MPCConfig.TARGET_SPEED}"
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
