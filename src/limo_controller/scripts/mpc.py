#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterAlreadyDeclaredException
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import (
    Point, Pose, PoseStamped, PoseWithCovariance, Quaternion, Twist, TwistWithCovariance,
    Vector3, TransformStamped
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

# MPC Parameters
NX = 4  # x = x, y, v, yaw
NU = 2  # a = [accel, steer]
T = 5  # horizon length

# MPC cost matrices
R = np.diag([1, 1])  # input cost matrix
Rd = np.diag([0.01, 1.0])  # input difference cost matrix
Q = np.diag([10, 10, 1, 1])  # state cost matrix
Qf = Q  # state final matrix

# MPC algorithm parameters (still needed for optimization)
MAX_ITER = 3  # Max iteration for iterative MPC
DU_TH = 0.1  # iteration finish convergence threshold
N_IND_SEARCH = 10  # Search index number for nearest point
DT = 0.01  # [s] time tick (faster for real-time)

# Vehicle parameters (corrected from FK node)
TRACK_WIDTH = 0.14  # [m] distance between left and right wheels
WHEEL_RADIUS = 0.045  # [m] wheel radius  
WHEEL_BASE = 0.2  # [m] distance between front and rear wheels
WB = WHEEL_BASE  # wheelbase for MPC calculations

# Vehicle constraints (adjusted for small robot)
MAX_STEER = np.deg2rad(10)  # maximum steering angle [rad] (reduced for small robot)
MAX_DSTEER = np.deg2rad(10)  # maximum steering speed [rad/s] (can be higher for small robot)
MAX_SPEED = 2.0  # maximum speed [m/s] (realistic for small robot)
MIN_SPEED = -1.0  # minimum speed [m/s] (reverse speed)
MAX_ACCEL = 20.0  # maximum accel [m/s²] (higher for small robot)

# Velocity constraints for differential drive
MAX_LINEAR_VEL = 2.0  # maximum linear velocity [m/s]
MIN_LINEAR_VEL = -1.0  # minimum linear velocity [m/s]
MAX_ANGULAR_VEL = 3.0  # maximum angular velocity [rad/s]
MIN_ANGULAR_VEL = -3.0  # minimum angular velocity [rad/s]
MAX_DLINEAR_VEL = 2.0  # maximum linear acceleration [m/s²]
MAX_DANGULAR_VEL = 3.0  # maximum angular acceleration [rad/s²]

class State:
    """Vehicle state class"""
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

class MPCNode(Node):
    def __init__(self):
        # Use a unique node name to avoid conflicts
        super().__init__('mpc_path_tracker')
        
        # Declare and retrieve parameters (with error handling)
        self._declare_parameter_if_not_exists("mode", "car")
        self._declare_parameter_if_not_exists("target_speed", 0.8)
        
        # Get parameter values with fallback defaults
        try:
            self.mode = self.get_parameter("mode").get_parameter_value().string_value
        except:
            self.mode = "car"
            self.get_logger().warn("Using default mode: car")
            
        try:
            self.target_speed = self.get_parameter("target_speed").get_parameter_value().double_value
        except:
            self.target_speed = 0.8
            self.get_logger().warn("Using default target_speed: 0.8 m/s")
        
        # Set up subscriber and timer
        self.odom_sub = self.create_subscription(Odometry, '/odometry/ground_truth', self.odom_callback, 10)
        self.create_timer(DT, self.timer_callback)  # MPC control loop
        self.robot_odom = None  # Initialize as None, will be set in callback
        
        # Set up publishers
        self.steering_pub = self.create_publisher(Float64MultiArray, "/steering_controller/commands", 10)
        self.velocity_pub = self.create_publisher(Float64MultiArray, "/velocity_controller/commands", 10)
        self.path_pub = self.create_publisher(Path, "/path", 10)
        
        # Load the path from YAML
        self.path = self.read_path()
        self.path_index = 0
        self.get_logger().info(f"Loaded path with {self.path.shape[0]} points." if self.path is not None else "Path not loaded!")
        
        # Publish the full path
        self.publish_path()
        
        # Vehicle parameters (from FK node)
        self.wheel_radius = WHEEL_RADIUS  # [m]
        self.l = WHEEL_BASE              # wheelbase for steering conversion
        self.track = TRACK_WIDTH         # [m], track width
        
        # MPC state variables
        self.current_state = State()
        self.target_ind = 0
        self.prev_linear_vel = 0.0  # previous linear velocity commands
        self.prev_angular_vel = 0.0  # previous angular velocity commands
        
        # Convert path to MPC format
        if self.path is not None:
            self.cx = self.path[:, 0]  # x coordinates
            self.cy = self.path[:, 1]  # y coordinates
            self.cyaw = self.path[:, 2]  # yaw angles
            self.ck = self.calculate_curvature()  # curvature
            self.sp = self.calc_speed_profile()  # speed profile
            self.dl = self.calculate_path_spacing()  # path spacing
            self.cyaw = self.smooth_yaw(self.cyaw)  # smooth yaw angles
            
            self.get_logger().info("MPC Controller initialized successfully")
            self.get_logger().info(f"Vehicle params: wheelbase={WB:.3f}m, track={TRACK_WIDTH:.3f}m, wheel_radius={WHEEL_RADIUS:.3f}m")
            self.get_logger().info(f"Target speed: {self.target_speed:.2f}m/s, Control mode: differential drive")
            self.get_logger().info(f"MPC params: T={T}, DT={DT:.3f}s, MAX_LINEAR_VEL={MAX_LINEAR_VEL:.1f}m/s, MAX_ANGULAR_VEL={MAX_ANGULAR_VEL:.1f}rad/s")
        else:
            self.get_logger().error("Failed to load path - controller inactive")

    def _declare_parameter_if_not_exists(self, name, default_value):
        """Helper method to declare parameter only if it doesn't exist"""
        try:
            self.declare_parameter(name, default_value)
        except ParameterAlreadyDeclaredException:
            self.get_logger().debug(f"Parameter '{name}' already declared, using existing value")

    def read_path(self):
        """Load path from YAML file (same as Stanley controller)"""
        try:
            pkg = get_package_share_directory('limo_controller')
            yaml_path = os.path.join(pkg, 'path', 'path.yaml')
            
            with open(yaml_path, 'r') as file:
                data = yaml.safe_load(file)
                
            path_points = []
            for point in data:
                x = point.get('x')
                y = point.get('y')
                yaw = point.get('yaw')
                if x is None or y is None or yaw is None:
                    self.get_logger().warn("Point missing required key: 'x', 'y', or 'yaw'.")
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

    def calculate_curvature(self):
        """Calculate curvature for each point in path"""
        curvature = []
        for i in range(len(self.path)):
            if i == 0 or i == len(self.path) - 1:
                curvature.append(0.0)
            else:
                # Simple curvature approximation
                x1, y1 = self.path[i-1][:2]
                x2, y2 = self.path[i][:2]
                x3, y3 = self.path[i+1][:2]
                
                # Calculate curvature using three points
                dx1, dy1 = x2 - x1, y2 - y1
                dx2, dy2 = x3 - x2, y3 - y2
                
                ds1 = math.sqrt(dx1**2 + dy1**2)
                ds2 = math.sqrt(dx2**2 + dy2**2)
                
                if ds1 < 1e-6 or ds2 < 1e-6:
                    curvature.append(0.0)
                else:
                    # Cross product for curvature
                    cross = dx1 * dy2 - dy1 * dx2
                    k = 2 * cross / (ds1 * ds2 * (ds1 + ds2))
                    curvature.append(k)
        
        return curvature

    def calc_speed_profile(self):
        """Generate speed profile based on curvature"""
        speed_profile = []
        for i, k in enumerate(self.ck):
            # Reduce speed in high curvature areas (less aggressive for better tracking)
            if abs(k) > 1.0:  # Very tight turns
                speed = self.target_speed * 0.4
            elif abs(k) > 0.5:  # Moderate turns
                speed = self.target_speed * 0.7
            else:  # Straight or gentle curves
                speed = self.target_speed
            speed_profile.append(speed)
        
        # Gradually reduce speed near the end
        if len(speed_profile) > 5:
            speed_profile[-5] = self.target_speed * 0.8
            speed_profile[-4] = self.target_speed * 0.6
            speed_profile[-3] = self.target_speed * 0.4
            speed_profile[-2] = self.target_speed * 0.2
            speed_profile[-1] = 0.05  # Almost stop at end
        
        return speed_profile

    def calculate_path_spacing(self):
        """Calculate average spacing between path points"""
        if len(self.path) < 2:
            return 0.1
        
        distances = []
        for i in range(1, len(self.path)):
            dx = self.path[i][0] - self.path[i-1][0]
            dy = self.path[i][1] - self.path[i-1][1]
            distances.append(math.sqrt(dx**2 + dy**2))
        
        return np.mean(distances)

    def smooth_yaw(self, yaw):
        """Smooth yaw angles to avoid discontinuities"""
        yaw_smooth = list(yaw)
        for i in range(len(yaw_smooth) - 1):
            dyaw = yaw_smooth[i + 1] - yaw_smooth[i]
            
            while dyaw >= math.pi / 2.0:
                yaw_smooth[i + 1] -= math.pi * 2.0
                dyaw = yaw_smooth[i + 1] - yaw_smooth[i]
            
            while dyaw <= -math.pi / 2.0:
                yaw_smooth[i + 1] += math.pi * 2.0
                dyaw = yaw_smooth[i + 1] - yaw_smooth[i]
        
        return yaw_smooth

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def calc_nearest_index(self, state, cx, cy, cyaw, pind):
        """Find nearest point on path"""
        dx = [state.x - icx for icx in cx[pind:(pind + N_IND_SEARCH)]]
        dy = [state.y - icy for icy in cy[pind:(pind + N_IND_SEARCH)]]
        
        d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]
        
        mind = min(d)
        ind = d.index(mind) + pind
        mind = math.sqrt(mind)
        
        dxl = cx[ind] - state.x
        dyl = cy[ind] - state.y
        
        angle = self.normalize_angle(cyaw[ind] - math.atan2(dyl, dxl))
        if angle < 0:
            mind *= -1
        
        return ind, mind

    def calc_ref_trajectory(self, state, cx, cy, cyaw, ck, sp, dl, pind):
        """Calculate reference trajectory for MPC"""
        xref = np.zeros((NX, T + 1))
        ncourse = len(cx)
        
        ind, _ = self.calc_nearest_index(state, cx, cy, cyaw, pind)
        
        if pind >= ind:
            ind = pind
        
        xref[0, 0] = cx[ind]
        xref[1, 0] = cy[ind]
        xref[2, 0] = sp[ind]
        xref[3, 0] = cyaw[ind]
        
        travel = 0.0
        
        for i in range(T + 1):
            travel += abs(state.v) * DT
            dind = int(round(travel / dl))
            
            if (ind + dind) < ncourse:
                xref[0, i] = cx[ind + dind]
                xref[1, i] = cy[ind + dind]
                xref[2, i] = sp[ind + dind]
                xref[3, i] = cyaw[ind + dind]
            else:
                xref[0, i] = cx[ncourse - 1]
                xref[1, i] = cy[ncourse - 1]
                xref[2, i] = sp[ncourse - 1]
                xref[3, i] = cyaw[ncourse - 1]
        
        return xref, ind

    def get_linear_model_matrix(self, v, phi, linear_vel_cmd, angular_vel_cmd):
        """Get linearized model matrices for differential drive"""
        A = np.zeros((NX, NX))
        A[0, 0] = 1.0
        A[1, 1] = 1.0
        A[2, 2] = 1.0
        A[3, 3] = 1.0
        
        # Linearized dynamics around operating point
        A[0, 2] = DT * math.cos(phi)  # dx/dv
        A[0, 3] = -DT * v * math.sin(phi)  # dx/dyaw
        A[1, 2] = DT * math.sin(phi)  # dy/dv
        A[1, 3] = DT * v * math.cos(phi)  # dy/dyaw
        
        B = np.zeros((NX, NU))
        B[0, 0] = DT * math.cos(phi)  # dx/du_linear
        B[1, 0] = DT * math.sin(phi)  # dy/du_linear
        B[2, 0] = 1.0  # dv/du_linear (direct velocity control)
        B[3, 1] = DT  # dyaw/du_angular
        
        C = np.zeros(NX)
        C[0] = DT * v * math.sin(phi) * phi
        C[1] = -DT * v * math.cos(phi) * phi
        C[2] = -v  # offset for velocity tracking
        
        return A, B, C

    def predict_motion(self, x0, linear_vels, angular_vels, xref):
        """Predict motion for MPC using differential drive model"""
        xbar = xref * 0.0
        for i, _ in enumerate(x0):
            xbar[i, 0] = x0[i]
        
        state = State(x=x0[0], y=x0[1], yaw=x0[3], v=x0[2])
        for (linear_vel, angular_vel, i) in zip(linear_vels, angular_vels, range(1, T + 1)):
            state = self.update_state(state, linear_vel, angular_vel)
            xbar[0, i] = state.x
            xbar[1, i] = state.y
            xbar[2, i] = state.v
            xbar[3, i] = state.yaw
        
        return xbar

    def update_state(self, state, linear_vel, angular_vel):
        """Update vehicle state using differential drive model"""
        # Input constraints
        linear_vel = np.clip(linear_vel, MIN_LINEAR_VEL, MAX_LINEAR_VEL)
        angular_vel = np.clip(angular_vel, MIN_ANGULAR_VEL, MAX_ANGULAR_VEL)
        
        # Differential drive dynamics
        state.x = state.x + linear_vel * math.cos(state.yaw) * DT
        state.y = state.y + linear_vel * math.sin(state.yaw) * DT
        state.yaw = state.yaw + angular_vel * DT
        state.v = linear_vel  # Direct velocity control
        
        return state

    def linear_mpc_control(self, xref, xbar, x0):
        """Linear MPC control for differential drive"""
        x = cvxpy.Variable((NX, T + 1))
        u = cvxpy.Variable((NU, T))
        
        cost = 0.0
        constraints = []
        
        for t in range(T):
            cost += cvxpy.quad_form(u[:, t], R)
            
            if t != 0:
                cost += cvxpy.quad_form(xref[:, t] - x[:, t], Q)
            
            A, B, C = self.get_linear_model_matrix(
                xbar[2, t], xbar[3, t], xbar[2, t], 0.0)  # Using current velocity and yaw
            constraints += [x[:, t + 1] == A @ x[:, t] + B @ u[:, t] + C]
            
            if t < (T - 1):
                cost += cvxpy.quad_form(u[:, t + 1] - u[:, t], Rd)
                constraints += [cvxpy.abs(u[0, t + 1] - u[0, t]) <= MAX_DLINEAR_VEL * DT]
                constraints += [cvxpy.abs(u[1, t + 1] - u[1, t]) <= MAX_DANGULAR_VEL * DT]
        
        cost += cvxpy.quad_form(xref[:, T] - x[:, T], Qf)
        
        constraints += [x[:, 0] == x0]
        constraints += [x[2, :] <= MAX_LINEAR_VEL]
        constraints += [x[2, :] >= MIN_LINEAR_VEL]
        constraints += [u[0, :] <= MAX_LINEAR_VEL]
        constraints += [u[0, :] >= MIN_LINEAR_VEL]
        constraints += [u[1, :] <= MAX_ANGULAR_VEL]
        constraints += [u[1, :] >= MIN_ANGULAR_VEL]
        
        prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
        prob.solve(solver=cvxpy.CLARABEL, verbose=False)
        
        if prob.status == cvxpy.OPTIMAL or prob.status == cvxpy.OPTIMAL_INACCURATE:
            ox = np.array(x.value[0, :]).flatten()
            oy = np.array(x.value[1, :]).flatten()
            ov = np.array(x.value[2, :]).flatten()
            oyaw = np.array(x.value[3, :]).flatten()
            linear_vels = np.array(u.value[0, :]).flatten()
            angular_vels = np.array(u.value[1, :]).flatten()
        else:
            self.get_logger().error("Cannot solve MPC optimization")
            linear_vels, angular_vels, ox, oy, oyaw, ov = None, None, None, None, None, None
        
        return linear_vels, angular_vels, ox, oy, oyaw, ov

    def iterative_linear_mpc_control(self, xref, x0, dref, prev_linear_vels, prev_angular_vels):
        """Iterative linear MPC control"""
        if prev_linear_vels is None or prev_angular_vels is None:
            prev_linear_vels = [0.0] * T
            prev_angular_vels = [0.0] * T
        
        for i in range(MAX_ITER):
            xbar = self.predict_motion(x0, prev_linear_vels, prev_angular_vels, xref)
            old_linear_vels, old_angular_vels = prev_linear_vels[:], prev_angular_vels[:]
            
            linear_vels, angular_vels, ox, oy, oyaw, ov = self.linear_mpc_control(xref, xbar, x0)
            
            if linear_vels is None:
                break
            
            prev_linear_vels = linear_vels
            prev_angular_vels = angular_vels
            
            du = (sum(abs(np.array(linear_vels) - np.array(old_linear_vels))) + 
                  sum(abs(np.array(angular_vels) - np.array(old_angular_vels))))
            
            if du <= DU_TH:
                break
        
        return linear_vels, angular_vels, ox, oy, oyaw, ov

    def check_path_completion(self, state, path_length, target_idx):
        """Check if we've reached the end of the path (for continuous tracking)"""
        # Check if we're near the end of the path
        remaining_points = path_length - target_idx
        
        if remaining_points <= 3:  # Near end of path
            # Check if we're close to the last point
            dx = state.x - self.cx[-1]
            dy = state.y - self.cy[-1]
            distance_to_end = math.hypot(dx, dy)
            
            # If close to end and moving slowly, consider path completed
            if distance_to_end < 0.2 and abs(state.v) < 0.1:  # Tighter thresholds
                return True
                
        return False

    def mpc_control(self):
        """Main MPC control function"""
        if self.path is None:
            self.get_logger().warn("No path available for MPC control")
            return
            
        # Check if robot odometry is available
        if not hasattr(self.robot_odom, 'pose') or self.robot_odom.pose is None:
            self.get_logger().debug("Robot odometry not available yet")
            return
        
        # Update current state from odometry
        try:
            x = self.robot_odom.pose.pose.position.x
            y = self.robot_odom.pose.pose.position.y
            orientation_q = self.robot_odom.pose.pose.orientation
            (_, _, yaw) = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
            
            # Get velocity from odometry
            v = math.sqrt(self.robot_odom.twist.twist.linear.x**2 + self.robot_odom.twist.twist.linear.y**2)
            
            self.current_state = State(x=x, y=y, yaw=yaw, v=v)
            
        except Exception as e:
            self.get_logger().error(f"Error reading odometry: {e}")
            return
        
        # Check if path is completed (for continuous tracking)
        if self.check_path_completion(self.current_state, len(self.cx), self.target_ind):
            self.get_logger().info("Path completed - stopping vehicle")
            self.cmd_vel_direct(0.0, 0.0)
            return
        
        # Calculate reference trajectory
        xref, self.target_ind = self.calc_ref_trajectory(
            self.current_state, self.cx, self.cy, self.cyaw, self.ck, self.sp, self.dl, self.target_ind)
        
        # Current state vector
        x0 = [self.current_state.x, self.current_state.y, self.current_state.v, self.current_state.yaw]
        
        # Create previous command arrays
        prev_linear_vels = [self.prev_linear_vel] * T
        prev_angular_vels = [self.prev_angular_vel] * T
        
        # Solve MPC (dref is not used in differential drive)
        linear_vels, angular_vels, ox, oy, oyaw, ov = self.iterative_linear_mpc_control(
            xref, x0, None, prev_linear_vels, prev_angular_vels)
        
        # Apply first control input
        if linear_vels is not None and angular_vels is not None:
            linear_vel = linear_vels[0]
            angular_vel = angular_vels[0]
            
            # Store for next iteration
            self.prev_linear_vel = linear_vel
            self.prev_angular_vel = angular_vel
            
            # Send commands directly to robot
            self.cmd_vel_direct(linear_vel, angular_vel)
            
            # Progress info
            progress = (self.target_ind / len(self.cx)) * 100
            distance_to_target = math.hypot(self.current_state.x - self.cx[self.target_ind], 
                                          self.current_state.y - self.cy[self.target_ind])
            
            self.get_logger().info(f"MPC: v={linear_vel:.3f}m/s, ω={angular_vel:.3f}rad/s, progress={progress:.1f}%, dist={distance_to_target:.3f}m")
        else:
            self.get_logger().warn("MPC optimization failed")

    def cmd_vel_direct(self, linear_vel, angular_vel):
        """Send velocity commands directly to robot"""
        # Limit velocities
        linear_vel = np.clip(linear_vel, MIN_LINEAR_VEL, MAX_LINEAR_VEL)
        angular_vel = np.clip(angular_vel, MIN_ANGULAR_VEL, MAX_ANGULAR_VEL)
        
        # Convert to wheel speeds for differential drive
        # v_left = linear_vel - (angular_vel * track_width / 2)
        # v_right = linear_vel + (angular_vel * track_width / 2)
        v_left = linear_vel - (angular_vel * self.track / 2.0)
        v_right = linear_vel + (angular_vel * self.track / 2.0)
        
        # Convert to wheel angular velocities
        wheel_speed_left = v_left / self.wheel_radius
        wheel_speed_right = v_right / self.wheel_radius
        
        # Set steering angles to 0 (differential drive)
        self.set_steering_angle(0.0, 0.0)
        
        # Set wheel velocities
        self.set_velocity(wheel_speed_left, wheel_speed_right)
        
        # Debug output
        self.get_logger().debug(f"Commands: linear={linear_vel:.3f}, angular={angular_vel:.3f}, wheels=[{wheel_speed_left:.2f}, {wheel_speed_right:.2f}]")

    def cmd_vel_steering(self, vx, steering_angle):
        """Convert desired steering angle and velocity to wheel commands (legacy function)"""
        # This function is kept for compatibility but not used in MPC
        self.get_logger().warn("cmd_vel_steering called but MPC uses cmd_vel_direct")
        self.cmd_vel_direct(vx, 0.0)

    def timer_callback(self):
        """Timer callback for MPC control loop"""
        # Check if robot odometry is available and initialized
        if (self.robot_odom is None or 
            not hasattr(self.robot_odom, 'pose') or 
            self.robot_odom.pose is None or 
            self.robot_odom.pose.pose is None):
            self.get_logger().debug("Waiting for odometry data...")
            return
            
        self.publish_path()
        self.mpc_control()

    def odom_callback(self, msg: Odometry):
        """Odometry callback"""
        self.robot_odom = msg

    def set_steering_angle(self, left_angle, right_angle):
        """Set steering angles"""
        msg = Float64MultiArray()
        msg.layout.data_offset = 0
        msg.data = [float(left_angle), float(right_angle)]
        self.steering_pub.publish(msg)

    def set_velocity(self, left_speed, right_speed):
        """Set wheel velocities"""
        msg = Float64MultiArray()
        msg.layout.data_offset = 0
        msg.data = [float(left_speed), float(right_speed)]
        self.velocity_pub.publish(msg)

def main(args=None):
    try:
        rclpy.init(args=args)
        node = MPCNode()
        
        if node.path is None:
            node.get_logger().error("Failed to initialize MPC controller - no valid path")
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

if __name__ == '__main__':
    main()