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

# MPC Parameters (adapted from working example)
NX = 3  # states: x, y, yaw (differential drive model)
NU = 2  # inputs: v, w (linear_vel, angular_vel)
T = 5   # horizon length

# MPC cost matrices (adapted for 3-state system)
R = np.diag([0.01, 0.01])  # input cost matrix
Rd = np.diag([0.01, 1.0])  # input difference cost matrix  
Q = np.diag([1.0, 1.0, 0.5])  # state cost matrix (3 states)
Qf = Q  # state final matrix

# MPC algorithm parameters (from working example)
MAX_ITER = 3  # Max iteration
DU_TH = 0.1  # iteration finish param
N_IND_SEARCH = 10  # Search index number
DT = 0.2  # [s] time tick (increased from 0.01 for better performance)

# Vehicle constraints (for differential drive)
MAX_LINEAR_VEL = 2.0   # maximum linear velocity [m/s]
MIN_LINEAR_VEL = -2.0  # minimum linear velocity [m/s]  
MAX_ANGULAR_VEL = 1.76327   # maximum angular velocity [rad/s]
MIN_ANGULAR_VEL = -1.76327  # minimum angular velocity [rad/s]

class State:
    """Vehicle state class for differential drive (3 states: x, y, yaw)"""
    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw

class MPCNode(Node):
    def __init__(self):
        super().__init__('mpc_path_tracker')
        
        # Declare and retrieve parameters
        self._declare_parameter_if_not_exists("target_speed", 0.8)
        
        try:
            self.target_speed = self.get_parameter("target_speed").get_parameter_value().double_value
        except:
            self.target_speed = 0.8
            self.get_logger().warn("Using default target_speed: 0.8 m/s")
        
        # Set up subscriber and timer
        self.odom_sub = self.create_subscription(Odometry, '/odometry/ground_truth', self.odom_callback, 10)
        self.create_timer(DT, self.timer_callback)
        self.robot_odom = None
        
        # Set up publishers
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.path_pub = self.create_publisher(Path, "/path", 10)
        
        # Load and process path
        self.path = self.read_path()
        
        if self.path is not None:
            self.cx = self.path[:, 0].tolist()  # convert to list like example
            self.cy = self.path[:, 1].tolist()
            self.cyaw = self.smooth_yaw(self.path[:, 2].tolist())
            self.ck = self.calculate_curvature()
            self.sp = self.calc_speed_profile()
            self.dl = self.calculate_path_spacing()
            
            self.publish_path()
            
            # MPC state variables
            self.current_state = State()
            self.target_ind = 0
            self.prev_v = None  # previous control commands
            self.prev_w = None
            
            self.get_logger().info("MPC Controller initialized successfully")
            self.get_logger().info(f"Path points: {len(self.cx)}, Target speed: {self.target_speed:.2f}m/s")
            self.get_logger().info(f"MPC params: T={T}, DT={DT:.2f}s, NX={NX}, NU={NU}")
        else:
            self.get_logger().error("Failed to load path - controller inactive")

    def _declare_parameter_if_not_exists(self, name, default_value):
        try:
            self.declare_parameter(name, default_value)
        except ParameterAlreadyDeclaredException:
            pass

    def read_path(self):
        """Load path from YAML file"""
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
                    continue
                path_points.append([x, y, yaw])
            
            if len(path_points) < 2:
                self.get_logger().error("Path must have at least 2 points")
                return None
                
            return np.array(path_points)
            
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
            pose_stamped.header = path_msg.header
            pose_stamped.pose.position.x = point[0]
            pose_stamped.pose.position.y = point[1]
            pose_stamped.pose.position.z = 0.0
            q = quaternion_from_euler(0, 0, point[2])
            pose_stamped.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            path_msg.poses.append(pose_stamped)
        
        self.path_pub.publish(path_msg)

    def calculate_curvature(self):
        """Calculate curvature (simplified - not used in differential drive)"""
        return [0.0] * len(self.cx)

    def calc_speed_profile(self):
        """Generate speed profile"""
        speed_profile = [self.target_speed] * len(self.cx)
        speed_profile[-1] = 0.0  # stop at end
        return speed_profile

    def calculate_path_spacing(self):
        """Calculate average path spacing"""
        if len(self.cx) < 2:
            return 0.1
        distances = []
        for i in range(1, len(self.cx)):
            dx = self.cx[i] - self.cx[i-1]
            dy = self.cy[i] - self.cy[i-1]
            distances.append(math.sqrt(dx**2 + dy**2))
        return np.mean(distances)

    def smooth_yaw(self, yaw):
        """Smooth yaw angles (from working example)"""
        for i in range(len(yaw) - 1):
            dyaw = yaw[i + 1] - yaw[i]
            while dyaw >= math.pi / 2.0:
                yaw[i + 1] -= math.pi * 2.0
                dyaw = yaw[i + 1] - yaw[i]
            while dyaw <= -math.pi / 2.0:
                yaw[i + 1] += math.pi * 2.0
                dyaw = yaw[i + 1] - yaw[i]
        return yaw

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def calc_nearest_index(self, state, cx, cy, cyaw, pind):
        """Find nearest point (from working example)"""
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
        """Calculate reference trajectory (adapted from working example)"""
        xref = np.zeros((NX, T + 1))
        ncourse = len(cx)
        
        ind, _ = self.calc_nearest_index(state, cx, cy, cyaw, pind)
        
        if pind >= ind:
            ind = pind
            
        xref[0, 0] = cx[ind]
        xref[1, 0] = cy[ind] 
        xref[2, 0] = cyaw[ind]
        
        travel = 0.0
        for i in range(T + 1):
            travel += abs(self.target_speed) * DT  # use target speed for planning
            dind = int(round(travel / dl))
            
            if (ind + dind) < ncourse:
                xref[0, i] = cx[ind + dind]
                xref[1, i] = cy[ind + dind]
                xref[2, i] = cyaw[ind + dind]
            else:
                xref[0, i] = cx[ncourse - 1]
                xref[1, i] = cy[ncourse - 1]
                xref[2, i] = cyaw[ncourse - 1]
        
        return xref, ind

    def get_linear_model_matrix(self, yaw):
        """Get linearized model matrices for differential drive"""
        A = np.zeros((NX, NX))
        A[0, 0] = 1.0
        A[1, 1] = 1.0
        A[2, 2] = 1.0
        
        B = np.zeros((NX, NU))
        B[0, 0] = DT * math.cos(yaw)  # dx/dv
        B[1, 0] = DT * math.sin(yaw)  # dy/dv
        B[2, 1] = DT                  # dyaw/dw
        
        C = np.zeros(NX)
        
        return A, B, C

    def update_state(self, state, v, w):
        """Update state with differential drive dynamics"""
        # Clip inputs
        v = np.clip(v, MIN_LINEAR_VEL, MAX_LINEAR_VEL)
        w = np.clip(w, MIN_ANGULAR_VEL, MAX_ANGULAR_VEL)
        
        # Update state
        state.x = state.x + v * math.cos(state.yaw) * DT
        state.y = state.y + v * math.sin(state.yaw) * DT
        state.yaw = state.yaw + w * DT
        
        return state

    def predict_motion(self, x0, ov, ow, xref):
        """Predict motion (adapted from working example)"""
        xbar = xref * 0.0
        for i, _ in enumerate(x0):
            xbar[i, 0] = x0[i]
            
        state = State(x=x0[0], y=x0[1], yaw=x0[2])
        for (vi, wi, i) in zip(ov, ow, range(1, T + 1)):
            state = self.update_state(state, vi, wi)
            xbar[0, i] = state.x
            xbar[1, i] = state.y
            xbar[2, i] = state.yaw
            
        return xbar

    def linear_mpc_control(self, xref, xbar, x0):
        """Linear MPC control (adapted from working example)"""
        x = cvxpy.Variable((NX, T + 1))
        u = cvxpy.Variable((NU, T))
        
        cost = 0.0
        constraints = []
        
        for t in range(T):
            cost += cvxpy.quad_form(u[:, t], R)
            
            if t != 0:
                cost += cvxpy.quad_form(xref[:, t] - x[:, t], Q)
                
            A, B, C = self.get_linear_model_matrix(xbar[2, t])  # use predicted yaw
            constraints += [x[:, t + 1] == A @ x[:, t] + B @ u[:, t] + C]
            
            if t < (T - 1):
                cost += cvxpy.quad_form(u[:, t + 1] - u[:, t], Rd)
        
        cost += cvxpy.quad_form(xref[:, T] - x[:, T], Qf)
        
        # Constraints
        constraints += [x[:, 0] == x0]
        constraints += [cvxpy.abs(u[0, :]) <= MAX_LINEAR_VEL]   # linear velocity bounds
        constraints += [cvxpy.abs(u[1, :]) <= MAX_ANGULAR_VEL]  # angular velocity bounds
        
        prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
        prob.solve(solver=cvxpy.CLARABEL, verbose=False)
        
        if prob.status == cvxpy.OPTIMAL or prob.status == cvxpy.OPTIMAL_INACCURATE:
            ox = np.array(x.value[0, :]).flatten()
            oy = np.array(x.value[1, :]).flatten()
            oyaw = np.array(x.value[2, :]).flatten()
            ov = np.array(u.value[0, :]).flatten()  # linear velocities
            ow = np.array(u.value[1, :]).flatten()  # angular velocities
        else:
            self.get_logger().error("Cannot solve MPC optimization")
            ov, ow, ox, oy, oyaw = None, None, None, None, None
            
        return ov, ow, ox, oy, oyaw

    def iterative_linear_mpc_control(self, xref, x0, ov, ow):
        """Iterative linear MPC (adapted from working example)"""
        ox, oy, oyaw = None, None, None
        
        if ov is None or ow is None:
            ov = [0.0] * T
            ow = [0.0] * T
            
        for i in range(MAX_ITER):
            xbar = self.predict_motion(x0, ov, ow, xref)
            pov, pow = ov[:], ow[:]
            ov, ow, ox, oy, oyaw = self.linear_mpc_control(xref, xbar, x0)
            
            if ov is None:
                break
                
            du = sum(abs(np.array(ov) - np.array(pov))) + sum(abs(np.array(ow) - np.array(pow)))
            if du <= DU_TH:
                break
        else:
            self.get_logger().debug("MPC reached max iterations")
            
        return ov, ow, ox, oy, oyaw

    def check_goal(self, state, goal):
        """Check if goal is reached"""
        dx = state.x - goal[0]
        dy = state.y - goal[1]
        d = math.hypot(dx, dy)
        return d <= 0.5  # goal tolerance

    def mpc_control(self):
        """Main MPC control function"""
        if self.path is None:
            self.get_logger().warn("No path available")
            return
            
        if self.robot_odom is None:
            self.get_logger().warn("No odometry available")
            return
            
        self.get_logger().info("MPC control running...")
            
        # Update current state from odometry
        try:
            pos = self.robot_odom.pose.pose.position
            orient = self.robot_odom.pose.pose.orientation
            _, _, yaw = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
            
            self.current_state = State(pos.x, pos.y, yaw)
            self.get_logger().info(f"Current state: x={pos.x:.3f}, y={pos.y:.3f}, yaw={yaw:.3f}")
        except Exception as e:
            self.get_logger().error(f"Error reading odometry: {e}")
            return
        
        # Check if goal reached
        goal = [self.cx[-1], self.cy[-1]]
        dist_to_goal = math.hypot(self.current_state.x - goal[0], self.current_state.y - goal[1])
        self.get_logger().info(f"Distance to goal: {dist_to_goal:.3f}m")
        
        if self.check_goal(self.current_state, goal):
            self.get_logger().info("Goal reached!")
            self.publish_cmd_vel(0.0, 0.0)
            return
            
        # Calculate reference trajectory
        self.get_logger().info("Calculating reference trajectory...")
        xref, self.target_ind = self.calc_ref_trajectory(
            self.current_state, self.cx, self.cy, self.cyaw, 
            self.ck, self.sp, self.dl, self.target_ind)
            
        # Current state vector
        x0 = [self.current_state.x, self.current_state.y, self.current_state.yaw]
        self.get_logger().info(f"Current state vector: {x0}")
        
        # Solve MPC
        self.get_logger().info("Solving MPC...")
        ov, ow, ox, oy, oyaw = self.iterative_linear_mpc_control(
            xref, x0, self.prev_v, self.prev_w)
            
        # Apply control
        if ov is not None and ow is not None:
            v_cmd, w_cmd = ov[0], ow[0]
            self.prev_v, self.prev_w = ov, ow
            
            self.get_logger().info(f"Publishing commands: v={v_cmd:.3f}, w={w_cmd:.3f}")
            self.publish_cmd_vel(v_cmd, w_cmd)
            
            # Log progress
            progress = (self.target_ind / len(self.cx)) * 100
            
            self.get_logger().info(f"MPC: v={v_cmd:.3f}, w={w_cmd:.3f}, progress={progress:.1f}%, dist_to_goal={dist_to_goal:.2f}m")
        else:
            self.get_logger().info("MPC optimization failed - ov or ow is None")

    def publish_cmd_vel(self, v, w):
        """Publish velocity commands"""
        msg = Twist()
        msg.linear.x = float(np.clip(v, MIN_LINEAR_VEL, MAX_LINEAR_VEL))
        msg.angular.z = float(np.clip(w, MIN_ANGULAR_VEL, MAX_ANGULAR_VEL))
        self.cmd_vel_pub.publish(msg)

    def timer_callback(self):
        """Timer callback for control loop"""
        self.get_logger().info("Timer callback triggered")
        
        if (self.robot_odom is None or 
            not hasattr(self.robot_odom, 'pose') or 
            self.robot_odom.pose is None):
            self.get_logger().info("Waiting for odometry...")
            return
            
        self.publish_path()
        self.mpc_control()

    def odom_callback(self, msg: Odometry):
        """Odometry callback"""
        self.robot_odom = msg

def main(args=None):
    try:
        rclpy.init(args=args)
        node = MPCNode()
        
        if node.path is None:
            node.get_logger().error("Failed to initialize - no valid path")
            return
            
        node.get_logger().info("MPC differential drive controller started")
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        node.get_logger().info("MPC controller stopped")
    except Exception as e:
        print(f"Error: {e}")
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