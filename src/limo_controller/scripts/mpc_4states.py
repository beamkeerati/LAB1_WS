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

import numpy as np
import math
import cvxpy
import matplotlib.pyplot as plt
from PathPlanning.CubicSpline import cubic_spline_planner
from mpc_lib import *

NX = 4  # x = x, y, v, yaw
NU = 2  # a = [accel, steer]
T = 5  # horizon length

# mpc parameters
R = np.diag([0.01, 0.01])  # input cost matrix
Rd = np.diag([0.01, 1.0])  # input difference cost matrix
Q = np.diag([1.0, 1.0, 0.5, 0.5])  # state cost matrix
Qf = Q  # state final matrix
GOAL_DIS = 1.5  # goal distance
STOP_SPEED = 0.5 / 3.6  # stop speed
MAX_TIME = 500.0  # max simulation time

# iterative paramter
MAX_ITER = 3  # Max iteration
DU_TH = 0.1  # iteration finish param

TARGET_SPEED = 10.0 / 3.6  # [m/s] target speed
N_IND_SEARCH = 10  # Search index number

DT = 0.01  # [s] time tick

# Vehicle parameters
LENGTH = 4.5  # [m]
WIDTH = 2.0  # [m]
BACKTOWHEEL = 1.0  # [m]
WHEEL_LEN = 0.3  # [m]
WHEEL_WIDTH = 0.2  # [m]
TREAD = 0.7  # [m]
WB = 1.0  # [m]
WHEEL_RADIUS = 0.3  # [m]
WHEEL_BASE = 1.0  # [m]
TRACK_WIDTH = 0.5  # [m] track width

MAX_STEER = math.radians(10.0)  # maximum steering angle [rad]
MAX_DSTEER = math.radians(30.0)  # maximum steering speed [rad/s]
MAX_SPEED = 55.0 / 3.6  # maximum speed [m/s]
MIN_SPEED = -20.0 / 3.6  # minimum speed [m/s]
MAX_ACCEL = 1.0  # maximum accel [m/ss]
MAX_LINEAR_VEL = 1.0  # maximum linear velocity [m/s]
MIN_LINEAR_VEL = -1.0  # minimum linear velocity [m/s]
MAX_ANGULAR_VEL = math.radians(30.0)  # maximum angular velocity [rad/s]
MIN_ANGULAR_VEL = -math.radians(30.0)  # minimum angular velocity [rad/s]



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
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.path_pub = self.create_publisher(Path, "/path", 10)
        
        # Load the path from YAML
        self.path = self.read_path()
        self.path_index = 0
        self.get_logger().info(f"Loaded path with {self.path.shape[0]} points." if self.path is not None else "Path not loaded!")
        
        # Publish the full path
        self.publish_path()
        
        #------------------------------------------#
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
            self.init_path()
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

    def init_path(self):
        # Path variables
        self.cx = self.path[:, 0]  # x coordinates
        self.cy = self.path[:, 1]  # y coordinates
        self.cyaw = self.path[:, 2]  # yaw angles
        self.ck = [0.0] * len(self.cx) # curvature
        self.sp = calc_speed_profile(self.cx, self.cy, self.cyaw, TARGET_SPEED) # speed profile
        self.dl = calculate_path_distance(self.cx, self.cy) # path spacing
        self.cyaw = smooth_yaw(self.cyaw)  # smooth yaw angles
        
        self.get_logger().info("MPC Controller initialized successfully")
        self.get_logger().info(f"Vehicle params: wheelbase={WB:.3f}m, track={TRACK_WIDTH:.3f}m, wheel_radius={WHEEL_RADIUS:.3f}m")
        self.get_logger().info(f"Target speed: {self.target_speed:.2f}m/s, Control mode: differential drive")
        self.get_logger().info(f"MPC params: T={T}, DT={DT:.3f}s, MAX_LINEAR_VEL={MAX_LINEAR_VEL:.1f}m/s, MAX_ANGULAR_VEL={MAX_ANGULAR_VEL:.1f}rad/s")
        
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
            self.current_state = self.get_state(self.robot_odom)
        except Exception as e:
            self.get_logger().error(f"Error reading odometry: {e}")
            return
        
        




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

    def get_state(self, robot_odom):
        """Extract state from odometry message"""
        x = robot_odom.pose.pose.position.x
        y = robot_odom.pose.pose.position.y
        orientation_q = robot_odom.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        v = robot_odom.twist.twist.linear.x

        return State(x=x, y=y, yaw=yaw, v=v)

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