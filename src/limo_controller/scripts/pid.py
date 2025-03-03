#!/usr/bin/python3
from dataclasses import dataclass
import rclpy
from rclpy.node import Node
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
from ament_index_python.packages import get_package_share_directory
import matplotlib.pyplot as plt  # Only needed for debugging or plotting

class PIDNode(Node):
    def __init__(self):
        super().__init__('pid_node')
        
        # --- Parameters and Dynamic Tuning ---
        # Mode ("car" or "bicycle")
        self.declare_parameter("mode", "car")
        self.mode = self.get_parameter("mode").get_parameter_value().string_value
        
        # PID gains (default values; dynamic tuning is supported)
        self.declare_parameter("Kp", 10.0)
        self.declare_parameter("Ki", 1.0)
        self.declare_parameter("Kd", 0.0)
        self.Kp = self.get_parameter("Kp").get_parameter_value().double_value
        self.Ki = self.get_parameter("Ki").get_parameter_value().double_value
        self.Kd = self.get_parameter("Kd").get_parameter_value().double_value
        
        # Constant forward velocity for path tracking (m/s)
        self.declare_parameter("target_speed", 2.0)
        self.target_speed = self.get_parameter("target_speed").get_parameter_value().double_value
        
        # Control loop period in seconds
        self.declare_parameter("control_period", 0.01)
        self.control_period = self.get_parameter("control_period").get_parameter_value().double_value
        
        # Initialize PID state variables
        self.integral_error = 0.0
        self.last_error = None

        # --- Subscribers, Publishers, and Timer ---
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/ground_truth', self.odom_callback, 10)
        self.create_timer(self.control_period, self.timer_callback)
        self.robot_odom = Odometry()
        
        self.steering_pub = self.create_publisher(
            Float64MultiArray, "/steering_controller/commands", 10)
        self.velocity_pub = self.create_publisher(
            Float64MultiArray, "/velocity_controller/commands", 10)
        self.path_pub = self.create_publisher(Path, "/path", 10)
        
        # Load the path from YAML into a NumPy array (each row: [x, y, yaw])
        self.path = self.read_path()
        self.path_index = 0  # pointer to the next target point
        if self.path is not None:
            self.get_logger().info(f"Loaded path with {self.path.shape[0]} points.")
        else:
            self.get_logger().error("Path not loaded!")
        
        # Publish the full path for visualization in RViz
        self.publish_path()
        
        # --- Vehicle Parameters ---
        self.wheel_radius = 0.3    # [m]
        self.l = 1.0               # wheelbase (for "car" mode)
        self.track = 0.5           # track width [m]
        self.L = 1.0               # vehicle wheelbase used for steering angle calculation
        
        # --- Robot State ---
        self.currentPos = [0.0, 0.0]  # current [x, y] position
        self.currentHeading = 0.0     # current heading (in degrees)
        
        # --- Dynamic Parameter Callback for PID Gains ---
        self.add_on_set_parameters_callback(self.parameter_callback)
    
    def parameter_callback(self, params):
        for param in params:
            if param.name == "Kp":
                self.Kp = param.value
                self.get_logger().info(f"Updated Kp: {self.Kp}")
            elif param.name == "Ki":
                self.Ki = param.value
                self.get_logger().info(f"Updated Ki: {self.Ki}")
            elif param.name == "Kd":
                self.Kd = param.value
                self.get_logger().info(f"Updated Kd: {self.Kd}")
            elif param.name == "target_speed":
                self.target_speed = param.value
                self.get_logger().info(f"Updated target_speed: {self.target_speed}")
            elif param.name == "control_period":
                self.control_period = param.value
                self.get_logger().info(f"Updated control_period: {self.control_period}")
        return rclpy.parameter.SetParametersResult(successful=True)
    
    def read_path(self):
        """
        Reads a YAML file containing a list of points.
        Each point is a dictionary with keys 'x', 'y', and 'yaw'.
        Returns a NumPy array with shape (n, 3) where each row is [x, y, yaw].
        """
        pkg = get_package_share_directory('limo_controller')
        yaml_path = os.path.join(pkg, 'path', 'path.yaml')
        try:
            with open(yaml_path, 'r') as file:
                data = yaml.safe_load(file)
        except Exception as e:
            self.get_logger().error(f"Failed to read YAML file: {e}")
            return None
        
        try:
            path_points = []
            for point in data:
                x = point.get('x')
                y = point.get('y')
                yaw = point.get('yaw')
                if x is None or y is None or yaw is None:
                    self.get_logger().warn("One of the points is missing a required key: 'x', 'y', or 'yaw'.")
                    continue
                path_points.append([x, y, yaw])
            return np.array(path_points)
        except Exception as e:
            self.get_logger().error(f"Error processing YAML data: {e}")
            return None

    def publish_path(self):
        """
        Converts the loaded path (NumPy array of [x, y, yaw]) into a nav_msgs/Path message
        and publishes it on the /path topic.
        """
        if self.path is None:
            self.get_logger().error("No path to publish.")
            return
        
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "world"
        
        for point in self.path:
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.header.frame_id = "map"
            pose_stamped.pose.position.x = point[0]
            pose_stamped.pose.position.y = point[1]
            pose_stamped.pose.position.z = 0.0
            q = quaternion_from_euler(0, 0, point[2])
            pose_stamped.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            path_msg.poses.append(pose_stamped)
        
        self.path_pub.publish(path_msg)
    
    def odom_callback(self, msg: Odometry):
        self.robot_odom = msg
        # Update current position and heading from odometry
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.currentPos = [x, y]
        self.currentHeading = math.degrees(yaw)
    
    def timer_callback(self):
        # Publish path (optional, e.g., for RViz visualization)
        self.publish_path()
        # Execute PID-based path tracking control
        self.pid_control()
    
    def pid_control(self):
        """
        PID control for path tracking:
          1. Compute cross-track error using the nearest point on the path.
          2. Use PID (with proportional, integral, and derivative terms) to compute a steering correction.
          3. Publish the command with a constant forward velocity.
        """
        if self.path is None or self.path.shape[0] == 0:
            self.get_logger().error("Path is empty, cannot perform PID control.")
            return
        
        # 1. Compute cross-track error and the index of the nearest path point.
        error, nearest_index = self.compute_cross_track_error()
        
        # 2. PID update
        dt = self.control_period
        derivative = 0.0
        if self.last_error is not None:
            derivative = (error - self.last_error) / dt
        self.integral_error += error * dt
        self.last_error = error
        steering_correction = self.Kp * error + self.Ki * self.integral_error + self.Kd * derivative
        
        # Limit steering command to physical bounds (example: ±10° in radians)
        max_steering_rad = 10.0 * math.pi / 180.0
        steering_correction = np.clip(steering_correction, -max_steering_rad, max_steering_rad)
        
        self.get_logger().info(f"Nearest index: {nearest_index}, Cross-track error: {error:.3f}, Steering correction: {steering_correction:.3f}")
        
        # 3. Use the constant forward speed from parameters
        vx = self.target_speed
        
        # Command the vehicle's velocity and steering angle
        self.cmd_vel_steering(vx, steering_correction)
    
    def compute_cross_track_error(self):
        """
        Computes the cross-track error from the current robot position to the path.
        The error is calculated as the signed perpendicular distance from the nearest path point,
        using the path’s heading to determine the sign.
        
        Returns:
        error: signed distance (positive means robot is to the right of the path)
        nearest_index: index of the nearest path point
        """
        # Current robot position
        x_robot, y_robot = self.currentPos
        # Extract path positions (assumes self.path is a NumPy array of shape (n,3))
        path_xy = self.path[:, :2]
        # Compute Euclidean distances using np.hypot (more concise and numerically stable)
        dists = np.hypot(path_xy[:, 0] - x_robot, path_xy[:, 1] - y_robot)
        nearest_index = int(np.argmin(dists))
        
        # Get the nearest path point and its yaw (heading)
        nearest_point = self.path[nearest_index]
        x_path, y_path, yaw_path = nearest_point[0], nearest_point[1], nearest_point[2]
        # Compute cross-track error:
        # The error is given by the dot product of (robot_position - path_point) and the unit vector orthogonal to the path.
        error = (x_robot - x_path) * (-math.sin(yaw_path)) + (y_robot - y_path) * math.cos(yaw_path)
        
        return -error, nearest_index

    
    def cmd_vel_steering(self, vx, steering_angle):
        """
        Converts the desired linear velocity and steering correction into wheel commands,
        and publishes these commands on the appropriate topics.
        """
        # If velocity is zero, stop the robot
        if vx == 0:
            self.set_steering_angle(0, 0)
            self.set_velocity(0, 0)
            return
        
        # Limit steering angle (example limits: ±10° in radians)
        angle_max = 10.0/180.0 * math.pi
        angle_min = -10.0/180.0 * math.pi
        steering_angle = np.clip(steering_angle, angle_min, angle_max)
        
        # Convert linear velocity to wheel speed using the wheel radius.
        wheel_speed = vx / self.wheel_radius
        
        # Compute left and right steering angles based on mode
        if self.mode == "bicycle":
            left_angle = right_angle = steering_angle
        elif self.mode == "car":
            tan_steering = math.tan(steering_angle)
            left_angle = math.atan((self.l * tan_steering) / (self.l + 0.5 * self.track * tan_steering))
            right_angle = math.atan((self.l * tan_steering) / (self.l - 0.5 * self.track * tan_steering))
        else:
            self.get_logger().warn(f"Unknown mode: {self.mode}. Command ignored.")
            return
        
        self.set_steering_angle(left_angle, right_angle)
        self.set_velocity(wheel_speed, wheel_speed)
    
    def set_steering_angle(self, left_angle, right_angle):
        msg = Float64MultiArray()
        msg.layout.data_offset = 0
        msg.data = [float(left_angle), float(right_angle)]
        self.steering_pub.publish(msg)
    
    def set_velocity(self, left_speed, right_speed):
        msg = Float64MultiArray()
        msg.layout.data_offset = 0
        msg.data = [float(left_speed), float(right_speed)]
        self.velocity_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PIDNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
