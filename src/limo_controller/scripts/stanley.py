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
import matplotlib.pyplot as plt  # Only needed if you later want to plot for debugging

class StanleyNode(Node):
    def __init__(self):
        super().__init__('stanley_node')
        
        # Declare and retrieve parameters
        self.declare_parameter("mode", "car")
        self.mode = self.get_parameter("mode").get_parameter_value().string_value
        
        # Set up subscriber and timer
        self.odom_sub = self.create_subscription(Odometry, '/filtered/odom', self.odom_callback, 10)
        # Timer callback now runs the Stanley controller periodically
        self.create_timer(0.01, self.timer_callback)
        self.robot_odom = Odometry()
        
        # Set up publishers
        self.steering_pub = self.create_publisher(Float64MultiArray, "/steering_controller/commands", 10)
        self.velocity_pub = self.create_publisher(Float64MultiArray, "/velocity_controller/commands", 10)
        self.path_pub = self.create_publisher(Path, "/path", 10)
        
        # Load the path from YAML into a NumPy array (each row: [x, y, yaw])
        self.path = self.read_path()
        self.path_index = 0  # pointer to the next target point
        self.get_logger().info(f"Loaded path with {self.path.shape[0]} points." if self.path is not None else "Path not loaded!")
        
        # Publish the full path as a nav_msgs/Path message
        self.publish_path()
        
        # Vehicle parameters (tune these as needed)
        self.wheel_radius = 0.3    # [m]
        self.l = 1.0               # parameter for "car" mode steering (wheelbase)
        self.track = 0.5           # [m], track width
        self.L = 1.0               # vehicle wheelbase used for front axle offset in Stanley control
        
        # Stanley controller parameters
        self.k = 5               # control gain (tune as needed)
        self.lastFoundIndex = 0    # pointer for target point (from Stanley algorithm)
        
        # State variables from odometry
        self.currentPos = [0.0, 0.0]       # current [x, y] position
        self.currentHeading = 0.0          # current heading in degrees
        
        # Speed setting (constant speed for Stanley control)
        self.speed = 2.0  # [m/s]

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
        # Set header: choose a frame (e.g., "map") and current time stamp.
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "world"
        
        # Convert each point into a PoseStamped message.
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
    
    def normalize_angle(self, angle):
        """
        Normalize an angle to the range [-pi, pi].
        """
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def stanley_controller_step(self, cx, cy, cyaw, currentPos, currentHeading, last_target_idx, speed):
        """
        Stanley controller step calculation.
        
        Parameters:
            cx, cy, cyaw: arrays of path x, y, and yaw values.
            currentPos: [x, y] current position.
            currentHeading: current heading in degrees.
            last_target_idx: index of the last target point.
            speed: current speed (m/s)
        
        Returns:
            delta: computed steering angle (rad)
            target_idx: updated target index
            error_front_axle: cross-track error
        """
        # Convert current heading to radians
        yaw = math.radians(currentHeading)
        # Compute front axle position
        fx = currentPos[0] + self.L * math.cos(yaw)
        fy = currentPos[1] + self.L * math.sin(yaw)
        
        # Compute distances from front axle to each point in the path
        dx = [fx - icx for icx in cx]
        dy = [fy - icy for icy in cy]
        d = np.hypot(dx, dy)
        target_idx = int(np.argmin(d))
        
        # Compute cross-track error by projecting error vector on a perpendicular vector
        error_x = cx[target_idx] - fx
        error_y = cy[target_idx] - fy
        # Perpendicular vector to vehicle heading: [-sin(yaw), cos(yaw)]
        error_front_axle = error_x * (-math.sin(yaw)) + error_y * math.cos(yaw)
        
        # Compute heading error between the path tangent (cyaw) and vehicle yaw
        theta_e = self.normalize_angle(cyaw[target_idx] - yaw)
        # Compute cross-track correction term
        theta_d = math.atan2(self.k * error_front_axle, speed)
        
        # Total steering command
        delta = theta_e + theta_d
        
        return delta, target_idx, error_front_axle

    def stanley_controller(self):
        """
        Computes the Stanley control action using current odometry and the loaded path.
        This method updates the vehicle’s current state, computes the appropriate steering angle,
        and commands the velocity and steering actuators.
        """
        # Extract the path components
        cx = self.path[:, 0]
        cy = self.path[:, 1]
        cyaw = self.path[:, 2]
        
        # Update current state from odometry (using a slight offset in x as in original node)
        x = self.robot_odom.pose.pose.position.x + 0.10
        y = self.robot_odom.pose.pose.position.y
        orientation_q = self.robot_odom.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.currentPos = [x, y]
        self.currentHeading = math.degrees(yaw)
        
        # Compute Stanley control to obtain the steering angle and target index
        steering_angle, target_idx, cross_track_error = self.stanley_controller_step(
            cx, cy, cyaw, self.currentPos, self.currentHeading, self.lastFoundIndex, self.speed)
        
        # Ensure target index does not go backward
        if target_idx < self.lastFoundIndex:
            target_idx = self.lastFoundIndex
        self.lastFoundIndex = target_idx
        
        self.get_logger().info(f"Steering Angle: {steering_angle:.2f} rad, TargetIdx: {target_idx}, Cross-Track Error: {cross_track_error:.2f}")
        
        # Command the vehicle with the computed steering angle and constant speed.
        self.cmd_vel_steering(self.speed, steering_angle)
    
    def cmd_vel_steering(self, vx, steering_angle):
        """
        Converts the desired steering angle and linear velocity into individual wheel commands.
        """
        mode = self.get_parameter("mode").get_parameter_value().string_value
        if vx == 0:
            self.set_steering_angle(0, 0)
            self.set_velocity(0, 0)
            return
        
        # Limit the steering angle (e.g., ±10°)
        angle_max = 10.0 / 180.0 * math.pi
        angle_min = -10.0 / 180.0 * math.pi
        steering_angle = np.clip(steering_angle, a_min=angle_min, a_max=angle_max)

        # Convert linear velocity to wheel speed using the wheel radius.
        wheel_speed = vx / self.wheel_radius
        
        if mode == "bicycle":
            left_angle = right_angle = steering_angle
        elif mode == "car":
            tan_steering = math.tan(steering_angle)
            left_angle = math.atan((self.l * tan_steering) / (self.l + 0.5 * self.track * tan_steering))
            right_angle = math.atan((self.l * tan_steering) / (self.l - 0.5 * self.track * tan_steering))
        else:
            self.get_logger().warn(f"Unknown mode: {mode}. Command ignored.")
            return
        
        # Publish steering and velocity commands
        self.set_steering_angle(left_angle, right_angle)
        self.set_velocity(wheel_speed, wheel_speed)
    
    def timer_callback(self):
        """
        Periodic callback that publishes the path (for visualization) and runs the Stanley controller.
        """
        self.publish_path()
        self.stanley_controller()

    def odom_callback(self, msg: Odometry):
        self.robot_odom = msg
        
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
    node = StanleyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
