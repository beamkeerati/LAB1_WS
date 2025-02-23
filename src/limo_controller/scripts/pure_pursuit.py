#!/usr/bin/python3
from dataclasses import dataclass
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import (
    Point, Pose, PoseWithCovariance, Quaternion, Twist, TwistWithCovariance,
    Vector3, TransformStamped
)
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Float64MultiArray
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import tf_transformations
import math
import numpy as np
import os
import yaml  
from ament_index_python.packages import get_package_share_directory

class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')
        
        self.declare_parameter("mode", "car")
        self.mode = self.get_parameter("mode").get_parameter_value().string_value

        self.odom_sub = self.create_subscription(Odometry, '/odometry/ground_truth', self.odom_callback, 10)
        self.create_timer(0.01, self.timer_callback)
        self.robot_odom = Odometry()
        
        self.steering_pub = self.create_publisher(Float64MultiArray, "/steering_controller/commands", 10)
        self.velocity_pub = self.create_publisher(Float64MultiArray, "/velocity_controller/commands", 10)
        
        self.yaml_lines = []
        self.read_path()


    def pure_pursuit(self, x_tp, y_tp):
        # Compute the lookahead point
        x = self.robot_odom.pose.pose.position.x - 0.1
        y = self.robot_odom.pose.pose.position.y
        K_dd = 0.5
        min_ld = 0.1
        max_ld = 1.0
        speed = 0.5
        
        l_d = np.clip(K_dd * speed, min_ld, max_ld)
        # Additional pure pursuit logic would go here

        
    def timer_callback(self):
        # Timer callback functionality (if any) would be implemented here
        pass
    
    def cmd_vel_steering(self, vx, steering_angle):
        # Retrieve control mode parameter
        mode = self.get_parameter("mode").get_parameter_value().string_value

        # If there's no forward motion, stop steering and velocity.
        if vx == 0:
            self.set_steering_angle(0, 0)
            self.set_velocity(0, 0)
            return

        # Compute wheel speed common to both modes
        wheel_speed = vx /self.wheel_radius

        # Compute the basic steering angle based on the input twist message
        base_steering_angle = steering_angle

        if mode == "bicycle":
            # In bicycle mode, both steering joints follow the same angle.
            left_angle = right_angle = base_steering_angle

        elif mode == "car":
            # In car mode, calculate individual wheel angles.
            tan_steering = math.tan(base_steering_angle)
            left_angle = math.atan((self.l * tan_steering) / (self.l + 0.5 * self.track * tan_steering))
            right_angle = math.atan((self.l * tan_steering) / (self.l - 0.5 * self.track * tan_steering))
            
        else:
            self.get_logger().warn(f"Unknown mode: {mode}. Ignoring command.")
            return

        # Set computed steering angles and wheel speeds
        self.set_steering_angle(left_angle, right_angle)
        self.set_velocity(wheel_speed, wheel_speed)
    
    def odom_callback(self, msg: Odometry):
        self.robot_odom = msg
        
    
    def set_steering_angle(self, left_steering_hinge_wheel, right_steering_hinge_wheel):
        msg = Float64MultiArray()
        msg.layout.data_offset = 0
        msg.data = [float(left_steering_hinge_wheel), float(right_steering_hinge_wheel)]
        self.steering_pub.publish(msg)

    def set_velocity(self, rear_left_wheel, rear_right_wheel):
        msg = Float64MultiArray()
        msg.layout.data_offset = 0
        msg.data = [float(rear_left_wheel), float(rear_right_wheel)]
        self.velocity_pub.publish(msg)
        
    def read_path(self):
        # Retrieve the package share directory for 'limo_controller'
        pkg_limo_controller = get_package_share_directory('limo_controller')
        # Build the path to the YAML file
        yaml_path = os.path.join(pkg_limo_controller, 'path', 'path.yaml')
        
        # Read the YAML file row by row
        self.yaml_lines = []
        try:
            with open(yaml_path, 'r') as file:
                for line in file:
                    # Strip any extraneous whitespace/newlines
                    clean_line = line.strip()
                    self.yaml_lines.append(clean_line)
                    # Log each line for debugging purposes
                    self.get_logger().info(f"YAML line: {clean_line}")
        except Exception as e:
            self.get_logger().error(f"Failed to read YAML file: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
