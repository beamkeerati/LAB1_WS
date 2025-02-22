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
from std_msgs.msg import Float64
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import tf_transformations
import math
import numpy as np 


class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odometry/ground_truth', self.odom_callback, 10)
        
        self.create_timer(0.01, self.timer_callback)
        
        self.robot_odom = Odometry()
        
    def pure_pursuit(self,x_tp,y_tp ):
        # Compute the lookahead point
        x = self.robot_odom.pose.pose.position.x - 0.1
        y = self.robot_odom.pose.pose.position.y
        K_dd = 0.5
        min_ld = 0.1
        max_ld = 1.0
        speed = 0.5
        
        l_d = np.clip(K_dd * speed, min_ld, max_ld)

        
    def cmd_vel(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_vel_pub.publish(msg)
        
    def timer_callback(self):
        
        pass
    
    def odom_callback(self, msg: Odometry):
        self.robot_odom = msg


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
