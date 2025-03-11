#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from collections import deque
import tf_transformations

class LimoPlotNode(Node):
    def __init__(self):
        super().__init__('limo_plot_node')

        #--------------------------- Variables ---------------------------#
        self.GT_x_data = deque(maxlen=20000) 
        self.GT_y_data = deque(maxlen=20000)
        self.Yawrate_x_data = deque(maxlen=2000) 
        self.Yawrate_y_data = deque(maxlen=2000) 
        self.Single_x_data = deque(maxlen=2000) 
        self.Single_y_data = deque(maxlen=2000) 
        self.Double_x_data = deque(maxlen=2000) 
        self.Double_y_data = deque(maxlen=2000) 
        self.EKF_x_data = deque(maxlen=20000) 
        self.EKF_y_data = deque(maxlen=20000)

        #--------------------------- Subscriptions ---------------------------#
        self.create_subscription(Odometry, '/odometry/ground_truth', self.odom_gt_callback, 10)
        self.create_subscription(Odometry, '/yaw_rate/odom', self.odom_yawrate_callback, 10)
        self.create_subscription(Odometry, '/single_track/odom', self.odom_single_callback, 10)
        self.create_subscription(Odometry, '/double_track/odom', self.odom_double_callback, 10)
        self.create_subscription(Odometry, '/filtered/odom', self.odom_ekf_callback, 10)

        #--------------------------- Timer for Plotting ---------------------------#
        self.create_timer(0.1, self.plot_callback)
        self.fig, self.ax1 = plt.subplots(figsize=(8, 6))
        plt.ion()

    def odom_gt_callback(self, msg: Odometry):
        self.GT_x_data.append(msg.pose.pose.position.x)
        self.GT_y_data.append(msg.pose.pose.position.y)
    
    def odom_yawrate_callback(self, msg: Odometry):
        self.Yawrate_x_data.append(msg.pose.pose.position.x)
        self.Yawrate_y_data.append(msg.pose.pose.position.y)

    def odom_single_callback(self, msg: Odometry):
        self.Single_x_data.append(msg.pose.pose.position.x)
        self.Single_y_data.append(msg.pose.pose.position.y)
    
    def odom_double_callback(self, msg: Odometry):
        self.Double_x_data.append(msg.pose.pose.position.x)
        self.Double_y_data.append(msg.pose.pose.position.y)
    
    def odom_ekf_callback(self, msg: Odometry):
        self.EKF_x_data.append(msg.pose.pose.position.x)
        self.EKF_y_data.append(msg.pose.pose.position.y)
    
    def plot_callback(self):
        """Updates the position plot in real-time."""
        self.ax1.clear()
        self.ax1.plot(self.GT_x_data, self.GT_y_data, 'go-', label='Ground Truth', markersize=5)
        self.ax1.plot(self.Yawrate_x_data, self.Yawrate_y_data, 'bo-', label='Yawrate', markersize=5)
        self.ax1.plot(self.Single_x_data, self.Single_y_data, 'mo-', label='Single Track', markersize=5)
        self.ax1.plot(self.Double_x_data, self.Double_y_data, 'ro-', label='Double Track', markersize=5)
        self.ax1.plot(self.EKF_x_data, self.EKF_y_data, 'ko-', label='EKF', markersize=5)
        
        self.ax1.set_xlabel('X Position')
        self.ax1.set_ylabel('Y Position')
        self.ax1.set_title('XY Limo Position')
        self.ax1.grid(True)
        self.ax1.legend()
        
        plt.draw()
        plt.pause(0.001)


def main(args=None):
    rclpy.init(args=args)
    node = LimoPlotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
