#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from collections import deque

class LimoPlotNode(Node):
    def __init__(self):
        super().__init__('limo_plot_node')

        #--------------------------- Variables ---------------------------#
        self.GT_x_data = deque(maxlen=2000) 
        self.GT_y_data = deque(maxlen=2000)

        self.GPS_x_data = deque(maxlen=2000) 
        self.GPS_y_data = deque(maxlen=2000) 

        #--------------------------- Subscription ---------------------------#
        self.create_subscription(Odometry, '/odometry/ground_truth', self.odom_gt_callback, 10)
        self.create_subscription(Odometry, '/gps/odom', self.odom_gps_callback, 10)

        #--------------------------- Timer for Plotting ---------------------------#
        self.create_timer(0.1, self.plot_callback)  # Update plot every 0.1s

        self.fig, self.ax = plt.subplots(figsize=(8, 6))
        plt.ion()

    def odom_gt_callback(self, msg: Odometry):
        self.GT_x_data.append(msg.pose.pose.position.x)
        self.GT_y_data.append(msg.pose.pose.position.y)

    def odom_gps_callback(self, msg: Odometry):
        self.GPS_x_data.append(msg.pose.pose.position.x)
        self.GPS_y_data.append(msg.pose.pose.position.y)

    def plot_callback(self):
        """Updates the plot in real-time using a ROS 2 timer."""
        self.ax.clear()
        self.ax.plot(self.GT_x_data, self.GT_y_data, 'go-', label='Ground Truth', markersize=5)  # Green line
        self.ax.scatter(self.GPS_x_data, self.GPS_y_data, c='b', label='GPS', s=10)  # Blue dots only
        self.ax.set_xlabel('X Position')
        self.ax.set_ylabel('Y Position')
        self.ax.set_title('Real-time Position Plot')
        self.ax.grid(True)
        self.ax.legend()
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
