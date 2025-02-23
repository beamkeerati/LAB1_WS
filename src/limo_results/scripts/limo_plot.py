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
        self.GT_x_data = deque(maxlen=2000) 
        self.GT_y_data = deque(maxlen=2000)
        self.GT_yaw_data = deque(maxlen=2000) 
        self.GT_linear_data = deque(maxlen=2000) 
        self.GT_angular_data = deque(maxlen=2000) 

        self.Yawrate_x_data = deque(maxlen=2000) 
        self.Yawrate_y_data = deque(maxlen=2000) 
        self.Yawrate_yaw_data = deque(maxlen=2000) 
        self.Yawrate_linear_data = deque(maxlen=2000) 
        self.Yawrate_angular_data = deque(maxlen=2000) 

        self.Single_x_data = deque(maxlen=2000) 
        self.Single_y_data = deque(maxlen=2000) 
        self.Single_yaw_data = deque(maxlen=2000) 
        self.Single_linear_data = deque(maxlen=2000) 
        self.Single_angular_data = deque(maxlen=2000) 
        
        self.Double_x_data = deque(maxlen=2000) 
        self.Double_y_data = deque(maxlen=2000) 
        self.Double_yaw_data = deque(maxlen=2000) 
        self.Double_linear_data = deque(maxlen=2000) 
        self.Double_angular_data = deque(maxlen=2000) 

        #--------------------------- Subscription ---------------------------#
        self.create_subscription(
            Odometry,
            '/odometry/ground_truth',
            self.odom_gt_callback,
            10
        )
        self.create_subscription(
            Odometry,
            '/yaw_rate/odom',
            self.odom_yawrate_callback,
            10
        )
        self.create_subscription(
            Odometry,
            '/single_track/odom',
            self.odom_single_callback,
            10
        )
        self.create_subscription(
            Odometry,
            '/double_track/odom',
            self.odom_double_callback,
            10
        )

        #--------------------------- Timer for Plotting ---------------------------#
        self.create_timer(0.1, self.plot_callback)  # Update plot every 0.1s

        self.fig, ((self.ax1, self.ax2), (self.ax3, self.ax4)) = plt.subplots(2, 2, figsize=(10, 8))
        plt.ion()

    def odom_gt_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        self.GT_x_data.append(x)
        self.GT_y_data.append(y)

        quat = msg.pose.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        self.GT_yaw_data.append(yaw)

        linear = msg.twist.twist.linear.x
        angular = msg.twist.twist.angular.z

        self.GT_linear_data.append(linear)
        self.GT_angular_data.append(angular)

        self.get_logger().info(f'Updated Ground Truth -> x: {x:.3f}, y: {y:.3f}, yaw: {yaw:.3f}, linear x: {linear:.3f}, angular z: {angular:.3f}')

    def odom_yawrate_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        self.Yawrate_x_data.append(x)
        self.Yawrate_y_data.append(y)

        quat = msg.pose.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        self.Yawrate_yaw_data.append(yaw)

        linear = msg.twist.twist.linear.x
        angular = msg.twist.twist.angular.z

        self.Yawrate_linear_data.append(linear)
        self.Yawrate_angular_data.append(angular)

        self.get_logger().info(f'Updated Yawrate -> x: {x:.3f}, y: {y:.3f}, yaw: {yaw:.3f}, linear x: {linear:.3f}, angular z: {angular:.3f}')

    def odom_single_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        self.Single_x_data.append(x)
        self.Single_y_data.append(y)

        quat = msg.pose.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        self.Single_yaw_data.append(yaw)

        linear = msg.twist.twist.linear.x
        angular = msg.twist.twist.angular.z

        self.Single_linear_data.append(linear)
        self.Single_angular_data.append(angular)

        self.get_logger().info(f'Updated Single -> x: {x:.3f}, y: {y:.3f}, yaw: {yaw:.3f}, linear x: {linear:.3f}, angular z: {angular:.3f}')

    def odom_double_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        self.Double_x_data.append(x)
        self.Double_y_data.append(y)

        quat = msg.pose.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        self.Double_yaw_data.append(yaw)

        linear = msg.twist.twist.linear.x
        angular = msg.twist.twist.angular.z

        self.Double_linear_data.append(linear)
        self.Double_angular_data.append(angular)

        self.get_logger().info(f'Updated Double -> x: {x:.3f}, y: {y:.3f}, yaw: {yaw:.3f}, linear x: {linear:.3f}, angular z: {angular:.3f}')

    def plot_callback(self):
        """Updates the plots in real-time using a ROS 2 timer."""
        # Update position plot (ax1)
        self.ax1.clear()
        self.ax1.plot(self.GT_x_data, self.GT_y_data, 'go-', label='Position Ground Truth', markersize=5)  # Green dots with lines
        self.ax1.plot(self.Yawrate_x_data, self.Yawrate_y_data, 'bo-', label='Position Yawrate', markersize=5)  # Blue dots with lines
        self.ax1.plot(self.Single_x_data, self.Single_y_data, 'mo-', label='Position Single Track', markersize=5)  # Purple (Magenta) dots with lines
        self.ax1.plot(self.Double_x_data, self.Double_y_data, 'ro-', label='Position Double Track', markersize=5)  # Red dots with lines
        self.ax1.set_xlabel('X Position')
        self.ax1.set_ylabel('Y Position')
        self.ax1.set_title('XY Limo Position')
        self.ax1.grid(True)
        self.ax1.legend()

        # Update yaw plot (ax2)
        self.ax2.clear()
        self.ax2.plot(self.GT_yaw_data, 'go-', label='Orientation Ground Truth', markersize=5)  # Green dots with lines
        self.ax2.plot(self.Yawrate_yaw_data, 'bo-', label='Orientation Yawrate', markersize=5)  # Blue dots with lines
        self.ax2.plot(self.Single_yaw_data, 'mo-', label='Orientation Single Track', markersize=5)  # Purple (Magenta) dots with lines
        self.ax2.plot(self.Double_yaw_data, 'ro-', label='Orientation Double Track', markersize=5)  # Red dots with lines
        self.ax2.set_xlabel('Time (s)')
        self.ax2.set_ylabel('Yaw (rad)')
        self.ax2.set_title('Yaw Orientation')
        self.ax2.grid(True)
        self.ax2.legend()

        # Update linear velocity plot (ax3)
        self.ax3.clear()
        self.ax3.plot(self.GT_linear_data, 'go-', label='Linear Velocity Ground Truth', markersize=5)
        self.ax3.plot(self.Yawrate_linear_data, 'bo-', label='Linear Velocity Yawrate', markersize=5)
        self.ax3.plot(self.Single_linear_data, 'mo-', label='Linear Velocity Single Track', markersize=5)
        self.ax3.plot(self.Double_linear_data, 'ro-', label='Linear Velocity Double Track', markersize=5)
        self.ax3.set_xlabel('Time (s)')
        self.ax3.set_ylabel('Linear Velocity (m/s)')
        self.ax3.set_title('Linear Velocity')
        self.ax3.grid(True)
        self.ax3.legend()

        # Update angular velocity plot (ax4)
        self.ax4.clear()
        self.ax4.plot(self.GT_angular_data, 'go-', label='Angular Velocity Ground Truth', markersize=5)
        self.ax4.plot(self.Yawrate_angular_data, 'bo-', label='Angular Velocity Yawrate', markersize=5)
        self.ax4.plot(self.Single_angular_data, 'mo-', label='Angular Velocity Single Track', markersize=5)
        self.ax4.plot(self.Double_angular_data, 'ro-', label='Angular Velocity Double Track', markersize=5)
        self.ax4.set_xlabel('Time (s)')
        self.ax4.set_ylabel('Angular Velocity (rad/s)')
        self.ax4.set_title('Angular Velocity')
        self.ax4.grid(True)
        self.ax4.legend()

        # Draw and pause to update the figure
        plt.draw()
        plt.pause(0.001)  # Brief pause to update the figures

def main(args=None):
    rclpy.init(args=args)
    node = LimoPlotNode()
    rclpy.spin(node)  # ROS 2 event loop
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
