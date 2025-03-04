#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from nav_msgs.msg import Odometry
import random

class GpsOdomNode(Node):
    def __init__(self):
        super().__init__('gps_odom_node')
        # 1. Create a new publisher for the /gps/odom topic
        self.gps_publisher = self.create_publisher(Odometry, '/gps/odom', 10)
        # 2. Subscribe to the /odometry/ground_truth topic
        self.subscription = self.create_subscription(
            Odometry,
            '/odometry/ground_truth',
            self.odom_callback,
            10
        )
        # Declare ROS parameters for noise mean and standard deviation
        self.declare_parameter('noise_mean', 0.0)
        self.declare_parameter('noise_stddev', 0.05)
        # Initialize internal variables with parameter values
        self.noise_mean = float(self.get_parameter('noise_mean').value)
        self.noise_stddev = float(self.get_parameter('noise_stddev').value)
        # Set up a callback to update parameters dynamically
        self.add_on_set_parameters_callback(self.parameter_callback)
        self.get_logger().info(f"Initialized GPS noise parameters: mean={self.noise_mean}, stddev={self.noise_stddev}")

    def odom_callback(self, msg: Odometry):
        """Handle incoming ground truth odometry, add noise, and publish as GPS odometry."""
        # 3. Add Gaussian noise to the GPS position data
        noisy_odom = Odometry()
        noisy_odom.header = msg.header            # Copy header (time stamp and frame id)
        noisy_odom.child_frame_id = msg.child_frame_id  # Copy child frame (e.g., base_link)
        # Copy pose orientation directly (assuming GPS doesn't affect orientation)
        noisy_odom.pose.pose.orientation = msg.pose.pose.orientation
        # Add Gaussian noise to position
        noisy_odom.pose.pose.position.x = msg.pose.pose.position.x + random.gauss(self.noise_mean, self.noise_stddev)
        noisy_odom.pose.pose.position.y = msg.pose.pose.position.y + random.gauss(self.noise_mean, self.noise_stddev)
        noisy_odom.pose.pose.position.z = msg.pose.pose.position.z + random.gauss(self.noise_mean, self.noise_stddev)
        # Copy twist (velocity) data as-is
        noisy_odom.twist = msg.twist
        # Publish the noisy odometry as a simulated GPS output
        self.gps_publisher.publish(noisy_odom)

    def parameter_callback(self, params):
        """Callback to handle updates to parameters (noise_mean, noise_stddev) at runtime."""
        for param in params:
            if param.name == 'noise_mean' and param.type_ == Parameter.Type.DOUBLE:
                self.noise_mean = float(param.value)
                self.get_logger().info(f"Updated noise_mean to {self.noise_mean}")
            elif param.name == 'noise_stddev' and param.type_ == Parameter.Type.DOUBLE:
                self.noise_stddev = float(param.value)
                self.get_logger().info(f"Updated noise_stddev to {self.noise_stddev}")
        # Return success to allow parameter update
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = GpsOdomNode()
    rclpy.spin(node)
    # Clean up and shutdown
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
