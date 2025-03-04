#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
import math

class EKFNode(Node):
    def __init__(self):
        super().__init__('ekf_node')
        # Declare ROS2 parameters for configurability
        self.declare_parameter('publish_frequency', 100.0)  # 100 Hz default
        # Process noise (std dev for x, y, yaw, v)
        self.declare_parameter('process_noise', [1.0, 1.0, math.radians(1.0), 1.0])
        # Measurement noise (std dev) for each sensor source:
        self.declare_parameter('R_double_track', [100.0, 100.0, math.radians(500.0)])
        self.declare_parameter('R_single_track', [100.0, 100.0, math.radians(500.0)])
        self.declare_parameter('R_yaw_rate', math.radians(1.0))
        self.declare_parameter('R_gps', [1.0, 1.0])
        
        # Get and configure parameter values
        freq = self.get_parameter('publish_frequency').value
        # Process noise covariance Q
        proc_noise = np.array(self.get_parameter('process_noise').value) ** 2  # square to variance
        self.Q = np.diag(proc_noise)
        # Measurement noise covariance for each sensor
        double_noise = np.array(self.get_parameter('R_double_track').value) ** 2
        single_noise = np.array(self.get_parameter('R_single_track').value) ** 2
        yaw_noise    = float(self.get_parameter('R_yaw_rate').value) ** 2
        gps_noise    = np.array(self.get_parameter('R_gps').value) ** 2
        self.R_double = np.diag(double_noise)            # 3x3 covariance for [x, y, yaw]
        self.R_single = np.diag(single_noise)            # 3x3 covariance for [x, y, yaw]
        self.R_yaw   = np.array([[yaw_noise]], dtype=float)  # 1x1 covariance for yaw
        self.R_gps   = np.diag(gps_noise)                # 2x2 covariance for [x, y]
        
        # Initialize state [x, y, yaw, v] and its covariance P
        self.x = np.zeros((4, 1))
        self.P = np.diag([1.0, 1.0, math.radians(1.0), 1.0])  # large initial uncertainty in yaw
        
        # Subscribers for each odometry input
        self.create_subscription(Odometry, '/double_track/odom', self.double_odom_callback, 10)
        self.create_subscription(Odometry, '/single_track/odom', self.single_odom_callback, 10)
        self.create_subscription(Odometry, '/yaw_rate/odom',   self.yaw_odom_callback, 10)
        self.create_subscription(Odometry, '/gps/odom',        self.gps_odom_callback, 10)
        
        # Publisher for the fused/filtered odometry
        self.pub_filtered = self.create_publisher(Odometry, '/filtered/odom', 10)
        
        # Timer for periodic EKF prediction and publishing (default 100 Hz)
        self.timer = self.create_timer(1.0 / float(freq), self.timer_callback)
        
        # Store last known control inputs (linear velocity and yaw rate)
        self.last_odom_v = 0.0       # last linear velocity (m/s)
        self.last_odom_yawrate = 0.0  # last yaw rate (rad/s)
        
        # Frames (could also be parameters if needed)
        self.output_frame = 'world'
        self.base_frame   = 'base_footprint'
        
        self.get_logger().info(f"Initialized EKF node with {freq} Hz, process noise {proc_noise}, and measurement noise.")
    
    def motion_model(self, x, u, dt):
        """Predict the next state given current state x and control input u = [v; yaw_rate] over time dt."""
        # State transition model for [x, y, yaw, v]:
        # x_k+1 = x_k + v*dt*cos(yaw)
        # y_k+1 = y_k + v*dt*sin(yaw)
        # yaw_k+1 = yaw_k + yaw_rate*dt
        # v_k+1 = v_k  (assuming constant velocity during dt)
        F = np.eye(4)
        F[3, 3] = 0.0  # velocity is replaced by control, so not carried over directly
        B = np.array([
            [dt * math.cos(x[2, 0]),  0.0],
            [dt * math.sin(x[2, 0]),  0.0],
            [0.0,                    dt ],
            [1.0,                    0.0]
        ])
        x_pred = F @ x + B @ u
        return x_pred
    
    def update_ekf(self, z, H, R):
        """EKF measurement update: update state x and covariance P with measurement z."""
        z = np.array(z, dtype=float).reshape(-1, 1)
        z_pred = H @ self.x
        y = z - z_pred  # innovation
        
        # If yaw is part of the measurement, normalize the yaw innovation to [-pi, pi]
        # Handle both cases: a 3D pose measurement [x, y, yaw] or a single yaw measurement
        if R.shape[0] == 3 and z.shape[0] == 3:  # [x, y, yaw] measurement
            # Normalize yaw difference
            y[2, 0] = math.atan2(math.sin(y[2, 0]), math.cos(y[2, 0]))
        elif R.shape[0] == 1 and z.shape[0] == 1:
            # Only yaw measurement
            y[0, 0] = math.atan2(math.sin(y[0, 0]), math.cos(y[0, 0]))
        
        # EKF equations
        S = H @ self.P @ H.T + R                # residual covariance
        K = self.P @ H.T @ np.linalg.inv(S)     # Kalman gain
        self.x = self.x + K @ y                 # state update
        # Normalize yaw angle after update
        self.x[2, 0] = math.atan2(math.sin(self.x[2, 0]), math.cos(self.x[2, 0]))
        I = np.eye(len(self.x))
        self.P = (I - K @ H) @ self.P           # covariance update
    
    # Callback for double track odometry (provides x, y, yaw, and velocities)
    def double_odom_callback(self, msg: Odometry):
        # Extract pose (x, y, yaw) from message
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        # Convert quaternion to yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        # Extract linear and angular velocity (in base_link frame)
        v = msg.twist.twist.linear.x
        yaw_rate = msg.twist.twist.angular.z
        # Update stored control input for prediction step
        self.last_odom_v = float(v)
        self.last_odom_yawrate = float(yaw_rate)
        # Measurement vector and measurement function for [x, y, yaw]
        z = [x, y, yaw]
        H = np.array([[1, 0, 0, 0],
                      [0, 1, 0, 0],
                      [0, 0, 1, 0]], dtype=float)
        # EKF update with double track odometry
        # self.update_ekf(z, H, self.R_double)
    
    # Callback for single track odometry (provides x, y, yaw; assume velocity is similar or already accounted)
    def single_odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        # Quaternion to yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        # (Optionally, could also use msg.twist.twist if needed for v or yaw_rate)
        z = [x, y, yaw]
        H = np.array([[1, 0, 0, 0],
                      [0, 1, 0, 0],
                      [0, 0, 1, 0]], dtype=float)
        self.update_ekf(z, H, self.R_single)
    
    # Callback for yaw rate odometry (e.g., from IMU)
    def yaw_odom_callback(self, msg: Odometry):
        # Extract yaw from orientation (we assume other pose components might be zero/ignored)
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        # Only yaw measurement
        z = [yaw]
        H = np.array([[0, 0, 1, 0]], dtype=float)  # measurement model: observe yaw from state
        self.update_ekf(z, H, self.R_yaw)
        # If the yaw odometry also provides angular velocity (e.g., IMU gyro), use it for prediction
        if abs(msg.twist.twist.angular.z) > 1e-6:
            self.last_odom_yawrate = float(msg.twist.twist.angular.z)
    
    # Callback for GPS odometry (provides x, y position)
    def gps_odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z_meas = [x, y]
        H = np.array([[1, 0, 0, 0],
                      [0, 1, 0, 0]], dtype=float)  # observe x and y from state
        self.update_ekf(z_meas, H, self.R_gps)
    
    # Timer callback for EKF prediction and publishing filtered odometry
    def timer_callback(self):
        # Time step for prediction
        dt = 1.0 / float(self.get_parameter('publish_frequency').value)
        # Control input vector [v; yaw_rate]
        u = np.array([[self.last_odom_v], [self.last_odom_yawrate]], dtype=float)
        # EKF Prediction step: project state and covariance forward
        self.x = self.motion_model(self.x, u, dt)
        # Compute Jacobian of motion model F for covariance update
        v = self.last_odom_v
        yaw = self.x[2, 0]
        F_jac = np.array([
            [1, 0, -v * dt * math.sin(yaw), dt * math.cos(yaw)],
            [0, 1,  v * dt * math.cos(yaw), dt * math.sin(yaw)],
            [0, 0, 1,                        0],
            [0, 0, 0,                        0]
        ], dtype=float)
        # Update covariance with process noise
        self.P = F_jac @ self.P @ F_jac.T + self.Q
        
        # Prepare and publish the filtered Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.output_frame   # e.g., "odom"
        odom_msg.child_frame_id = self.base_frame      # e.g., "base_link"
        # State [x, y, yaw] into pose
        odom_msg.pose.pose.position.x = float(self.x[0, 0])
        odom_msg.pose.pose.position.y = float(self.x[1, 0])
        odom_msg.pose.pose.position.z = 0.0  # assuming ground plane
        # Convert yaw back to quaternion for orientation
        half_yaw = self.x[2, 0] * 0.5
        odom_msg.pose.pose.orientation.z = math.sin(half_yaw)
        odom_msg.pose.pose.orientation.w = math.cos(half_yaw)
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        
        # Fill pose covariance (only x, y, yaw relevant). 
        # Indices: [0]=var(x), [7]=var(y), [35]=var(yaw). Off-diagonals for x-y, x-yaw, y-yaw also set.
        cov = [0.0] * 36
        cov[0]  = float(self.P[0, 0]); cov[1]  = float(self.P[0, 1]); cov[5]  = float(self.P[0, 2])
        cov[6]  = float(self.P[1, 0]); cov[7]  = float(self.P[1, 1]); cov[11] = float(self.P[1, 2])
        cov[30] = float(self.P[2, 0]); cov[31] = float(self.P[2, 1]); cov[35] = float(self.P[2, 2])
        # For unobserved dimensions (z, roll, pitch), use large covariance to denote ignorance
        cov[14] = 1e6; cov[21] = 1e6; cov[28] = 1e6
        odom_msg.pose.covariance = cov
        
        # For twist, we output the latest known velocity in the robot frame (base_link)
        odom_msg.twist.twist.linear.x = float(self.last_odom_v)
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.z = float(self.last_odom_yawrate)
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        # (Twist covariance could be filled if available; here left as unknown)
        odom_msg.twist.covariance = tuple([0.0] * 36)
        
        # Publish the fused odometry
        self.pub_filtered.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = EKFNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()