#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from tf_transformations import euler_from_quaternion
import math
import numpy as np
import yaml
import os
from datetime import datetime
from ament_index_python.packages import get_package_share_directory


class PathTrackingMeasurement(Node):
    def __init__(self):
        super().__init__("path_tracking_measurement")

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, "/odometry/ground_truth", self.odom_callback, 10
        )

        self.reference_sub = self.create_subscription(
            PoseStamped, "/mpc/reference_pose", self.reference_callback, 10
        )

        # Publishers for real-time tracking metrics
        self.tracking_error_pub = self.create_publisher(
            Float64MultiArray, "/tracking/errors", 10
        )

        # Timer for periodic data saving and statistics
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1 Hz for statistics

        # Data storage
        self.tracking_data = []
        self.current_odom = None
        self.current_reference = None

        # Statistics
        self.error_history = {
            "lateral_error": [],
            "heading_error": [],
            "speed_error": [],
            "position_error": [],
        }

        # Parameters
        self.declare_parameter("save_interval", 10.0)  # Save every 10 seconds
        self.declare_parameter("max_data_points", 10000)  # Maximum data points to store
        self.declare_parameter("output_file", "tracking_results.yaml")

        self.save_interval = (
            self.get_parameter("save_interval").get_parameter_value().double_value
        )
        self.max_data_points = (
            self.get_parameter("max_data_points").get_parameter_value().integer_value
        )
        self.output_file = (
            self.get_parameter("output_file").get_parameter_value().string_value
        )

        # Initialize save timer
        self.save_timer = self.create_timer(self.save_interval, self.save_data_callback)

        # Setup output directory
        try:
            pkg_share = get_package_share_directory("limo_controller")
            self.output_dir = os.path.join(pkg_share, "tracking_results")
        except:
            self.output_dir = os.path.join(
                os.path.expanduser("~"), "limo_tracking_results"
            )

        # Create output directory if it doesn't exist
        os.makedirs(self.output_dir, exist_ok=True)

        self.get_logger().info(f"Path tracking measurement node initialized")
        self.get_logger().info(f"Output directory: {self.output_dir}")
        self.get_logger().info(f"Data will be saved every {self.save_interval} seconds")

    def odom_callback(self, msg: Odometry):
        """Store current odometry data"""
        self.current_odom = msg
        self.process_tracking_data()

    def reference_callback(self, msg: PoseStamped):
        """Store current reference pose data"""
        self.current_reference = msg
        self.process_tracking_data()

    def process_tracking_data(self):
        """Process tracking data when both odometry and reference are available"""
        if self.current_odom is None or self.current_reference is None:
            return

        # Extract current pose
        current_x = self.current_odom.pose.pose.position.x
        current_y = self.current_odom.pose.pose.position.y
        current_orientation = self.current_odom.pose.pose.orientation
        _, _, current_yaw = euler_from_quaternion(
            [
                current_orientation.x,
                current_orientation.y,
                current_orientation.z,
                current_orientation.w,
            ]
        )
        current_speed = math.sqrt(
            self.current_odom.twist.twist.linear.x**2
            + self.current_odom.twist.twist.linear.y**2
        )

        # Extract reference pose
        ref_x = self.current_reference.pose.position.x
        ref_y = self.current_reference.pose.position.y
        ref_orientation = self.current_reference.pose.orientation
        _, _, ref_yaw = euler_from_quaternion(
            [ref_orientation.x, ref_orientation.y, ref_orientation.z, ref_orientation.w]
        )

        # Calculate tracking errors
        errors = self.calculate_tracking_errors(
            current_x, current_y, current_yaw, current_speed, ref_x, ref_y, ref_yaw
        )

        # Store data point
        timestamp = self.get_clock().now().to_msg()
        data_point = {
            "timestamp": {"sec": timestamp.sec, "nanosec": timestamp.nanosec},
            "actual_pose": {
                "x": float(current_x),
                "y": float(current_y),
                "yaw": float(current_yaw),
                "speed": float(current_speed),
            },
            "reference_pose": {
                "x": float(ref_x),
                "y": float(ref_y),
                "yaw": float(ref_yaw),
            },
            "tracking_errors": errors,
        }

        self.tracking_data.append(data_point)

        # Limit data storage
        if len(self.tracking_data) > self.max_data_points:
            self.tracking_data = self.tracking_data[-self.max_data_points :]

        # Update error history for statistics
        self.error_history["lateral_error"].append(errors["lateral_error"])
        self.error_history["heading_error"].append(errors["heading_error"])
        self.error_history["speed_error"].append(errors["speed_error"])
        self.error_history["position_error"].append(errors["position_error"])

        # Limit history size
        max_history = 1000
        for key in self.error_history:
            if len(self.error_history[key]) > max_history:
                self.error_history[key] = self.error_history[key][-max_history:]

        # Publish real-time errors
        self.publish_tracking_errors(errors)

    def calculate_tracking_errors(
        self, curr_x, curr_y, curr_yaw, curr_speed, ref_x, ref_y, ref_yaw
    ):
        """Calculate various tracking errors"""

        # Position error (Euclidean distance)
        position_error = math.sqrt((curr_x - ref_x) ** 2 + (curr_y - ref_y) ** 2)

        # Lateral error (cross-track error)
        # Vector from reference to current position
        dx = curr_x - ref_x
        dy = curr_y - ref_y

        # Project onto normal to reference direction
        lateral_error = abs(-dx * math.sin(ref_yaw) + dy * math.cos(ref_yaw))

        # Heading error
        heading_error = self.normalize_angle(curr_yaw - ref_yaw)

        # Speed error (we don't have reference speed, so use 0 as baseline)
        speed_error = abs(curr_speed - 0.8)  # Assuming target speed is 0.8 m/s

        # Along-track error (distance along the path direction)
        along_track_error = dx * math.cos(ref_yaw) + dy * math.sin(ref_yaw)

        return {
            "position_error": float(position_error),
            "lateral_error": float(lateral_error),
            "heading_error": float(abs(heading_error)),
            "speed_error": float(speed_error),
            "along_track_error": float(along_track_error),
            "heading_error_signed": float(heading_error),
        }

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def publish_tracking_errors(self, errors):
        """Publish current tracking errors for real-time monitoring"""
        msg = Float64MultiArray()
        msg.data = [
            errors["position_error"],
            errors["lateral_error"],
            errors["heading_error"],
            errors["speed_error"],
            errors["along_track_error"],
        ]
        self.tracking_error_pub.publish(msg)

    def calculate_statistics(self):
        """Calculate tracking performance statistics"""
        if not self.error_history["lateral_error"]:
            return {}

        stats = {}
        for error_type, values in self.error_history.items():
            if values:
                stats[error_type] = {
                    "mean": float(np.mean(values)),
                    "std": float(np.std(values)),
                    "max": float(np.max(values)),
                    "min": float(np.min(values)),
                    "rms": float(np.sqrt(np.mean(np.square(values)))),
                }

        return stats

    def timer_callback(self):
        """Periodic callback for logging statistics"""
        if len(self.tracking_data) > 0:
            stats = self.calculate_statistics()

            # Log current performance
            if "lateral_error" in stats:
                lateral_rms = stats["lateral_error"]["rms"]
                heading_rms = stats["heading_error"]["rms"]
                position_rms = stats["position_error"]["rms"]

                self.get_logger().info(
                    f"Tracking Performance - "
                    f"Lateral RMS: {lateral_rms:.3f}m, "
                    f"Heading RMS: {math.degrees(heading_rms):.1f}deg, "
                    f"Position RMS: {position_rms:.3f}m, "
                    f"Data points: {len(self.tracking_data)}"
                )

    def save_data_callback(self):
        """Periodic callback to save data"""
        if len(self.tracking_data) > 0:
            self.save_data_to_yaml()

    def save_data_to_yaml(self):
        """Save tracking data to YAML file"""
        if not self.tracking_data:
            return

        # Generate filename with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"tracking_data_{timestamp}.yaml"
        filepath = os.path.join(self.output_dir, filename)

        # Calculate final statistics
        stats = self.calculate_statistics()

        # Prepare data for saving
        output_data = {
            "metadata": {
                "total_data_points": len(self.tracking_data),
                "measurement_duration_sec": len(self.tracking_data)
                * 0.05,  # Approximate
                "generated_at": timestamp,
                "node_name": self.get_name(),
            },
            "statistics": stats,
            "tracking_data": self.tracking_data[
                -100:
            ],  # Save last 100 points to avoid huge files
        }

        try:
            with open(filepath, "w") as file:
                yaml.dump(output_data, file, default_flow_style=False, indent=2)

            self.get_logger().info(f"Tracking data saved to: {filepath}")

            # Also save a "latest" file for easy access
            latest_filepath = os.path.join(self.output_dir, "latest_tracking_data.yaml")
            with open(latest_filepath, "w") as file:
                yaml.dump(output_data, file, default_flow_style=False, indent=2)

        except Exception as e:
            self.get_logger().error(f"Failed to save tracking data: {e}")

    def save_final_results(self):
        """Save comprehensive results when node shuts down"""
        if not self.tracking_data:
            return

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"final_tracking_results_{timestamp}.yaml"
        filepath = os.path.join(self.output_dir, filename)

        stats = self.calculate_statistics()

        # Comprehensive output
        output_data = {
            "experiment_summary": {
                "total_data_points": len(self.tracking_data),
                "measurement_duration_sec": len(self.tracking_data) * 0.05,
                "completed_at": timestamp,
            },
            "performance_statistics": stats,
            "mpc_parameters": {
                "note": "MPC parameters used during this experiment",
                "horizon_length": 15,
                "control_dt": 0.05,
                "target_speed": 0.8,
                "position_weight": 10.0,
                "yaw_weight": 5.0,
            },
            "full_tracking_data": self.tracking_data,
        }

        try:
            with open(filepath, "w") as file:
                yaml.dump(output_data, file, default_flow_style=False, indent=2)

            self.get_logger().info(f"Final tracking results saved to: {filepath}")

            # Print summary to console
            if "lateral_error" in stats:
                self.get_logger().info("=== FINAL TRACKING PERFORMANCE SUMMARY ===")
                self.get_logger().info(
                    f"Lateral Error - RMS: {stats['lateral_error']['rms']:.3f}m, Max: {stats['lateral_error']['max']:.3f}m"
                )
                self.get_logger().info(
                    f"Heading Error - RMS: {math.degrees(stats['heading_error']['rms']):.1f}deg, Max: {math.degrees(stats['heading_error']['max']):.1f}deg"
                )
                self.get_logger().info(
                    f"Position Error - RMS: {stats['position_error']['rms']:.3f}m, Max: {stats['position_error']['max']:.3f}m"
                )
                self.get_logger().info(
                    "==============================================="
                )

        except Exception as e:
            self.get_logger().error(f"Failed to save final results: {e}")


def main(args=None):
    rclpy.init(args=args)

    node = PathTrackingMeasurement()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down tracking measurement node...")
    finally:
        # Save final results before shutdown
        node.save_final_results()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
