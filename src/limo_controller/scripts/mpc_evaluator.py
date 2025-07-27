#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterType
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from datetime import datetime
import os
import yaml
import json
import time
import math
from collections import deque
from threading import Lock

# ROS message imports
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64, String
from sensor_msgs.msg import JointState

# Transform utilities
from tf_transformations import euler_from_quaternion


class MPCPerformanceEvaluator(Node):
    """
    Comprehensive MPC Controller Performance Evaluation Node
    
    This node evaluates MPC controller performance using metrics adapted from 
    odometry evaluation literature, including:
    - Position tracking error
    - Heading alignment error  
    - Path following accuracy
    - Control smoothness
    - Computational performance
    """
    
    def __init__(self):
        super().__init__('mpc_performance_evaluator')
        
        # Evaluation parameters
        self.declare_parameter('experiment_name', 'mpc_evaluation')
        self.declare_parameter('evaluation_duration', 120.0)  # seconds
        self.declare_parameter('save_directory', '/tmp/mpc_evaluation')
        self.declare_parameter('sampling_rate', 20.0)  # Hz
        self.declare_parameter('enable_plots', True)
        self.declare_parameter('enable_real_time_plots', False)
        
        # Get parameters
        self.experiment_name = self.get_parameter('experiment_name').value
        self.evaluation_duration = self.get_parameter('evaluation_duration').value
        self.save_directory = self.get_parameter('save_directory').value
        self.sampling_rate = self.get_parameter('sampling_rate').value
        self.enable_plots = self.get_parameter('enable_plots').value
        self.enable_real_time_plots = self.get_parameter('enable_real_time_plots').value
        
        # FIX: Expand tilde in save directory path
        self.save_directory = os.path.expanduser(self.save_directory)
        
        # Create save directory
        os.makedirs(self.save_directory, exist_ok=True)
        
        # Data storage with thread safety
        self.data_lock = Lock()
        self.data_buffer = {
            'timestamps': deque(),
            'robot_poses': deque(),
            'reference_poses': deque(), 
            'cmd_velocities': deque(),
            'actual_velocities': deque(),
            'path_points': None,
            'control_inputs': deque(),
            'computational_times': deque()
        }
        
        # Performance metrics storage
        self.metrics = {
            'position_errors': deque(),
            'heading_errors': deque(),
            'velocity_errors': deque(),
            'cross_track_errors': deque(),
            'control_effort': deque(),
            'control_smoothness': deque(),
            'path_progress': deque()
        }
        
        # Previous values for derivative calculations
        self.prev_cmd_vel = None
        self.prev_timestamp = None
        self.start_time = None
        self.path_loaded = False
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/ground_truth', self.odom_callback, 10)
        self.ref_sub = self.create_subscription(
            PoseStamped, '/mpc/reference_pose', self.reference_callback, 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.path_sub = self.create_subscription(
            Path, '/path', self.path_callback, 10)
        
        # Timer for periodic evaluation and data recording
        self.timer = self.create_timer(1.0/self.sampling_rate, self.evaluation_timer)
        
        # Timer for experiment completion
        self.completion_timer = self.create_timer(
            self.evaluation_duration, self.complete_evaluation)
        
        # Real-time plotting setup
        if self.enable_real_time_plots:
            self.setup_real_time_plots()
        
        self.get_logger().info(f"MPC Performance Evaluator initialized")
        self.get_logger().info(f"Experiment: {self.experiment_name}")
        self.get_logger().info(f"Duration: {self.evaluation_duration}s")
        self.get_logger().info(f"Save directory: {self.save_directory}")
        
    def setup_real_time_plots(self):
        """Setup real-time plotting"""
        plt.ion()
        self.fig, self.axes = plt.subplots(2, 3, figsize=(15, 10))
        self.fig.suptitle(f'Real-time MPC Performance - {self.experiment_name}')
        
    def odom_callback(self, msg):
        """Handle robot odometry data"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        if self.start_time is None:
            self.start_time = current_time
            
        # Extract pose and velocity
        pose = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z,
            'timestamp': current_time - self.start_time
        }
        
        # Extract orientation
        orientation_q = msg.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        pose['yaw'] = yaw
        
        # Extract velocities
        velocity = {
            'linear_x': msg.twist.twist.linear.x,
            'linear_y': msg.twist.twist.linear.y,
            'angular_z': msg.twist.twist.angular.z,
            'timestamp': current_time - self.start_time
        }
        
        with self.data_lock:
            self.data_buffer['timestamps'].append(current_time - self.start_time)
            self.data_buffer['robot_poses'].append(pose)
            self.data_buffer['actual_velocities'].append(velocity)
            
    def reference_callback(self, msg):
        """Handle MPC reference pose data"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        if self.start_time is None:
            return
            
        # Extract reference pose
        ref_pose = {
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'z': msg.pose.position.z,
            'timestamp': current_time - self.start_time
        }
        
        # Extract reference orientation
        orientation_q = msg.pose.orientation
        (_, _, yaw) = euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        ref_pose['yaw'] = yaw
        
        with self.data_lock:
            self.data_buffer['reference_poses'].append(ref_pose)
            
    def cmd_vel_callback(self, msg):
        """Handle command velocity data"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        if self.start_time is None:
            return
            
        cmd_vel = {
            'linear_x': msg.linear.x,
            'linear_y': msg.linear.y,
            'angular_z': msg.angular.z,
            'timestamp': current_time - self.start_time
        }
        
        with self.data_lock:
            self.data_buffer['cmd_velocities'].append(cmd_vel)
            
        # Calculate control smoothness if we have previous command
        if self.prev_cmd_vel is not None and self.prev_timestamp is not None:
            dt = current_time - self.prev_timestamp
            if dt > 0:
                linear_accel = (msg.linear.x - self.prev_cmd_vel.linear.x) / dt
                angular_accel = (msg.angular.z - self.prev_cmd_vel.angular.z) / dt
                
                control_smoothness = math.sqrt(linear_accel**2 + angular_accel**2)
                self.metrics['control_smoothness'].append(control_smoothness)
                
        self.prev_cmd_vel = msg
        self.prev_timestamp = current_time
        
    def path_callback(self, msg):
        """Handle reference path data"""
        if self.path_loaded:
            return
            
        path_points = []
        for pose_stamped in msg.poses:
            point = {
                'x': pose_stamped.pose.position.x,
                'y': pose_stamped.pose.position.y,
                'z': pose_stamped.pose.position.z
            }
            
            # Extract yaw
            orientation_q = pose_stamped.pose.orientation
            (_, _, yaw) = euler_from_quaternion([
                orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
            point['yaw'] = yaw
            
            path_points.append(point)
            
        with self.data_lock:
            self.data_buffer['path_points'] = path_points
            
        self.path_loaded = True
        self.get_logger().info(f"Loaded reference path with {len(path_points)} points")
        
    def evaluation_timer(self):
        """Periodic evaluation and metric calculation"""
        with self.data_lock:
            # Need at least robot pose and reference pose
            if (len(self.data_buffer['robot_poses']) == 0 or 
                len(self.data_buffer['reference_poses']) == 0):
                return
                
            # Get latest data
            robot_pose = self.data_buffer['robot_poses'][-1]
            ref_pose = self.data_buffer['reference_poses'][-1]
            
            # Calculate position error (Euclidean distance)
            pos_error = math.sqrt(
                (robot_pose['x'] - ref_pose['x'])**2 + 
                (robot_pose['y'] - ref_pose['y'])**2
            )
            self.metrics['position_errors'].append(pos_error)
            
            # Calculate heading error
            heading_error = self.normalize_angle(robot_pose['yaw'] - ref_pose['yaw'])
            self.metrics['heading_errors'].append(abs(heading_error))
            
            # Calculate velocity error if we have both actual and commanded velocities
            if (len(self.data_buffer['actual_velocities']) > 0 and 
                len(self.data_buffer['cmd_velocities']) > 0):
                
                actual_vel = self.data_buffer['actual_velocities'][-1]
                cmd_vel = self.data_buffer['cmd_velocities'][-1]
                
                vel_error = math.sqrt(
                    (actual_vel['linear_x'] - cmd_vel['linear_x'])**2 +
                    (actual_vel['angular_z'] - cmd_vel['angular_z'])**2
                )
                self.metrics['velocity_errors'].append(vel_error)
                
                # Control effort (magnitude of control inputs)
                control_effort = math.sqrt(
                    cmd_vel['linear_x']**2 + cmd_vel['angular_z']**2
                )
                self.metrics['control_effort'].append(control_effort)
            
            # Calculate cross-track error if path is available
            if self.data_buffer['path_points'] is not None:
                cross_track_error = self.calculate_cross_track_error(
                    robot_pose, self.data_buffer['path_points'])
                self.metrics['cross_track_errors'].append(cross_track_error)
                
                # Calculate path progress
                progress = self.calculate_path_progress(
                    robot_pose, self.data_buffer['path_points'])
                self.metrics['path_progress'].append(progress)
        
        # Update real-time plots if enabled
        if self.enable_real_time_plots:
            self.update_real_time_plots()
            
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
        
    def calculate_cross_track_error(self, robot_pose, path_points):
        """Calculate minimum distance from robot to path"""
        if len(path_points) < 2:
            return 0.0
            
        min_distance = float('inf')
        
        for i in range(len(path_points) - 1):
            # Calculate distance from point to line segment
            p1 = np.array([path_points[i]['x'], path_points[i]['y']])
            p2 = np.array([path_points[i+1]['x'], path_points[i+1]['y']])
            robot_point = np.array([robot_pose['x'], robot_pose['y']])
            
            distance = self.point_to_line_distance(robot_point, p1, p2)
            min_distance = min(min_distance, distance)
            
        return min_distance
        
    def point_to_line_distance(self, point, line_start, line_end):
        """Calculate distance from point to line segment"""
        line_vec = line_end - line_start
        point_vec = point - line_start
        
        line_len = np.linalg.norm(line_vec)
        if line_len == 0:
            return np.linalg.norm(point_vec)
            
        line_unitvec = line_vec / line_len
        proj_length = np.dot(point_vec, line_unitvec)
        
        if proj_length < 0:
            return np.linalg.norm(point_vec)
        elif proj_length > line_len:
            return np.linalg.norm(point - line_end)
        else:
            proj_point = line_start + proj_length * line_unitvec
            return np.linalg.norm(point - proj_point)
            
    def calculate_path_progress(self, robot_pose, path_points):
        """Calculate progress along the path (0-100%)"""
        if len(path_points) < 2:
            return 0.0
            
        # Find closest point on path
        min_distance = float('inf')
        closest_segment = 0
        
        for i in range(len(path_points) - 1):
            p1 = np.array([path_points[i]['x'], path_points[i]['y']])
            p2 = np.array([path_points[i+1]['x'], path_points[i+1]['y']])
            robot_point = np.array([robot_pose['x'], robot_pose['y']])
            
            distance = self.point_to_line_distance(robot_point, p1, p2)
            if distance < min_distance:
                min_distance = distance
                closest_segment = i
                
        # Calculate total path length
        total_length = 0.0
        for i in range(len(path_points) - 1):
            p1 = np.array([path_points[i]['x'], path_points[i]['y']])
            p2 = np.array([path_points[i+1]['x'], path_points[i+1]['y']])
            total_length += np.linalg.norm(p2 - p1)
            
        # Calculate length to closest segment
        length_to_segment = 0.0
        for i in range(closest_segment):
            p1 = np.array([path_points[i]['x'], path_points[i]['y']])
            p2 = np.array([path_points[i+1]['x'], path_points[i+1]['y']])
            length_to_segment += np.linalg.norm(p2 - p1)
            
        if total_length == 0:
            return 0.0
            
        return (length_to_segment / total_length) * 100.0
        
    def update_real_time_plots(self):
        """Update real-time plots"""
        if len(self.metrics['position_errors']) < 10:
            return
            
        # Clear previous plots
        for ax in self.axes.flat:
            ax.clear()
            
        # Plot position errors
        self.axes[0, 0].plot(list(self.metrics['position_errors']))
        self.axes[0, 0].set_title('Position Error (m)')
        self.axes[0, 0].set_ylabel('Error (m)')
        
        # Plot heading errors
        self.axes[0, 1].plot(list(self.metrics['heading_errors']))
        self.axes[0, 1].set_title('Heading Error (rad)')
        self.axes[0, 1].set_ylabel('Error (rad)')
        
        # Plot velocity errors
        if len(self.metrics['velocity_errors']) > 0:
            self.axes[0, 2].plot(list(self.metrics['velocity_errors']))
            self.axes[0, 2].set_title('Velocity Error')
            self.axes[0, 2].set_ylabel('Error')
        
        # Plot cross-track errors
        if len(self.metrics['cross_track_errors']) > 0:
            self.axes[1, 0].plot(list(self.metrics['cross_track_errors']))
            self.axes[1, 0].set_title('Cross-track Error (m)')
            self.axes[1, 0].set_ylabel('Error (m)')
        
        # Plot control effort
        if len(self.metrics['control_effort']) > 0:
            self.axes[1, 1].plot(list(self.metrics['control_effort']))
            self.axes[1, 1].set_title('Control Effort')
            self.axes[1, 1].set_ylabel('Magnitude')
        
        # Plot path progress
        if len(self.metrics['path_progress']) > 0:
            self.axes[1, 2].plot(list(self.metrics['path_progress']))
            self.axes[1, 2].set_title('Path Progress (%)')
            self.axes[1, 2].set_ylabel('Progress (%)')
            
        plt.pause(0.01)
        
    def complete_evaluation(self):
        """Complete the evaluation and generate reports"""
        self.get_logger().info("Evaluation completed - generating reports...")
        
        # Save raw data
        self.save_raw_data()
        
        # Calculate summary statistics
        summary_stats = self.calculate_summary_statistics()
        
        # Save summary statistics
        self.save_summary_statistics(summary_stats)
        
        # Generate plots if enabled
        if self.enable_plots:
            self.generate_comprehensive_plots()
            
        # Generate evaluation report
        self.generate_evaluation_report(summary_stats)
        
        self.get_logger().info(f"Evaluation complete! Results saved to {self.save_directory}")
        
        # Shutdown the node
        self.destroy_node()
        rclpy.shutdown()
        
    def save_raw_data(self):
        """Save raw collected data to files"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        with self.data_lock:
            # Convert deque data to lists for JSON serialization
            raw_data = {}
            for key, value in self.data_buffer.items():
                if isinstance(value, deque):
                    raw_data[key] = list(value)
                else:
                    raw_data[key] = value
                    
            # Save raw sensor data
            with open(f"{self.save_directory}/raw_data_{timestamp}.json", 'w') as f:
                json.dump(raw_data, f, indent=2)
                
            # Convert metrics to lists
            metrics_data = {}
            for key, value in self.metrics.items():
                metrics_data[key] = list(value)
                
            # Save metrics data
            with open(f"{self.save_directory}/metrics_{timestamp}.json", 'w') as f:
                json.dump(metrics_data, f, indent=2)
                
        self.get_logger().info("Raw data saved")
        
    def calculate_summary_statistics(self):
        """Calculate summary statistics for all metrics"""
        stats = {}
        
        for metric_name, metric_data in self.metrics.items():
            if len(metric_data) == 0:
                continue
                
            data_array = np.array(list(metric_data))
            
            stats[metric_name] = {
                'mean': float(np.mean(data_array)),
                'std': float(np.std(data_array)),
                'min': float(np.min(data_array)),
                'max': float(np.max(data_array)),
                'median': float(np.median(data_array)),
                'q25': float(np.percentile(data_array, 25)),
                'q75': float(np.percentile(data_array, 75)),
                'rms': float(np.sqrt(np.mean(data_array**2))),
                'count': len(data_array)
            }
            
        return stats
        
    def save_summary_statistics(self, stats):
        """Save summary statistics to file"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Save as JSON
        with open(f"{self.save_directory}/summary_stats_{timestamp}.json", 'w') as f:
            json.dump(stats, f, indent=2)
            
        # Save as CSV for easy analysis
        df_stats = pd.DataFrame(stats).T
        df_stats.to_csv(f"{self.save_directory}/summary_stats_{timestamp}.csv")
        
        self.get_logger().info("Summary statistics saved")
        
    def generate_comprehensive_plots(self):
        """Generate comprehensive performance analysis plots"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Set up the plotting style
        try:
            plt.style.use('seaborn-v0_8')
        except OSError:
            try:
                plt.style.use('seaborn-darkgrid')
            except OSError:
                self.get_logger().warn("Seaborn style not available, using default")
        
        sns.set_palette("husl")
        
        # Create comprehensive figure
        fig = plt.figure(figsize=(20, 16))
        fig.suptitle(f'MPC Controller Performance Analysis - {self.experiment_name}', 
                    fontsize=16, fontweight='bold')
        
        # Define subplot layout
        gs = fig.add_gridspec(4, 4, hspace=0.4, wspace=0.4)
        
        # 1. Position and heading errors over time
        ax1 = fig.add_subplot(gs[0, :2])
        if len(self.metrics['position_errors']) > 0:
            time_axis = np.arange(len(self.metrics['position_errors'])) / self.sampling_rate
            ax1.plot(time_axis, list(self.metrics['position_errors']), 'b-', label='Position Error')
            ax1.set_xlabel('Time (s)')
            ax1.set_ylabel('Position Error (m)')
            ax1.set_title('Position Tracking Error Over Time')
            ax1.grid(True, alpha=0.3)
            ax1.legend()
        
        ax2 = fig.add_subplot(gs[0, 2:])
        if len(self.metrics['heading_errors']) > 0:
            time_axis = np.arange(len(self.metrics['heading_errors'])) / self.sampling_rate
            ax2.plot(time_axis, np.degrees(list(self.metrics['heading_errors'])), 'r-', label='Heading Error')
            ax2.set_xlabel('Time (s)')
            ax2.set_ylabel('Heading Error (deg)')
            ax2.set_title('Heading Tracking Error Over Time')
            ax2.grid(True, alpha=0.3)
            ax2.legend()
        
        # 2. Cross-track error and path progress
        ax3 = fig.add_subplot(gs[1, :2])
        if len(self.metrics['cross_track_errors']) > 0:
            time_axis = np.arange(len(self.metrics['cross_track_errors'])) / self.sampling_rate
            ax3.plot(time_axis, list(self.metrics['cross_track_errors']), 'g-', label='Cross-track Error')
            ax3.set_xlabel('Time (s)')
            ax3.set_ylabel('Cross-track Error (m)')
            ax3.set_title('Cross-track Error Over Time')
            ax3.grid(True, alpha=0.3)
            ax3.legend()
        
        ax4 = fig.add_subplot(gs[1, 2:])
        if len(self.metrics['path_progress']) > 0:
            time_axis = np.arange(len(self.metrics['path_progress'])) / self.sampling_rate
            ax4.plot(time_axis, list(self.metrics['path_progress']), 'm-', label='Path Progress')
            ax4.set_xlabel('Time (s)')
            ax4.set_ylabel('Progress (%)')
            ax4.set_title('Path Following Progress')
            ax4.grid(True, alpha=0.3)
            ax4.legend()
        
        # 3. Control performance
        ax5 = fig.add_subplot(gs[2, :2])
        if len(self.metrics['control_effort']) > 0:
            time_axis = np.arange(len(self.metrics['control_effort'])) / self.sampling_rate
            ax5.plot(time_axis, list(self.metrics['control_effort']), 'c-', label='Control Effort')
            ax5.set_xlabel('Time (s)')
            ax5.set_ylabel('Control Magnitude')
            ax5.set_title('Control Effort Over Time')
            ax5.grid(True, alpha=0.3)
            ax5.legend()
        
        ax6 = fig.add_subplot(gs[2, 2:])
        if len(self.metrics['control_smoothness']) > 0:
            time_axis = np.arange(len(self.metrics['control_smoothness'])) / self.sampling_rate
            ax6.plot(time_axis, list(self.metrics['control_smoothness']), 'orange', label='Control Smoothness')
            ax6.set_xlabel('Time (s)')
            ax6.set_ylabel('Control Rate (jerk)')
            ax6.set_title('Control Smoothness Over Time')
            ax6.grid(True, alpha=0.3)
            ax6.legend()
        
        # 4. Error distribution histograms
        ax7 = fig.add_subplot(gs[3, 0])
        if len(self.metrics['position_errors']) > 0:
            try:
                sns.histplot(list(self.metrics['position_errors']), bins=30, kde=True, ax=ax7, color='blue')
            except:
                ax7.hist(list(self.metrics['position_errors']), bins=30, alpha=0.7, color='blue')
            ax7.set_xlabel('Position Error (m)')
            ax7.set_ylabel('Frequency')
            ax7.set_title('Position Error Distribution')
            ax7.grid(True, alpha=0.3)
        
        ax8 = fig.add_subplot(gs[3, 1])
        if len(self.metrics['heading_errors']) > 0:
            try:
                sns.histplot(np.degrees(list(self.metrics['heading_errors'])), bins=30, kde=True, ax=ax8, color='red')
            except:
                ax8.hist(np.degrees(list(self.metrics['heading_errors'])), bins=30, alpha=0.7, color='red')
            ax8.set_xlabel('Heading Error (deg)')
            ax8.set_ylabel('Frequency')
            ax8.set_title('Heading Error Distribution')
            ax8.grid(True, alpha=0.3)
        
        # 5. Trajectory plot (if robot poses available)
        ax9 = fig.add_subplot(gs[3, 2:])
        with self.data_lock:
            if (len(self.data_buffer['robot_poses']) > 0 and 
                self.data_buffer['path_points'] is not None):
                
                # Plot reference path
                path_x = [p['x'] for p in self.data_buffer['path_points']]
                path_y = [p['y'] for p in self.data_buffer['path_points']]
                ax9.plot(path_x, path_y, 'k--', linewidth=2, label='Reference Path')
                
                # Plot actual trajectory
                robot_x = [p['x'] for p in self.data_buffer['robot_poses']]
                robot_y = [p['y'] for p in self.data_buffer['robot_poses']]
                ax9.plot(robot_x, robot_y, 'b-', linewidth=1.5, label='Actual Trajectory')
                
                # Mark start and end points
                if len(robot_x) > 0:
                    ax9.plot(robot_x[0], robot_y[0], 'go', markersize=8, label='Start')
                    ax9.plot(robot_x[-1], robot_y[-1], 'ro', markersize=8, label='End')
                
                ax9.set_xlabel('X (m)')
                ax9.set_ylabel('Y (m)')
                ax9.set_title('Trajectory Comparison')
                ax9.axis('equal')
                ax9.grid(True, alpha=0.3)
                ax9.legend()
        
        # Save the comprehensive plot
        plt.tight_layout(rect=[0, 0.03, 1, 0.95])
        plt.savefig(f"{self.save_directory}/comprehensive_analysis_{timestamp}.png", 
                   dpi=300, bbox_inches='tight')
        plt.savefig(f"{self.save_directory}/comprehensive_analysis_{timestamp}.pdf", 
                   bbox_inches='tight')
        
        plt.close(fig)
        self.get_logger().info("Comprehensive plots generated")
        
    def generate_evaluation_report(self, stats):
        """Generate a comprehensive evaluation report"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        report_content = f"""
# MPC Controller Performance Evaluation Report

**Experiment Name:** {self.experiment_name}  
**Date:** {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}  
**Duration:** {self.evaluation_duration} seconds  
**Sampling Rate:** {self.sampling_rate} Hz  

## Executive Summary

This report presents a comprehensive evaluation of the MPC controller performance based on metrics adapted from odometry evaluation literature. The analysis includes tracking accuracy, control performance, and computational efficiency.

## Performance Metrics Summary

### Position Tracking Performance
"""
        
        # Add position error statistics if available
        if 'position_errors' in stats:
            pe_stats = stats['position_errors']
            report_content += f"""
- **Mean Position Error:** {pe_stats['mean']:.4f} m
- **RMS Position Error:** {pe_stats['rms']:.4f} m
- **Maximum Position Error:** {pe_stats['max']:.4f} m
- **Standard Deviation:** {pe_stats['std']:.4f} m
- **95th Percentile:** {pe_stats.get('q75', 0):.4f} m
"""
        
        # Add heading error statistics if available
        if 'heading_errors' in stats:
            he_stats = stats['heading_errors']
            report_content += f"""
### Heading Tracking Performance
- **Mean Heading Error:** {math.degrees(he_stats['mean']):.2f} degrees
- **RMS Heading Error:** {math.degrees(he_stats['rms']):.2f} degrees
- **Maximum Heading Error:** {math.degrees(he_stats['max']):.2f} degrees
- **Standard Deviation:** {math.degrees(he_stats['std']):.2f} degrees
"""
        
        # Add cross-track error statistics if available
        if 'cross_track_errors' in stats:
            cte_stats = stats['cross_track_errors']
            report_content += f"""
### Path Following Performance
- **Mean Cross-track Error:** {cte_stats['mean']:.4f} m
- **RMS Cross-track Error:** {cte_stats['rms']:.4f} m
- **Maximum Cross-track Error:** {cte_stats['max']:.4f} m
- **Standard Deviation:** {cte_stats['std']:.4f} m
"""
        
        # Add control performance statistics
        if 'control_effort' in stats:
            ce_stats = stats['control_effort']
            report_content += f"""
### Control Performance
- **Mean Control Effort:** {ce_stats['mean']:.4f}
- **RMS Control Effort:** {ce_stats['rms']:.4f}
- **Maximum Control Effort:** {ce_stats['max']:.4f}
"""
        
        if 'control_smoothness' in stats:
            cs_stats = stats['control_smoothness']
            report_content += f"""
- **Mean Control Smoothness:** {cs_stats['mean']:.4f} (1/s²)
- **RMS Control Smoothness:** {cs_stats['rms']:.4f} (1/s²)
- **Maximum Control Rate:** {cs_stats['max']:.4f} (1/s²)
"""
        
        # Add path progress information
        if 'path_progress' in stats and stats['path_progress']['count'] > 0:
            pp_stats = stats['path_progress']
            report_content += f"""
### Path Completion
- **Final Path Progress:** {pp_stats['max']:.1f}%
- **Average Progress Rate:** {pp_stats['mean']/self.evaluation_duration*100:.2f}%/s
"""
        
        # Performance assessment
        report_content += """
## Performance Assessment

### Tracking Accuracy
"""
        
        if 'position_errors' in stats:
            pe_rms = stats['position_errors']['rms']
            if pe_rms < 0.1:
                report_content += "- **Position Tracking:** EXCELLENT (RMS < 0.1m)\n"
            elif pe_rms < 0.2:
                report_content += "- **Position Tracking:** GOOD (RMS < 0.2m)\n"
            elif pe_rms < 0.5:
                report_content += "- **Position Tracking:** ACCEPTABLE (RMS < 0.5m)\n"
            else:
                report_content += "- **Position Tracking:** POOR (RMS > 0.5m)\n"
        
        if 'heading_errors' in stats:
            he_rms = math.degrees(stats['heading_errors']['rms'])
            if he_rms < 5:
                report_content += "- **Heading Tracking:** EXCELLENT (RMS < 5°)\n"
            elif he_rms < 10:
                report_content += "- **Heading Tracking:** GOOD (RMS < 10°)\n"
            elif he_rms < 20:
                report_content += "- **Heading Tracking:** ACCEPTABLE (RMS < 20°)\n"
            else:
                report_content += "- **Heading Tracking:** POOR (RMS > 20°)\n"
        
        # Control quality assessment
        report_content += """
### Control Quality
"""
        
        if 'control_smoothness' in stats:
            cs_mean = stats['control_smoothness']['mean']
            if cs_mean < 1.0:
                report_content += "- **Control Smoothness:** EXCELLENT (Low jerk)\n"
            elif cs_mean < 2.0:
                report_content += "- **Control Smoothness:** GOOD (Moderate jerk)\n"
            elif cs_mean < 5.0:
                report_content += "- **Control Smoothness:** ACCEPTABLE (Higher jerk)\n"
            else:
                report_content += "- **Control Smoothness:** POOR (Very high jerk)\n"
        
        # Recommendations
        report_content += """
## Recommendations

### Parameter Tuning Suggestions
"""
        
        if 'position_errors' in stats and stats['position_errors']['rms'] > 0.2:
            report_content += "- Consider increasing position tracking weights (Q matrix)\n"
        
        if 'heading_errors' in stats and math.degrees(stats['heading_errors']['rms']) > 10:
            report_content += "- Consider increasing yaw tracking weight in Q matrix\n"
        
        if 'control_smoothness' in stats and stats['control_smoothness']['mean'] > 2.0:
            report_content += "- Consider increasing control rate penalties (Rd matrix)\n"
            report_content += "- Consider reducing control horizon or increasing control cost (R matrix)\n"
        
        if 'cross_track_errors' in stats and stats['cross_track_errors']['mean'] > 0.3:
            report_content += "- Consider increasing lookahead distance (N_IND_SEARCH)\n"
            report_content += "- Consider reducing target speed for better path following\n"
        
        report_content += """
### System Improvements
- Monitor computational performance to ensure real-time capability
- Consider adaptive parameter tuning based on path curvature
- Implement path preview for better anticipatory control
- Add constraint handling for improved feasibility

## Data Files Generated
- Raw sensor data: `raw_data_*.json`
- Processed metrics: `metrics_*.json`  
- Summary statistics: `summary_stats_*.csv`
- Comprehensive plots: `comprehensive_analysis_*.png`

---
*Report generated automatically by MPC Performance Evaluator*
"""
        
        # Save the report
        with open(f"{self.save_directory}/evaluation_report_{timestamp}.md", 'w') as f:
            f.write(report_content)
            
        # Also save as text file
        with open(f"{self.save_directory}/evaluation_report_{timestamp}.txt", 'w') as f:
            f.write(report_content)
            
        self.get_logger().info("Evaluation report generated")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        evaluator = MPCPerformanceEvaluator()
        rclpy.spin(evaluator)
    except KeyboardInterrupt:
        print("Evaluation interrupted by user")
    except Exception as e:
        print(f"Error in evaluation: {e}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()