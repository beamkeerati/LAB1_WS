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

class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')
        
        # Declare and retrieve parameters
        self.declare_parameter("mode", "car")
        self.mode = self.get_parameter("mode").get_parameter_value().string_value
        
        # Set up subscriber and timer
        self.odom_sub = self.create_subscription(Odometry, '/odometry/ground_truth', self.odom_callback, 10)
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
        self.L = 1.0               # vehicle wheelbase used for steering angle calculation
        
        # Pure pursuit state variables
        self.currentPos = [0.0, 0.0]       # current [x, y] position
        self.currentHeading = 330          # current heading in degrees
        self.lastFoundIndex = 0            # pointer for pure pursuit step
        self.lookAheadDis = 0.8            # look-ahead distance for goal point
        self.linearVel = 100               # scaling parameter (used in simulation code; here used to compute updates)

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
    
    def sgn(self, num):
        """Returns 1 if num is nonnegative, -1 if negative."""
        return 1 if num >= 0 else -1

    def pt_to_pt_distance(self, pt1, pt2):
        """Computes Euclidean distance between two 2D points."""
        return math.sqrt((pt2[0] - pt1[0])**2 + (pt2[1] - pt1[1])**2)
    
    def pure_pursuit_step(self, path, currentPos, currentHeading, lookAheadDis, LFindex):
        """
        Computes the goal point along the path using the pure pursuit algorithm.
        Returns: goalPt (target point), updated lastFoundIndex, and turnVel (proportional control output).
        """
        currentX = currentPos[0]
        currentY = currentPos[1]
        lastFoundIndex = LFindex
        foundIntersection = False
        startingIndex = lastFoundIndex
        goalPt = [currentX, currentY]  # default to current pos if no intersection found
        
        for i in range(startingIndex, len(path) - 1):
            x1 = path[i][0] - currentX
            y1 = path[i][1] - currentY
            x2 = path[i + 1][0] - currentX
            y2 = path[i + 1][1] - currentY
            dx = x2 - x1
            dy = y2 - y1
            dr = math.sqrt(dx**2 + dy**2)
            D = x1 * y2 - x2 * y1
            discriminant = (lookAheadDis**2) * (dr**2) - D**2

            if discriminant >= 0:
                sol_x1 = (D * dy + self.sgn(dy) * dx * math.sqrt(discriminant)) / (dr**2)
                sol_x2 = (D * dy - self.sgn(dy) * dx * math.sqrt(discriminant)) / (dr**2)
                sol_y1 = (-D * dx + abs(dy) * math.sqrt(discriminant)) / (dr**2)
                sol_y2 = (-D * dx - abs(dy) * math.sqrt(discriminant)) / (dr**2)

                sol_pt1 = [sol_x1 + currentX, sol_y1 + currentY]
                sol_pt2 = [sol_x2 + currentX, sol_y2 + currentY]

                minX = min(path[i][0], path[i + 1][0])
                minY = min(path[i][1], path[i + 1][1])
                maxX = max(path[i][0], path[i + 1][0])
                maxY = max(path[i][1], path[i + 1][1])

                if (((minX <= sol_pt1[0] <= maxX) and (minY <= sol_pt1[1] <= maxY)) or 
                    ((minX <= sol_pt2[0] <= maxX) and (minY <= sol_pt2[1] <= maxY))):
                    
                    foundIntersection = True
                    
                    if (((minX <= sol_pt1[0] <= maxX) and (minY <= sol_pt1[1] <= maxY)) and 
                        ((minX <= sol_pt2[0] <= maxX) and (minY <= sol_pt2[1] <= maxY))):
                        if self.pt_to_pt_distance(sol_pt1, path[i+1]) < self.pt_to_pt_distance(sol_pt2, path[i+1]):
                            goalPt = sol_pt1
                        else:
                            goalPt = sol_pt2
                    else:
                        if (minX <= sol_pt1[0] <= maxX) and (minY <= sol_pt1[1] <= maxY):
                            goalPt = sol_pt1
                        else:
                            goalPt = sol_pt2
                    
                    if self.pt_to_pt_distance(goalPt, path[i+1]) < self.pt_to_pt_distance([currentX, currentY], path[i+1]):
                        lastFoundIndex = i
                        break
                    else:
                        lastFoundIndex = i + 1
                else:
                    foundIntersection = False
                    goalPt = [path[lastFoundIndex][0], path[lastFoundIndex][1]]
        # Compute turn velocity using proportional control (used here for debugging/monitoring)
        Kp = 3
        absTargetAngle = math.degrees(math.atan2(goalPt[1] - currentPos[1], goalPt[0] - currentPos[0]))
        if absTargetAngle < 0:
            absTargetAngle += 360
        turnError = absTargetAngle - currentHeading
        if turnError > 180 or turnError < -180:
            turnError = -1 * self.sgn(turnError) * (360 - abs(turnError))
        turnVel = Kp * turnError
        return goalPt, lastFoundIndex, turnVel

    def pure_pursuit(self):
        """
        Computes the pure pursuit control action using the current odometry.
        It uses a constant speed, calculates the look-ahead distance, and then
        computes the desired steering angle based on the goal point.
        Finally, it commands the robot's velocity and steering.
        """
        # Set a constant speed
        speed = 2.0
        K_dd = 0.5
        min_ld = 0.1
        max_ld = 2.0
        L_d = np.clip(K_dd * speed, min_ld, max_ld)
        
        # Update current state from odometry (using a slight offset in x as in your original node)
        x = self.robot_odom.pose.pose.position.x - 0.10
        y = self.robot_odom.pose.pose.position.y
        orientation_q = self.robot_odom.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.currentPos = [x, y]
        self.currentHeading = math.degrees(yaw)
        
        # Convert the path (loaded as a NumPy array of [x,y,yaw]) to a list of [x,y] for the pure pursuit algorithm.
        path_xy = self.path[:, :2].tolist()
        
        goalPt, self.lastFoundIndex, turnVel = self.pure_pursuit_step(path_xy, self.currentPos, self.currentHeading, self.lookAheadDis, self.lastFoundIndex)
        self.get_logger().info(f"GoalPt: {goalPt}, LastFoundIndex: {self.lastFoundIndex}, TurnVel: {turnVel:.2f}")
        
        # Compute the target angle from the goal point.
        absTargetAngle = math.degrees(math.atan2(goalPt[1] - self.currentPos[1], goalPt[0] - self.currentPos[0]))
        if absTargetAngle < 0:
            absTargetAngle += 360
        heading_error = absTargetAngle - self.currentHeading
        # Normalize the heading error to the range [-180, 180]
        if heading_error > 180:
            heading_error -= 360
        elif heading_error < -180:
            heading_error += 360
        
        heading_error_rad = math.radians(heading_error)
        # Compute desired steering angle using a simple bicycle model formula.
        steering_angle = math.atan2(2 * self.L * math.sin(heading_error_rad), L_d)
        
        # Command the vehicle's velocity and steering angle.
        self.cmd_vel_steering(speed, steering_angle)
    
    def timer_callback(self):
        # Publish the full path (optional, e.g., for visualization in RViz)
        self.publish_path()
        # Execute pure pursuit control logic periodically
        self.pure_pursuit()
    
    def cmd_vel_steering(self, vx, steering_angle):
        mode = self.get_parameter("mode").get_parameter_value().string_value
        if vx == 0:
            self.set_steering_angle(0, 0)
            self.set_velocity(0, 0)
            return
        
        angle_max = 10.0/180.0*math.pi
        angle_min = -10.0/180.0*math.pi
        steering_angle = np.clip(steering_angle, a_min = angle_min, a_max = angle_max)       
        
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
        
        self.set_steering_angle(left_angle, right_angle)
        self.set_velocity(wheel_speed, wheel_speed)
    
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
    node = PurePursuitNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
