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
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import math

@dataclass
class JointStateData:
    position: float = None
    velocity: float = None
    effort: float = None

class DoubleTrackData:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    beta: float = 0.0
    v: float = 0.0
    omega: float = 0.0
    dt: float = 0.01
    v_RL: float = 0.0
    v_RR: float = 0.0
    track_width: float = 0.14
    wheel_radius: float = 0.045
    wheel_base: float = 0.2

class SingleTrackData:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    beta: float = 0.0
    v: float = 0.0
    omega: float = 0.0
    dt: float = 0.01
    v_RL: float = 0.0
    v_RR: float = 0.0
    right_wheel_angle: float = 0.0
    left_wheel_angle: float = 0.0
    steering_angle: float = 0.0
    track_width: float = 0.14
    wheel_radius: float = 0.045
    wheel_base: float = 0.2
    
class YawRateData:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    beta: float = 0.0
    v: float = 0.0
    omega: float = 0.0
    dt: float = 0.01
    v_RL: float = 0.0
    v_RR: float = 0.0
    track_width: float = 0.14
    wheel_radius: float = 0.045
    wheel_base: float = 0.2

class FKNode(Node):
    def __init__(self):
        super().__init__("fk_node")
        # Dynamic transform broadcaster for continuous odometry updates.
        self.tf_bc = TransformBroadcaster(self)
        # Static transform broadcaster to publish the initial fixed transform once.
        self.static_tf_bc = StaticTransformBroadcaster(self)
        
        # Initialize the robot odometry variable and a flag to check initial reception.
        self.robot_odom = None
        self.odom_initialized = False

        # Subscribe to the ground truth odometry topic.
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/ground_truth', self.odom_callback, 10
        )
        
        self.create_subscription(
            Imu,                # Message type
            '/imu_plugin/out',  # Topic name
            self.imu_callback,  # Callback function
            10                  # QoS history depth
        )
        
        self.create_subscription(
            JointState,         # Message type
            "/joint_states",    # Topic name
            self.joint_states_callback,  # Callback function
            10                  # QoS history depth
        )
        
        # Predefined order for desired joints
        self.order = [
            "right_steering_hinge_wheel",
            "front_left_wheel",
            "left_steering_hinge_wheel",
            "front_right_wheel",
            "rear_left_wheel",
            "rear_right_wheel",
        ]
        self.joint_state = {}

        self.odom_double_track_publisher = self.create_publisher(Odometry, "double_track/odom", 10)
        self.odom_single_track_publisher = self.create_publisher(Odometry, "single_track/odom", 10)
        self.odom_yaw_rate_publisher = self.create_publisher(Odometry, "yaw_rate/odom", 10)
        self.steer_L = self.create_publisher(Float64, "steer_L", 10)
        self.steer_R = self.create_publisher(Float64, "steer_R", 10)
        self.steer = self.create_publisher(Float64, "steer", 10)

        self.get_logger().warn("Forward Kinematics Node has been initialized.")
        
        # Instantiate kinematic data classes with default values.
        # Their initial conditions will be overwritten when the first odom message is received.
        self.d_track = DoubleTrackData()
        self.s_track = SingleTrackData()
        self.y_rate = YawRateData()

        # Publish the initial static transform (one time) from 'odom' to 'base_footprint'
        self.send_initial_static_transform()
        
    def odom_callback(self, msg: Odometry):
        if not self.odom_initialized:
            # Set the initial conditions based on the first odometry message.
            self.d_track.x = msg.pose.pose.position.x
            self.d_track.y = msg.pose.pose.position.y
            self.s_track.x = msg.pose.pose.position.x
            self.s_track.y = msg.pose.pose.position.y
            self.y_rate.x = msg.pose.pose.position.x
            self.y_rate.y = msg.pose.pose.position.y
            
            # Extract yaw from the quaternion for initial orientation.
            q = msg.pose.pose.orientation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            self.d_track.theta = yaw
            self.s_track.theta = yaw
            self.y_rate.theta = yaw
            
            self.odom_initialized = True
            self.get_logger().info("Initial odometry received and initial conditions set.")
        
        self.robot_odom = msg

    def send_initial_static_transform(self):
        """Publish a static transform at the initial state one time."""
        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = "world"
        static_transform.child_frame_id = "base_footprint"
        static_transform.transform.translation.x = 0.0
        static_transform.transform.translation.y = 0.0
        static_transform.transform.translation.z = 0.0
        q = quaternion_from_euler(0, 0, 0)
        static_transform.transform.rotation.x = q[0]
        static_transform.transform.rotation.y = q[1]
        static_transform.transform.rotation.z = q[2]
        static_transform.transform.rotation.w = q[3]
        self.static_tf_bc.sendTransform(static_transform)
        self.get_logger().info("Initial static transform from 'world' to 'base_footprint' published.")
        
    def imu_callback(self, msg: Imu):
        self.y_rate.omega = msg.angular_velocity.z

    def safe_get(self, array, index, default=0.0):
        """Safely return the element at the given index; otherwise, return default (0.0)."""
        if array and index < len(array):
            value = array[index]
            return value if value is not None else default
        else:
            return default

    def joint_states_callback(self, msg: JointState):
        # Ensure we have received an initial odom message before processing joint states.
        if not self.odom_initialized:
            self.get_logger().warn("Waiting for initial odometry message before processing joint states.")
            return
        
        try:
            indices = {joint: msg.name.index(joint) for joint in self.order}
        except ValueError as e:
            self.get_logger().error(f"Joint name not found: {e}")
            return

        self.joint_state = {
            joint: JointStateData(
                position=self.safe_get(msg.position, indices[joint]),
                velocity=self.safe_get(msg.velocity, indices[joint]),
                effort=self.safe_get(msg.effort, indices[joint]),
            )
            for joint in self.order
        }
        try:
            self.double_track_odom()
            self.single_track_odom()
            self.yaw_rate_odom()
        except Exception as e:
            self.get_logger().error(f"Error in odometry computation: {e}")

    def double_track_odom(self):
        self.d_track.v_RL = (self.joint_state["rear_left_wheel"].velocity * self.d_track.wheel_radius)
        self.d_track.v_RR = (self.joint_state["rear_right_wheel"].velocity * self.d_track.wheel_radius)
        
        self.d_track.x += (self.d_track.v * self.d_track.dt *
                             math.cos(self.d_track.beta + self.d_track.theta +
                                      (self.d_track.omega * self.d_track.dt / 2.0)))
        self.d_track.y += (self.d_track.v * self.d_track.dt *
                             math.sin(self.d_track.beta + self.d_track.theta +
                                      (self.d_track.omega * self.d_track.dt / 2.0)))
        self.d_track.theta += self.d_track.omega * self.d_track.dt
        self.d_track.beta = 0.0
        self.d_track.v = (self.d_track.v_RL + self.d_track.v_RR) / 2.0
        self.d_track.omega = (self.d_track.v_RR - self.d_track.v_RL) / self.d_track.track_width

        self.odom_pub(
            self.d_track.x,
            self.d_track.y,
            self.d_track.theta,
            self.d_track.v,
            self.d_track.omega,
            "double_track"
        )

    def single_track_odom(self):
        self.s_track.v_RL = (self.joint_state["rear_left_wheel"].velocity * self.s_track.wheel_radius)
        self.s_track.v_RR = (self.joint_state["rear_right_wheel"].velocity * self.s_track.wheel_radius)
        self.s_track.right_wheel_angle = self.joint_state["right_steering_hinge_wheel"].position
        self.s_track.left_wheel_angle = self.joint_state["left_steering_hinge_wheel"].position
        
        steer_l = math.atan((self.s_track.wheel_base *
                             math.atan(self.s_track.left_wheel_angle)) /
                             (self.s_track.wheel_base - 0.5 * self.s_track.track_width *
                             math.tan(self.s_track.left_wheel_angle)))
        steer_r = math.atan((self.s_track.wheel_base *
                             math.atan(self.s_track.right_wheel_angle)) /
                             (self.s_track.wheel_base + 0.5 * self.s_track.track_width *
                             math.tan(self.s_track.right_wheel_angle)))
        if not (math.tan(self.s_track.left_wheel_angle) == 0 or math.tan(self.s_track.right_wheel_angle) == 0):
            steer = math.atan((2 * math.tan(self.s_track.left_wheel_angle) *
                               math.tan(self.s_track.right_wheel_angle)) /
                               (math.tan(self.s_track.left_wheel_angle) +
                                math.tan(self.s_track.right_wheel_angle)))
            self.steer.publish(Float64(data=steer))
        else:
            steer = 0.0
            self.steer.publish(Float64(data=0.0))
        self.steer_L.publish(Float64(data=steer_l))
        self.steer_R.publish(Float64(data=steer_r))

        self.s_track.x += (self.s_track.v * self.s_track.dt *
                           math.cos(self.s_track.beta + self.s_track.theta +
                                    (self.s_track.omega * self.s_track.dt / 2.0)))
        self.s_track.y += (self.s_track.v * self.s_track.dt *
                           math.sin(self.s_track.beta + self.s_track.theta +
                                    (self.s_track.omega * self.s_track.dt / 2.0)))
        self.s_track.theta += self.s_track.omega * self.s_track.dt
        self.s_track.beta = 0.0
        self.s_track.v = (self.s_track.v_RL + self.s_track.v_RR) / 2.0
        self.s_track.omega = (self.s_track.v / self.s_track.wheel_base) * math.tan(steer)

        self.odom_pub(
            self.s_track.x,
            self.s_track.y,
            self.s_track.theta,
            self.s_track.v,
            self.s_track.omega,
            "single_track"
        )
        
    def yaw_rate_odom(self):
        self.y_rate.v_RL = (self.joint_state["rear_left_wheel"].velocity * self.y_rate.wheel_radius)
        self.y_rate.v_RR = (self.joint_state["rear_right_wheel"].velocity * self.y_rate.wheel_radius)
        
        self.y_rate.x += (self.y_rate.v * self.y_rate.dt *
                          math.cos(self.y_rate.beta + self.y_rate.theta +
                                   (self.y_rate.omega * self.y_rate.dt / 2.0)))
        self.y_rate.y += (self.y_rate.v * self.y_rate.dt *
                          math.sin(self.y_rate.beta + self.y_rate.theta +
                                   (self.y_rate.omega * self.y_rate.dt / 2.0)))
        self.y_rate.theta += self.y_rate.omega * self.y_rate.dt
        self.y_rate.beta = 0.0
        self.y_rate.v = (self.y_rate.v_RL + self.y_rate.v_RR) / 2.0
        # Note: y_rate.omega is set directly from the IMU callback.
        
        self.odom_pub(
            self.y_rate.x,
            self.y_rate.y,
            self.y_rate.theta,
            self.y_rate.v,
            self.y_rate.omega,
            "yaw_rate"
        )
        
    def odom_pub(self, x, y, theta, v, omega, mode):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.child_frame_id = "base_footprint"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        q = quaternion_from_euler(0, 0, theta)
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        msg.pose.covariance = [0.0] * 36
        msg.twist.twist.linear.x = v
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = omega
        msg.twist.covariance = [0.0] * 36

        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = "world"
        transform.child_frame_id = "base_footprint"
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        
        #Ground truth odometry
        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = "world"
        transform.child_frame_id = "base_footprint"
        transform.transform.translation.x = self.robot_odom.pose.pose.position.x
        transform.transform.translation.y = self.robot_odom.pose.pose.position.y
        transform.transform.translation.z = self.robot_odom.pose.pose.position.z
        transform.transform.rotation = self.robot_odom.pose.pose.orientation
        
        self.static_tf_bc.sendTransform(transform)
        
        if mode == "double_track":
            self.odom_double_track_publisher.publish(msg)
        elif mode == "single_track":
            self.odom_single_track_publisher.publish(msg)
        elif mode == "yaw_rate":
            self.odom_yaw_rate_publisher.publish(msg)
        else:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = FKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
