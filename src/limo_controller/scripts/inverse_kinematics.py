#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import math


class IKNode(Node):
    def __init__(self):
        super().__init__("ik_node")

        self.declare_parameter("mode", "car")
        self.mode = self.get_parameter("mode").get_parameter_value().string_value

        self.timer = self.create_timer(100, self.timer_callback)
        self.steering_pub = self.create_publisher(
            Float64MultiArray, "/steering_controller/commands", 10
        )
        self.velocity_pub = self.create_publisher(
            Float64MultiArray, "/velocity_controller/commands", 10
        )

        self.create_subscription(Twist, "cmd_vel", self.cmd_callback, 10)

        self.l = 0.2  # distance between front and rear wheels
        self.wheel_radius = 0.045  # wheel radius
        self.track = 0.14  # distance between left and right wheels
        
        self.get_logger().warn("Inverse Kinematics Node has been initialized.")

    def cmd_callback(self, msg: Twist):
        # Log received command
        self.get_logger().info(f"Linear: {msg.linear.x}, Angular: {msg.angular.z}")
        
        # Retrieve control mode parameter
        mode = self.get_parameter("mode").get_parameter_value().string_value

        # If there's no forward motion, stop steering and velocity.
        if msg.linear.x == 0:
            self.set_steering_angle(0, 0)
            self.set_velocity(0, 0)
            return

        # Compute wheel speed common to both modes
        wheel_speed = msg.linear.x / self.wheel_radius

        # Compute the basic steering angle based on the input twist message
        base_steering_angle = math.atan(self.l * msg.angular.z / msg.linear.x)

        if mode == "bicycle":
            # In bicycle mode, both steering joints follow the same angle.
            left_angle = right_angle = base_steering_angle

        elif mode == "car":
            # In car mode, calculate individual wheel angles.
            tan_steering = math.tan(base_steering_angle)
            left_angle = math.atan((self.l * tan_steering) / (self.l + 0.5 * self.track * tan_steering))
            right_angle = math.atan((self.l * tan_steering) / (self.l - 0.5 * self.track * tan_steering))
            
        else:
            self.get_logger().warn(f"Unknown mode: {mode}. Ignoring command.")
            return

        # Set computed steering angles and wheel speeds
        self.set_steering_angle(left_angle, right_angle)
        self.set_velocity(wheel_speed, wheel_speed)


    def set_steering_angle(self, left_steering_hinge_wheel, right_steering_hinge_wheel):
        msg = Float64MultiArray()
        msg.layout.data_offset = 0
        msg.data = [float(left_steering_hinge_wheel), float(right_steering_hinge_wheel)]
        self.steering_pub.publish(msg)

    def set_velocity(self, rear_left_wheel, rear_right_wheel):
        msg = Float64MultiArray()
        msg.layout.data_offset = 0
        msg.data = [float(rear_left_wheel), float(rear_right_wheel)]
        self.velocity_pub.publish(msg)

    def timer_callback(self):

        # if self.mode == "bicycle":
        #     self.get_logger().info("Bicycle mode")
        # elif self.mode == "car":
        #     self.get_logger().info("Car mode")
        # else:
        #     self.get_logger().info("Unknown mode")
        pass


def main(args=None):
    rclpy.init(args=args)
    node = IKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
