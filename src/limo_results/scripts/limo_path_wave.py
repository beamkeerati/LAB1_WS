#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class LimoPathNode(Node):
    def __init__(self):
        super().__init__('limo_path_node')
        
        # Create a publisher for /cmd_vel
        self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        
        # Timer to send commands at regular intervals (e.g., every 0.1 seconds)
        self.timer = self.create_timer(0.1, self.send_cmd_vel)
        
        # State tracking variables
        self.state = "left"  
        self.time_start = self.get_clock().now()  # Track the time for the first movement

    def send_cmd_vel(self):
        # Create a Twist message
        msg = Twist()
        
        if self.state == "left":
            msg.linear.x = 0.3
            msg.angular.z = -0.3
            elapsed_time = self.get_clock().now() - self.time_start
            if elapsed_time.nanoseconds > 8 * 1e9:
                self.state = "right" 
                self.time_start = self.get_clock().now()  # Reset timer for turning
                
        elif self.state == "right":
            msg.linear.x = 0.3
            msg.angular.z = 0.3
            elapsed_time = self.get_clock().now() - self.time_start
            if elapsed_time.nanoseconds > 8 * 1e9: 
                self.state = "left" 
                self.time_start = self.get_clock().now()  # Reset timer for straight movement
        
        # Publish the message
        self.cmd_vel_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LimoPathNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
