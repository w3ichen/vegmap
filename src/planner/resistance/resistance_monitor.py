#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import math

class ResistanceMonitor(Node):
    """
    Monitors robot position and reduces velocity when in resistance zones.
    """
    
    def __init__(self):
        super().__init__('resistance_monitor')
        
        # Parameters
        self.declare_parameter('resistance_factor', 0.7)
        self.resistance_factor = self.get_parameter('resistance_factor').value
        
        # Define resistance zones (x, y, width, height)
        self.resistance_zones = [
            {'x': 3.0, 'y': 2.0, 'width': 1.0, 'height': 1.0},
            {'x': -4.0, 'y': 5.0, 'width': 1.0, 'height': 1.0},
            {'x': 6.0, 'y': -3.0, 'width': 1.0, 'height': 1.0},
            {'x': -2.0, 'y': -6.0, 'width': 1.0, 'height': 1.0},
            {'x': 8.0, 'y': 7.0, 'width': 1.0, 'height': 1.0}
        ]
        
        # Subscribe to robot position and velocity command
        self.pos_sub = self.create_subscription(
            Odometry, 
            '/a200_0000/platform/odom', 
            self.position_callback, 
            10
        )
        
        self.cmd_vel_sub = self.create_subscription(
            Twist, 
            '/a200_0000/cmd_vel', 
            self.cmd_vel_callback, 
            10
        )
        
        # Publisher for adjusted velocity
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/a200_0000/platform/cmd_vel_unstamped',
            10
        )
        
        # State
        self.robot_in_zone = False
        self.last_cmd_vel = Twist()
        
        self.get_logger().info('Resistance Monitor started')
        
    def position_callback(self, msg):
        """Process robot position data"""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Check if robot is in a resistance zone
        in_zone = False
        
        for zone in self.resistance_zones:
            if (abs(x - zone['x']) <= zone['width']/2 and 
                abs(y - zone['y']) <= zone['height']/2):
                in_zone = True
                break
        
        # Update state if changed
        if in_zone != self.robot_in_zone:
            self.robot_in_zone = in_zone
            if in_zone:
                self.get_logger().info('Robot entered resistance zone')
            else:
                self.get_logger().info('Robot left resistance zone')
            
            # Adjust velocity based on new state
            self.adjust_velocity()
    
    def cmd_vel_callback(self, msg):
        """Store the latest velocity command and adjust if needed"""
        self.last_cmd_vel = msg
        self.adjust_velocity()
    
    def adjust_velocity(self):
        """Apply resistance factor to velocity if in a zone"""
        adjusted_cmd = Twist()
        
        # Copy the last command
        adjusted_cmd.linear.x = self.last_cmd_vel.linear.x
        adjusted_cmd.linear.y = self.last_cmd_vel.linear.y
        adjusted_cmd.linear.z = self.last_cmd_vel.linear.z
        adjusted_cmd.angular.x = self.last_cmd_vel.angular.x
        adjusted_cmd.angular.y = self.last_cmd_vel.angular.y
        adjusted_cmd.angular.z = self.last_cmd_vel.angular.z
        
        # Apply resistance factor if in a zone
        if self.robot_in_zone:
            adjusted_cmd.linear.x *= self.resistance_factor
            adjusted_cmd.linear.y *= self.resistance_factor
            adjusted_cmd.linear.z *= self.resistance_factor
            adjusted_cmd.angular.x *= self.resistance_factor
            adjusted_cmd.angular.y *= self.resistance_factor
            adjusted_cmd.angular.z *= self.resistance_factor
        
        # Publish adjusted velocity
        self.cmd_vel_pub.publish(adjusted_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ResistanceMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()