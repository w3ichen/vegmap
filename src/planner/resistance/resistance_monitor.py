#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import std_msgs.msg

class ResistanceMonitor(Node):
    """
    Monitors robot position and reduces velocity when in resistance zones.
    Each zone has a custom resistance factor.
    """
    
    def __init__(self):
        super().__init__('resistance_monitor')
        
        # Parameters - default resistance factor (used as fallback)
        self.declare_parameter('default_resistance_factor', 0.5)
        self.default_resistance_factor = self.get_parameter('default_resistance_factor').value
        
        # Define resistance zones (x, y, width, height, resistance_factor)
        # Higher resistance_factor = stronger resistance (more reduction)
        self.resistance_zones = [
            # Original zones with variable resistance
            {'x': 3.0, 'y': 2.0, 'width': 1.0, 'height': 1.0, 'resistance_factor': 0.5},  # 50% reduction
            {'x': -4.0, 'y': 5.0, 'width': 1.0, 'height': 1.0, 'resistance_factor': 0.7}, # 70% reduction
            {'x': 6.0, 'y': -3.0, 'width': 1.0, 'height': 1.0, 'resistance_factor': 0.3}, # 30% reduction
            {'x': -2.0, 'y': -6.0, 'width': 1.0, 'height': 1.0, 'resistance_factor': 0.8}, # 80% reduction
            {'x': 8.0, 'y': 7.0, 'width': 1.0, 'height': 1.0, 'resistance_factor': 0.6},  # 60% reduction
            
            # New zones with different sizes and resistance factors
            {'x': 0.0, 'y': 0.0, 'width': 2.0, 'height': 2.0, 'resistance_factor': 0.4},  # 40% reduction (center)
            {'x': 5.0, 'y': 5.0, 'width': 1.5, 'height': 1.5, 'resistance_factor': 0.9},  # 90% reduction (very high)
            {'x': -5.0, 'y': -5.0, 'width': 1.5, 'height': 1.5, 'resistance_factor': 0.2}, # 20% reduction (light)
            {'x': -6.0, 'y': 2.0, 'width': 1.2, 'height': 1.2, 'resistance_factor': 0.75}, # 75% reduction
            {'x': 7.0, 'y': -7.0, 'width': 1.8, 'height': 1.8, 'resistance_factor': 0.85}  # 85% reduction
        ]
        
        # Subscribe to robot position and velocity command
        self.pos_sub = self.create_subscription(
            Odometry, 
            'a200_0000/platform/odom/filtered', 
            self.position_callback, 
            10
        )
        
        self.cmd_vel_sub = self.create_subscription(
            Twist, 
            'a200_0000/cmd_vel', 
            self.cmd_vel_callback, 
            10
        )
        
        # Publisher for adjusted velocity
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            'a200_0000/platform/cmd_vel_unstamped',
            10
        )
        
        # State
        self.robot_in_zone = False
        self.last_cmd_vel = Twist()
        self.active_zone_index = -1
        self.active_resistance_factor = self.default_resistance_factor
    
        self.get_logger().info('Enhanced Resistance Monitor started')
        self.get_logger().info(f'Monitoring {len(self.resistance_zones)} resistance zones')
    
    def position_callback(self, msg):
        """Process robot position data"""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        self.get_logger().debug(f'Robot at x={x:.2f}, y={y:.2f}')
        
        # Check if robot is in a resistance zone
        in_zone = False
        active_index = -1
        active_factor = self.default_resistance_factor
        
        for i, zone in enumerate(self.resistance_zones):
            if (abs(x - zone['x']) <= zone['width']/2 and 
                abs(y - zone['y']) <= zone['height']/2):
                in_zone = True
                active_index = i
                active_factor = zone['resistance_factor']
                break
        
        # Update state if changed
        if (in_zone != self.robot_in_zone or 
            active_index != self.active_zone_index or
            active_factor != self.active_resistance_factor):
            
            self.robot_in_zone = in_zone
            self.active_zone_index = active_index
            self.active_resistance_factor = active_factor
            
            if in_zone:
                self.get_logger().info(f'Robot entered resistance zone {active_index} ' +
                                     f'(factor: {active_factor:.2f})')
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
            before_x = adjusted_cmd.linear.x
            before_y = adjusted_cmd.linear.y
            
            # Apply the active zone's resistance factor
            adjusted_cmd.linear.x *= self.active_resistance_factor
            adjusted_cmd.linear.y *= self.active_resistance_factor
            adjusted_cmd.linear.z *= self.active_resistance_factor
            adjusted_cmd.angular.x *= self.active_resistance_factor
            adjusted_cmd.angular.y *= self.active_resistance_factor
            adjusted_cmd.angular.z *= self.active_resistance_factor
            
            self.get_logger().info(
                f'Reducing velocity in zone {self.active_zone_index} ' +
                f'(factor: {self.active_resistance_factor:.2f}): ' +
                f'{before_x:.2f}->{adjusted_cmd.linear.x:.2f}, ' +
                f'{before_y:.2f}->{adjusted_cmd.linear.y:.2f}'
            )
        
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