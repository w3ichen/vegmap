#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import std_msgs.msg
from tf2_msgs.msg import TFMessage
import math

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
            {'x': 3.0, 'y': 2.0, 'radius': 2.0, 'resistance_factor': 0.5},  # 50% reduction
            {'x': -4.0, 'y': 5.0, 'radius': 1.0, 'resistance_factor': 0.7}, # 70% reduction
            {'x': 6.0, 'y': -3.0, 'radius': 1.0, 'resistance_factor': 0.3}, # 30% reduction
            {'x': 1.0, 'y': -6.0, 'radius': 3.0, 'resistance_factor': 0.8}, # 80% reduction
            {'x': 8.0, 'y': 7.0, 'radius': 1.0, 'resistance_factor': 0.6},  # 60% reduction
            {'x': 0.0, 'y': 0.0, 'radius': 1.0, 'resistance_factor': 0.4},  # 40% reduction (center)
            {'x': 5.0, 'y': 5.0, 'radius': 1.0, 'resistance_factor': 0.9},  # 90% reduction (very high)
            {'x': -5.0, 'y': -5.0, 'radius': 2.0, 'resistance_factor': 0.2}, # 20% reduction (light)
            {'x': -6.0, 'y': 2.0, 'radius': 1.0, 'resistance_factor': 0.75}, # 75% reduction
            {'x': 7.0, 'y': -7.0, 'radius': 1.0, 'resistance_factor': 0.85}, # 85% reduction
        ]

        # using ground truth position for position
        self.pos_sub = self.create_subscription(
            TFMessage,
            '/model/a200_0000/robot/pose',
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

        # keeping track of robot position
        self.robot_x = 0.0
        self.robot_y = 0.0
    
        self.get_logger().info('Circular Resistance Monitor started')
        self.get_logger().info(f'Monitoring {len(self.resistance_zones)} circular resistance zones')

    def position_callback(self,msg):
        """process robot position from /model/a200_0000/robot/pose"""
        found_transform = False

        for transform in msg.transforms:
            if (transform.header.frame_id == 'resistance_zones' and
                transform.child_frame_id == 'a200_0000/robot'):

                # set robot position
                self.robot_x = transform.transform.translation.x
                self.robot_y = transform.transform.translation.y

                self.get_logger().debug(f'Robot at x={self.robot_x:.2f}, y={self.robot_y:.2f}')
                found_transform = True
                break

        if found_transform:
            self.check_zones()


    def check_zones(self):
        """checking if robot is in a circular resistance zone"""
        in_zone = False
        active_index = -1
        active_factor = self.default_resistance_factor
        
        for i, zone in enumerate(self.resistance_zones):
            # Calculate distance from robot to zone center
            distance = math.sqrt((self.robot_x - zone['x'])**2 + (self.robot_y - zone['y'])**2)
            
            # Check if robot is within the circle radius
            if distance <= zone['radius']:
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
                self.get_logger().info(f'Robot entered circular resistance zone {active_index} ' +
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