#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import std_msgs.msg
from tf2_msgs.msg import TFMessage
import math
import time

class DenseResistanceMonitor(Node):
    """
    Monitors robot position and reduces velocity when in resistance zones.
    Enhanced version with more dense zones and improved features:
    - 30 resistance zones with varying resistance factors
    - Smooth transition between zones
    - Dynamic resistance based on robot velocity
    - Logging of zone transitions
    """
    
    def __init__(self):
        super().__init__('dense_resistance_monitor')
        
        # Parameters - default resistance factor (used as fallback)
        self.declare_parameter('default_resistance_factor', 0.5)
        self.default_resistance_factor = self.get_parameter('default_resistance_factor').value
        
        # Define resistance zones (x, y, radius, resistance_factor)
        # Higher resistance_factor = stronger resistance (more reduction)
        self.resistance_zones = [
            # Original zones with variable resistance
            {'x': 3.0, 'y': 2.0, 'radius': 1.5, 'resistance_factor': 0.5},  # 50% reduction
            {'x': -4.0, 'y': 5.0, 'radius': 2.0, 'resistance_factor': 0.7}, # 70% reduction
            {'x': 6.0, 'y': -3.0, 'radius': 0.8, 'resistance_factor': 0.3}, # 30% reduction
            
            # Additional zones
            {'x': 2.0, 'y': 4.0, 'radius': 0.6, 'resistance_factor': 0.55}, # 55% reduction
            {'x': 4.0, 'y': 5.0, 'radius': 1.5, 'resistance_factor': 0.65}, # 65% reduction
            {'x': 0.0, 'y': 3.0, 'radius': 1.5, 'resistance_factor': 0.45}, # 45% reduction
            {'x': -2.0, 'y': 3.0, 'radius': 0.5, 'resistance_factor': 0.35}, # 35% reduction
            {'x': -3.0, 'y': 0.0, 'radius': 0.6, 'resistance_factor': 0.25}, # 25% reduction
            {'x': 2.0, 'y': -1.0, 'radius': 2.5, 'resistance_factor': 0.95}, # 95% reduction (extreme)
            {'x': -2.0, 'y': -2.0, 'radius': 0.5, 'resistance_factor': 0.78}, # 78% reduction
            {'x': -4.0, 'y': -4.0, 'radius': 0.7, 'resistance_factor': 0.42}, # 42% reduction
            {'x': 3.0, 'y': -4.0, 'radius': 0.5, 'resistance_factor': 0.63}, # 63% reduction
            {'x': -1.0, 'y': -4.0, 'radius': 2.0, 'resistance_factor': 0.82}, # 82% reduction
            {'x': -8.0, 'y': -3.0, 'radius': 0.8, 'resistance_factor': 0.15}, # 15% reduction (very light)
            {'x': -6.0, 'y': -7.0, 'radius': 0.6, 'resistance_factor': 0.38}, # 38% reduction
            {'x': 2.0, 'y': -8.0, 'radius': 0.6, 'resistance_factor': 0.68}, # 68% reduction
            {'x': 8.0, 'y': 0.0, 'radius': 0.7, 'resistance_factor': 0.72}, # 72% reduction
            {'x': 9.0, 'y': 3.0, 'radius': 0.5, 'resistance_factor': 0.58}, # 58% reduction
            {'x': -2.0, 'y': 7.0, 'radius': 0.5, 'resistance_factor': 0.47}, # 47% resistance
            {'x': -8.0, 'y': 8.0, 'radius': 0.6, 'resistance_factor': 0.33}, # 33% reduction
            {'x': 5.0, 'y': 9.0, 'radius': 1.2, 'resistance_factor': 0.88}, # 88% reduction
            {'x': 7.0, 'y': 6.0, 'radius': 1.0, 'resistance_factor': 0.93}, # 93% reduction
            {'x': 9.0, 'y': -9.0, 'radius': 0.5, 'resistance_factor': 0.28}, # 28% reduction
            
            # Additional large zones for a dense environment
            {'x': 1.0, 'y': -6.0, 'radius': 3.0, 'resistance_factor': 0.8}, # 80% reduction
            {'x': 8.0, 'y': 7.0, 'radius': 1.7, 'resistance_factor': 0.6},  # 60% reduction
            {'x': 0.0, 'y': 0.0, 'radius': 1.8, 'resistance_factor': 0.4},  # 40% reduction (center)
            {'x': 5.0, 'y': 5.0, 'radius': 1.6, 'resistance_factor': 0.9},  # 90% reduction (very high)
            {'x': -5.0, 'y': -5.0, 'radius': 2.0, 'resistance_factor': 0.2}, # 20% reduction (light)
            {'x': -6.0, 'y': 2.0, 'radius': 1.7, 'resistance_factor': 0.75}, # 75% reduction
            {'x': 7.0, 'y': -7.0, 'radius': 1.5, 'resistance_factor': 0.85}, # 85% reduction
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
        self.transition_time = None
        self.previous_zone_index = -1

        # keeping track of robot position and velocity
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_linear_speed = 0.0
        
        # Add transition buffer for smoother transitions
        self.transition_buffer = 0.2  # 20% buffer for smoother transitions
    
        self.get_logger().info('Dense Resistance Monitor started')
        self.get_logger().info(f'Monitoring {len(self.resistance_zones)} circular resistance zones')

    def position_callback(self, msg):
        """process robot position from /model/a200_0000/robot/pose"""
        found_transform = False

        for transform in msg.transforms:
            if ((transform.header.frame_id == 'resistance_zones_2' or
                 transform.header.frame_id == 'resistance_zone_2') and
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
        """checking if robot is in a circular resistance zone with enhanced features"""
        in_zone = False
        active_index = -1
        active_factor = self.default_resistance_factor
        min_distance_ratio = float('inf')
        
        # Track multiple zone influences
        zone_influences = []
        
        for i, zone in enumerate(self.resistance_zones):
            # Calculate distance from robot to zone center
            distance = math.sqrt((self.robot_x - zone['x'])**2 + (self.robot_y - zone['y'])**2)
            
            # Calculate distance ratio (1.0 = at edge, 0.0 = at center)
            distance_ratio = distance / zone['radius'] if zone['radius'] > 0 else float('inf')
            
            # Check if robot is within or near the circle radius
            if distance_ratio <= 1.0 + self.transition_buffer:
                # If within the transition buffer, add to influences
                influence = 0.0
                
                if distance_ratio <= 1.0:
                    # Inside zone - full influence based on position in zone
                    influence = 1.0 - (distance_ratio * 0.2)  # Slightly stronger at center
                else:
                    # In buffer zone - partial influence
                    buffer_position = (distance_ratio - 1.0) / self.transition_buffer
                    influence = 1.0 - buffer_position
                
                zone_influences.append({
                    'index': i,
                    'factor': zone['resistance_factor'],
                    'influence': influence,
                    'distance_ratio': distance_ratio
                })
                
                # Keep track of the most influential zone (closest to center)
                if distance_ratio < min_distance_ratio:
                    in_zone = True
                    active_index = i
                    active_factor = zone['resistance_factor']
                    min_distance_ratio = distance_ratio
        
        # Determine combined resistance factor if multiple zones are influencing
        if len(zone_influences) > 1:
            # Calculate weighted resistance factor based on influences
            total_influence = sum(z['influence'] for z in zone_influences)
            if total_influence > 0:
                weighted_factor = sum(z['factor'] * z['influence'] for z in zone_influences) / total_influence
                active_factor = weighted_factor
                self.get_logger().debug(f'Multiple zones influencing: final resistance = {active_factor:.2f}')
        
        # Update state if changed
        zone_changed = (active_index != self.active_zone_index)
        
        if (in_zone != self.robot_in_zone or 
            zone_changed or
            abs(active_factor - self.active_resistance_factor) > 0.05):
            
            self.robot_in_zone = in_zone
            self.previous_zone_index = self.active_zone_index
            self.active_zone_index = active_index
            self.active_resistance_factor = active_factor
            
            if zone_changed:
                # Track transition time for logging and analytics
                current_time = time.time()
                if self.transition_time is not None:
                    time_in_previous_zone = current_time - self.transition_time
                    if self.previous_zone_index >= 0:
                        self.get_logger().info(
                            f'Zone transition: {self.previous_zone_index} → {active_index}, ' +
                            f'Time in previous zone: {time_in_previous_zone:.2f}s'
                        )
                self.transition_time = current_time
            
            if in_zone:
                if len(zone_influences) > 1:
                    self.get_logger().info(
                        f'Robot in multiple zones. Primary: {active_index} ' +
                        f'(combined factor: {active_factor:.2f})'
                    )
                else:
                    self.get_logger().info(
                        f'Robot in zone {active_index} ' +
                        f'(factor: {active_factor:.2f})'
                    )
            else:
                self.get_logger().info('Robot left all resistance zones')
            
            # Adjust velocity based on new state
            self.adjust_velocity()
    
    def cmd_vel_callback(self, msg):
        """Store the latest velocity command and adjust if needed"""
        self.last_cmd_vel = msg
        
        # Calculate speed for dynamic resistance adjustments
        self.robot_linear_speed = math.sqrt(
            msg.linear.x**2 + 
            msg.linear.y**2 + 
            msg.linear.z**2
        )
        
        self.adjust_velocity()
    
    def adjust_velocity(self):
        """Apply resistance factor to velocity if in a zone with enhancements"""
        adjusted_cmd = Twist()
        
        # Copy the last command
        adjusted_cmd.linear.x = self.last_cmd_vel.linear.x
        adjusted_cmd.linear.y = self.last_cmd_vel.linear.y
        adjusted_cmd.linear.z = self.last_cmd_vel.linear.z
        adjusted_cmd.angular.x = self.last_cmd_vel.angular.x
        adjusted_cmd.angular.y = self.last_cmd_vel.angular.y
        adjusted_cmd.angular.z = self.last_cmd_vel.angular.z
        
        # Apply dynamic resistance factor if in a zone
        if self.robot_in_zone:
            before_x = adjusted_cmd.linear.x
            before_y = adjusted_cmd.linear.y
            
            # Calculate dynamic resistance - higher speed = slightly more resistance
            # Max increase of 15% at high speeds
            speed_modifier = min(0.15 * (self.robot_linear_speed / 2.0), 0.15)
            dynamic_factor = max(0.05, self.active_resistance_factor - speed_modifier)
            
            # Apply the dynamic resistance factor
            adjusted_cmd.linear.x *= dynamic_factor
            adjusted_cmd.linear.y *= dynamic_factor
            adjusted_cmd.linear.z *= dynamic_factor
            
            # Apply less reduction to angular velocity (rotation) - make it easier to turn in high resistance
            angular_factor = dynamic_factor + ((1.0 - dynamic_factor) * 0.3)
            adjusted_cmd.angular.x *= angular_factor
            adjusted_cmd.angular.y *= angular_factor
            adjusted_cmd.angular.z *= angular_factor
            
            # Only log significant velocity changes
            if abs(before_x - adjusted_cmd.linear.x) > 0.05 or abs(before_y - adjusted_cmd.linear.y) > 0.05:
                self.get_logger().info(
                    f'Reducing velocity in zone {self.active_zone_index} ' +
                    f'(dynamic factor: {dynamic_factor:.2f}): ' +
                    f'{before_x:.2f}->{adjusted_cmd.linear.x:.2f}, ' +
                    f'{before_y:.2f}->{adjusted_cmd.linear.y:.2f}'
                )
        
        # Publish adjusted velocity
        self.cmd_vel_pub.publish(adjusted_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = DenseResistanceMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()