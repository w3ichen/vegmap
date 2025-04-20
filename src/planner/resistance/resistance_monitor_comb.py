#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import std_msgs.msg
from tf2_msgs.msg import TFMessage
import math

class ResistanceMonitorComb(Node):
    """
    Monitors robot position and reduces velocity when in resistance zones.
    Each zone has a custom resistance factor based on transparency values from SDF.
    """
    
    def __init__(self):
        super().__init__('resistance_monitor_comb')
        
        # Parameters - default resistance factor (used as fallback)
        self.declare_parameter('default_resistance_factor', 0.5)
        self.default_resistance_factor = self.get_parameter('default_resistance_factor').value
        
        # Define resistance zones from resistance_comb.sdf
        # Note: resistance_factor values adjusted based on updated transparency values
        self.resistance_zones = [
            {'x': 3.0, 'y': 2.0, 'radius': 2.0, 'resistance_factor': 0.5},  # Zone 1
            {'x': -4.0, 'y': 5.0, 'radius': 1.0, 'resistance_factor': 0.7}, # Zone 2
            {'x': 6.0, 'y': -3.0, 'radius': 1.0, 'resistance_factor': 0.3}, # Zone 3
            {'x': 1.0, 'y': -6.0, 'radius': 3.0, 'resistance_factor': 0.8}, # Zone 4
            {'x': 8.0, 'y': 7.0, 'radius': 1.0, 'resistance_factor': 0.6},  # Zone 5
            {'x': 0.0, 'y': 0.0, 'radius': 1.0, 'resistance_factor': 0.4},  # Zone 6
            {'x': 5.0, 'y': 5.0, 'radius': 1.0, 'resistance_factor': 0.9},  # Zone 7
            {'x': -5.0, 'y': -5.0, 'radius': 2.0, 'resistance_factor': 0.2}, # Zone 8
            {'x': -6.0, 'y': 2.0, 'radius': 1.0, 'resistance_factor': 0.75}, # Zone 9
            {'x': 7.0, 'y': -7.0, 'radius': 1.0, 'resistance_factor': 0.85}, # Zone 10
        ]

        # Add tree and bush model positions for visualization/logging
        self.vegetation_models = [
            {'name': 'tree_1_mesh', 'x': 0.0, 'y': 6.0, 'type': 'tree'},
            {'name': 'tree_2_mesh', 'x': -7.0, 'y': -1.0, 'type': 'tree'},
            {'name': 'tree_3_mesh', 'x': 3.0, 'y': -2.0, 'type': 'tree'},
            {'name': 'oak_tree', 'x': 8.0, 'y': 2.0, 'type': 'tree'},
            {'name': 'tree_8', 'x': -5.0, 'y': -7.0, 'type': 'tree'},
            {'name': 'bush_0_1', 'x': 2.0, 'y': 9.0, 'type': 'bush'},
            {'name': 'bush_0_2', 'x': -2.0, 'y': 4.0, 'type': 'bush'},
            {'name': 'bush_2', 'x': 9.0, 'y': -9.0, 'type': 'bush'},
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
        self.near_vegetation = False
        self.closest_vegetation = None

        # keeping track of robot position
        self.robot_x = 0.0
        self.robot_y = 0.0
    
        self.get_logger().info('Circular Resistance Monitor with Vegetation started')
        self.get_logger().info(f'Monitoring {len(self.resistance_zones)} resistance zones and {len(self.vegetation_models)} vegetation models')

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
            # self.check_vegetation_proximity()

    # def check_vegetation_proximity(self):
    #     """Check if robot is near any vegetation models"""
    #     closest_distance = float('inf')
    #     closest_model = None
    #     proximity_threshold = 1.5  # Distance to trigger vegetation proximity warning
        
    #     for veg in self.vegetation_models:
    #         # Calculate distance from robot to vegetation
    #         distance = math.sqrt((self.robot_x - veg['x'])**2 + (self.robot_y - veg['y'])**2)
            
    #         # Update closest vegetation
    #         if distance < closest_distance:
    #             closest_distance = distance
    #             closest_model = veg
        
    #     # Check if we're near vegetation
    #     near_veg = closest_distance < proximity_threshold
        
    #     # Log if state changed
    #     if near_veg != self.near_vegetation or closest_model != self.closest_vegetation:
    #         self.near_vegetation = near_veg
    #         self.closest_vegetation = closest_model
            
    #         if near_veg:
    #             self.get_logger().info(f"Robot near {closest_model['type']} model '{closest_model['name']}' " +
    #                                  f"(distance: {closest_distance:.2f}m)")

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
            adjusted_cmd.linear.x *= (1.0 - self.active_resistance_factor)
            adjusted_cmd.linear.y *= (1.0 - self.active_resistance_factor)
            adjusted_cmd.linear.z *= (1.0 - self.active_resistance_factor)
            adjusted_cmd.angular.x *= (1.0 - self.active_resistance_factor)
            adjusted_cmd.angular.y *= (1.0 - self.active_resistance_factor)
            adjusted_cmd.angular.z *= (1.0 - self.active_resistance_factor)
            
            self.get_logger().info(
                f'Reducing velocity in zone {self.active_zone_index+1} ' +
                f'(factor: {self.active_resistance_factor:.2f}): ' +
                f'{before_x:.2f}->{adjusted_cmd.linear.x:.2f}, ' +
                f'{before_y:.2f}->{adjusted_cmd.linear.y:.2f}'
            )
        
        # Publish adjusted velocity
        self.cmd_vel_pub.publish(adjusted_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ResistanceMonitorComb()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()