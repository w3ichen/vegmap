#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import std_msgs.msg
from std_msgs.msg import Float32
from tf2_msgs.msg import TFMessage
import math
from planner_msgs.srv import UpdateCost

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class ResistanceMonitor(Node):
    """
    Monitors robot position and reduces velocity when in resistance zones.
    Each zone has a custom resistance factor.
    """
    
    def __init__(self):
        super().__init__('resistance_monitor')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer_init = self.create_timer(1.0, self.initialize_costmap_zones)
        
        
        # Parameters - default resistance factor (used as fallback)
        self.declare_parameter('default_resistance_factor', 0.5)
        self.default_resistance_factor = self.get_parameter('default_resistance_factor').value

        self.prev_position = None  # [x, y, z]
        self.prev_time = None      # seconds
        self.commanded_speed = 0.0 # Las commanded linear speed
        self.cost = 0.0            # traverse cost
        self.actual_speed = 0.0    # Actual speed of the robot

        self.update_cost_client = self.create_client(UpdateCost, '/veg_costmap/update')
        
        # Define resistance zones (x, y, width, height, resistance_factor)
        # Higher resistance_factor = stronger resistance (more reduction)
        self.resistance_zones = [
            # grass zones
            {'x': 3.0, 'y': 2.0, 'radius': 1.0, 'resistance_factor': 0.5},  # 50% reduction
            {'x': -4.0, 'y': 5.0, 'radius': 1.0, 'resistance_factor': 0.7}, # 70% reduction
            {'x': 6.0, 'y': -3.0, 'radius': 1.0, 'resistance_factor': 0.3}, # 30% reduction
            {'x': 1.0, 'y': -6.0, 'radius': 1.0, 'resistance_factor': 0.8}, # 80% reduction
            {'x': 8.0, 'y': 7.0, 'radius': 1.0, 'resistance_factor': 0.6},  # 60% reduction
            {'x': -3.0, 'y': 0.0, 'radius': 1.0, 'resistance_factor': 0.4},  # 40% reduction (center)
            {'x': 5.0, 'y': 5.0, 'radius': 1.0, 'resistance_factor': 0.6},  # 60% reduction (very high)
            {'x': -5.0, 'y': -5.0, 'radius': 1.0, 'resistance_factor': 0.2}, # 20% reduction (light)
            {'x': -6.0, 'y': 2.0, 'radius': 1.0, 'resistance_factor': 0.75}, # 75% reduction
            {'x': 7.0, 'y': -7.0, 'radius': 1.0, 'resistance_factor': 0.75}, # 75% reduction

            # tree zones - bigger radius so they stop before hitting the tree in simulation
            {'x': 0.0, 'y': 6.0, 'radius':0.1, 'resistance_factor': 0.8}, # 80% reduction (should technically be 100%)
            {'x':-7.0, 'y': 0.0, 'radius':0.1, 'resistance_factor': 0.8}, 
            {'x': 3.0, 'y':-2.0, 'radius':0.1, 'resistance_factor': 0.8}, 
        ]

        """ Create subscribers """
        # Commanded velocity subscription
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/a200_0000/cmd_vel',
            self.cmd_vel_callback,
            10  
        )

        # Ground truth pose subscription
        self.ground_truth_sub = self.create_subscription(
            TFMessage,
            '/model/a200_0000/robot/pose',
            self.position_callback,
            10  
        )
    
        # Cost publisher
        self.cost_pub = self.create_publisher(
            Float32,
            '/cost_traverse',
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
        self.active_zone_index = -1
        self.active_resistance_factor = self.default_resistance_factor
        self.timer = self.create_timer(0.1, self.publish_cost)

        # keeping track of robot position
        self.robot_x = 0.0
        self.robot_y = 0.0

    # publish grass zones
    def initialize_costmap_zones(self):

        return # Disable
        """Initialize all zones in the costmap at startup"""
        for i, zone in enumerate(self.resistance_zones):
            self.update_costmap(
                x=zone['x'],
                y=zone['y'],
                cost=int(255 * zone['resistance_factor']),
                obstacle_type=f'grass_{i+1}'
            )
        self.timer_init.cancel()  # Only run once

    def position_callback(self,msg):
        """process robot position from /model/a200_0000/robot/pose"""
        found_transform = False

        for transform in msg.transforms:
            if (transform.header.frame_id == 'resistance_zone' and   # in world frame
                transform.child_frame_id == 'a200_0000/robot'):       # get robot pose

                # extract current position
                current_position = [
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z
                ]

                # extract current time
                current_time = transform.header.stamp.sec + transform.header.stamp.nanosec / 1e9

                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                # t.header.stamp = transform.header.stamp
                # t.header.frame_id = 'map'
                t.header.frame_id = 'odom'
                t.child_frame_id = 'base_link'
                t.transform.translation.x = current_position[0]
                t.transform.translation.y = current_position[1]
                t.transform.translation.z = current_position[2]
                t.transform.rotation = transform.transform.rotation
                self.tf_broadcaster.sendTransform(t)


                wheel_transforms = [
                    {"name": "front_left_wheel", "x": 0.35, "y": 0.35, "z": 0.0},
                    {"name": "front_right_wheel", "x": 0.35, "y": -0.35, "z": 0.0},
                    {"name": "rear_left_wheel", "x": -0.35, "y": 0.35, "z": 0.0},
                    {"name": "rear_right_wheel", "x": -0.35, "y": -0.35, "z": 0.0}
                ]
                for wheel in wheel_transforms:
                    wt = TransformStamped()
                    wt.header.stamp = self.get_clock().now().to_msg()
                    wt.header.frame_id = 'base_link'
                    wt.child_frame_id = wheel["name"]
                    wt.transform.translation.x = wheel["x"]
                    wt.transform.translation.y = wheel["y"]
                    wt.transform.translation.z = wheel["z"]
                    wt.transform.rotation.w = 1.0  # Identity rotation
                    self.tf_broadcaster.sendTransform(wt)


                # calculate actual speed
                if self.prev_position is not None and self.prev_time is not None:
                    # difference in time
                    dt = current_time - self.prev_time

                    if dt > 0:
                        dx = current_position[0] - self.prev_position[0]
                        dy = current_position[1] - self.prev_position[1]

                        distance = math.sqrt(dx**2 + dy**2)

                        self.actual_speed = distance / dt

                        if self.commanded_speed > 0:
                            temp_cost = 1.0 - (self.actual_speed / self.commanded_speed)
                            self.cost = max(0.0, temp_cost)
                            self.get_logger().info(f"current cost: {temp_cost:.2f}")

                            if self.cost > 0.9:
                                self.update_nearest_tree_cost()
                                # when blocked by tree | here defined as cost > 0.9
                            
                # update previous position and time
                self.prev_position = current_position
                self.prev_time = current_time

                self.get_logger().debug(f'Robot at x={self.robot_x:.2f}, y={self.robot_y:.2f}')
                
                self.robot_x = current_position[0]
                self.robot_y = current_position[1]      
                
                found_transform = True
                break

        if found_transform:
            self.check_zones()

    def update_nearest_tree_cost(self):
        """find the nearest tree and update its cost in the costmap"""
        tree_indices = [10, 11, 12] #### lol there's got to be a better way to do this ####

        nearest_tree_idx = -1
        min_distance = float('inf')

        for idx in tree_indices:
            if idx < len(self.resistance_zones):
                tree = self.resistance_zones[idx]
                distance = math.sqrt((self.robot_x - tree['x'])**2 + (self.robot_y - tree['y'])**2)

                if distance < min_distance:
                    min_distance = distance
                    nearest_tree_idx = idx


        if nearest_tree_idx != -1 and min_distance < 2.0:
            tree = self.resistance_zones[nearest_tree_idx]
            self.get_logger().info(f'Robot is blocked - updating costmap for tree {nearest_tree_idx} at ({tree["x"]}, {tree["y"]})')    

            
            self.update_costmap(
                x=tree['x'],
                y=tree['y'],
                cost=255,
                obstacle_type=f'tree_{nearest_tree_idx - 9}'
            )
    



    def publish_cost(self):
        """ Publish current grid cost """
        if hasattr(self, 'actual_speed') and self.commanded_speed > 0: 
            # calculate cost
            # cost = 1.0 - (self.actual_speed / self.commanded_speed)
            # cost = max(0.0, cost)
            cost_msg = Float32()
            cost_msg.data = float(self.cost)
            self.cost_pub.publish(cost_msg)

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
                if (active_index < 10):
                    self.get_logger().info(f'Robot entered grass {active_index} ' +
                                        f'(factor: {active_factor:.2f})')
                else:
                    self.get_logger().info(f'Robot entered tree {active_index} ' +
                                        f'(factor: {active_factor:.2f})')
                
                ### Call service to update costmap
                self.update_costmap(
                    x=self.resistance_zones[active_index]['x'],
                    y=self.resistance_zones[active_index]['y'],
                    cost=int(255 * active_factor),
                    obstacle_type = f'grass_{active_index}'
                )

            else:
                self.get_logger().info('Robot left resistance zone')
            
            # Adjust velocity based on new state
            self.adjust_velocity()


    def update_costmap(self, x, y, cost, obstacle_type):
        if not self.update_cost_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Service /veg_costmap/update not available')
            return
        
        # create request
        request = UpdateCost.Request()
        request.x = float(x)
        request.y = float(y)

        cost = min(255, max(0, int(cost)))  # Ensure cost is within valid range
        request.cost = cost

        request.obstacle_type = obstacle_type

        future = self.update_cost_client.call_async(request)
        self.get_logger().info(f'Updating costmap for {obstacle_type} at ({x}, {y}) with cost {cost}')
    
    def cmd_vel_callback(self, msg):
        """Store the latest velocity command and adjust if needed"""
        self.last_cmd_vel = msg
        self.commanded_speed = math.sqrt(msg.linear.x**2 + msg.linear.y**2 + msg.linear.z**2)
        self.get_logger().info(f"Received cmd_vel: linear.x={msg.linear.x:.2f}, linear.y={msg.linear.y:.2f}")
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
            
            if self.active_resistance_factor >= 1.0:
                # For tree zones (factor ≥ 1.0), we want to completely stop
                reduction_factor = 0
            else:
                # For grass zones (factor < 1.0), we reduce by that factor
                reduction_factor = 1.0 - self.active_resistance_factor
            
            # Apply the calculated reduction factor
            adjusted_cmd.linear.x *= reduction_factor
            adjusted_cmd.linear.y *= reduction_factor
            adjusted_cmd.linear.z *= reduction_factor
            adjusted_cmd.angular.x *= reduction_factor
            adjusted_cmd.angular.y *= reduction_factor
            adjusted_cmd.angular.z *= reduction_factor
            
            self.get_logger().info(
                f'Reducing velocity in zone {self.active_zone_index} ' +
                f'(factor: {self.active_resistance_factor:.2f}, reduction: {reduction_factor:.2f}): ' +
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