#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from planner_msgs.srv import UpdateCost
from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage
import math

class TraverseCostUpdater(Node):
    """
    Subscribes to the traverse cost and updates the costmap layer
    to reflect the actual traversability of the terrain
    """
    
    def __init__(self):
        super().__init__('traverse_cost_updater')
        
        # Subscribe to the cost traverse topic
        self.cost_sub = self.create_subscription(
            Float32, 
            '/cost_traverse', 
            self.cost_callback, 
            10
        )
        
        # Subscribe to robot pose to get current position
        self.pose_sub = self.create_subscription(
            TFMessage,
            '/model/a200_0000/robot/pose',
            self.pose_callback,
            10
        )
        
        # Client for updating the costmap
        self.update_cost_client = self.create_client(
            UpdateCost, 
            '/veg_costmap/update'
        )
        
        # Current robot position
        self.robot_x = 0.0
        self.robot_y = 0.0
        
        # Current traversability cost
        self.current_cost = 0.0
        
        # Update timer - updates costmap cells around the robot periodically
        self.update_timer = self.create_timer(0.5, self.update_costmap)

        self.get_logger().info('Traverse Cost Updater started')
        
    def pose_callback(self, msg):
        """Process robot position from TF message"""
        for transform in msg.transforms:
            if (transform.header.frame_id == 'resistance_zones' and
                transform.child_frame_id == 'a200_0000/robot'):
                
                # Set robot position
                self.robot_x = transform.transform.translation.x
                self.robot_y = transform.transform.translation.y
                break
    
    def cost_callback(self, msg):
        """Process traverse cost message"""
        # Scale cost from 0-1 range to costmap values (0-254)
        self.current_cost = min(254.0, msg.data * 254.0)
        self.get_logger().debug(f'Received cost: {self.current_cost:.2f}')
        
    def update_costmap(self):
        """Update the costmap with current traversability cost"""
        if not self.update_cost_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Costmap update service not available')
            return
            
        # Area around the robot to update (trail behind)
        # Create a small grid of updates centered on the robot
        # with a trail of cells behind in the direction of travel
        
        # Simple update at robot position for now
        request = UpdateCost.Request()
        request.x = float(self.robot_x)
        request.y = float(self.robot_y)
        request.cost = int(self.current_cost)
        
        self.update_cost_client.call_async(request).add_done_callback(
            lambda future: self.update_callback(future)
        )
        
    def update_callback(self, future):
        """Process the result of the costmap update"""
        try:
            response = future.result()
            if not response.success:
                self.get_logger().warn(f'Failed to update costmap: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = TraverseCostUpdater()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()