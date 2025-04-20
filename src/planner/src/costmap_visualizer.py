#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import numpy as np
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

class CostmapVisualizer(Node):
    """
    Visualizes costmap in RViz using marker arrays for better visualization
    """
    def __init__(self):
        super().__init__('costmap_visualizer')
        
        # Parameters
        self.declare_parameter('marker_scale', 0.1,
                              ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE,
                                                 description='Scale of the markers'))
        self.declare_parameter('marker_height', 0.1,
                              ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE,
                                                 description='Height of the markers'))
        self.declare_parameter('publish_rate', 2.0,
                              ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE,
                                                 description='Rate at which to publish visualization markers'))
        self.declare_parameter('cost_threshold', 10,
                              ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER,
                                                 description='Minimum cost to visualize (0-100)'))
        
        # Get parameters
        self.marker_scale = self.get_parameter('marker_scale').value
        self.marker_height = self.get_parameter('marker_height').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.cost_threshold = self.get_parameter('cost_threshold').value
        
        # Subscriber
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/traversability_costmap',
            self.costmap_callback,
            10
        )
        
        # Publishers
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/costmap_visualization',
            10
        )
        
        # Store costmap data
        self.costmap = None
        self.costmap_width = 0
        self.costmap_height = 0
        self.costmap_resolution = 0.1
        self.costmap_origin_x = 0.0
        self.costmap_origin_y = 0.0
        
        # Timer for updating visualization
        self.timer = self.create_timer(1.0 / self.publish_rate, self.update_visualization)
        
        self.get_logger().info('Costmap Visualizer started')
    
    def costmap_callback(self, msg):
        """Process costmap data"""
        self.costmap_width = msg.info.width
        self.costmap_height = msg.info.height
        self.costmap_resolution = msg.info.resolution
        self.costmap_origin_x = msg.info.origin.position.x
        self.costmap_origin_y = msg.info.origin.position.y
        
        # Reshape data to 2D array
        self.costmap = np.array(msg.data).reshape(self.costmap_height, self.costmap_width)
        
    def cost_to_color(self, cost):
        """Convert cost (0-100) to color"""
        # Green (0, 1, 0) for low cost to Red (1, 0, 0) for high cost
        if cost <= 0:
            return ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.2)
        elif cost >= 100:  # Obstacle
            return ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0)
        else:
            # Gradient from green to yellow to red
            if cost < 50:
                # Green to Yellow
                ratio = cost / 50.0
                return ColorRGBA(r=ratio, g=1.0, b=0.0, a=0.3 + 0.7 * (cost / 100.0))
            else:
                # Yellow to Red
                ratio = (cost - 50) / 50.0
                return ColorRGBA(r=1.0, g=1.0 - ratio, b=0.0, a=0.3 + 0.7 * (cost / 100.0))
    
    def update_visualization(self):
        """Update visualization markers"""
        if self.costmap is None:
            return
        
        # Create marker array
        marker_array = MarkerArray()
        
        # Delete previous markers
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        delete_marker.header.frame_id = "resistance_zones"
        delete_marker.header.stamp = self.get_clock().now().to_msg()
        marker_array.markers.append(delete_marker)
        
        # Optimize marker creation - use cubes for grid cells
        cube_marker = Marker()
        cube_marker.header.frame_id = "resistance_zones"
        cube_marker.header.stamp = self.get_clock().now().to_msg()
        cube_marker.type = Marker.CUBE_LIST
        cube_marker.action = Marker.ADD
        cube_marker.scale.x = self.marker_scale
        cube_marker.scale.y = self.marker_scale
        cube_marker.scale.z = self.marker_height
        cube_marker.pose.orientation.w = 1.0
        cube_marker.id = 1
        
        # Add points and colors for each cell above threshold
        for y in range(self.costmap_height):
            for x in range(self.costmap_width):
                cost = self.costmap[y, x]
                if cost > self.cost_threshold:
                    # Convert grid indices to world coordinates
                    world_x = self.costmap_origin_x + (x + 0.5) * self.costmap_resolution
                    world_y = self.costmap_origin_y + (y + 0.5) * self.costmap_resolution
                    
                    # Add point
                    point = Point(x=world_x, y=world_y, z=0.05)
                    cube_marker.points.append(point)
                    
                    # Add color based on cost
                    color = self.cost_to_color(cost)
                    cube_marker.colors.append(color)
        
        # Only add the cube marker if it has points
        if len(cube_marker.points) > 0:
            marker_array.markers.append(cube_marker)
            
        # Publish marker array
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = CostmapVisualizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()