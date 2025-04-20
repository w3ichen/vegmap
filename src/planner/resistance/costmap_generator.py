#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Float32
from tf2_msgs.msg import TFMessage
import math
import xml.etree.ElementTree as ET
import os
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

class CostMapGenerator(Node):
    """
    Generates estimated costmap based on world geometry
    Updates costmap based on robot traversal cost
    """

    def __init__(self):
        super().__init__('costmap_generator')

        # parameters
        self.declare_parameter('map_resolution', 0.1,
                              ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE,
                                                 description='Resolution of the costmap in meters per cell'))
        self.declare_parameter('map_width', 20.0,
                              ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE,
                                                 description='Width of the map in meters'))
        self.declare_parameter('map_height', 20.0,
                              ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE,
                                                 description='Height of the map in meters'))
        self.declare_parameter('map_origin_x', -10.0,
                              ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE,
                                                 description='X coordinate of the map origin'))
        self.declare_parameter('map_origin_y', -10.0,
                              ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE,
                                                 description='Y coordinate of the map origin'))
        

        #### amount of noise stddev ###
        # position noise +/- 5%
        self.declare_parameter('noise_factor_position', 0.05,
                              ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE,
                                                 description='Noise factor for the initial costmap'))
        
        # solid obstacle cost noise +/- 5% (90 - 100) - since solid obstacles are provably more untraversable
        self.declare_parameter('noise_factor_obstacle', 0.05,
                                ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE,
                                                    description='Noise factor for the obstacles'))
        
        # resistance cost noise +/- 10% - since vegetation noise is more severe
        self.declare_parameter('noise_factor_resistance', 0.10,
                                ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE,
                                                    description='Noise factor for the resistance zones'))
        ###
        
        
        self.declare_parameter('learning_rate', 0.3,
                              ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE,
                                                 description='Learning rate for updating the costmap'))
        self.declare_parameter('sdf_path', '',
                              ParameterDescriptor(type=ParameterType.PARAMETER_STRING,
                                                 description='Path to the SDF file'))
        

        # iniitialize parameters
        self.map_resolution = self.get_parameter('map_resolution').value
        self.map_width = self.get_parameter('map_width').value
        self.map_height = self.get_parameter('map_height').value
        self.map_origin_x = self.get_parameter('map_origin_x').value
        self.map_origin_y = self.get_parameter('map_origin_y').value
        # self.noise_factor = self.get_parameter('noise_factor').value
        self.noise_factor_position = self.get_parameter('noise_factor_position').value
        self.noise_factor_obstacle = self.get_parameter('noise_factor_obstacle').value
        self.noise_factor_resistance = self.get_parameter('noise_factor_resistance').value

        self.learning_rate = self.get_parameter('learning_rate').value
        self.sdf_path = self.get_parameter('sdf_path').value

        # initialize costmap
        self.costmap_width = int(self.map_width / self.map_resolution)
        self.costmap_height = int(self.map_height / self.map_resolution)
        self.costmap = np.zeros((self.costmap_height, self.costmap_width), dtype=np.float32)

        # robot position tracking
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.current_traversal_cost = 0.0

        # subscribers
        ## ground truth robot pose
        self.pose_sub = self.create_subscription(
            TFMessage,
            '/model/a200_0000/robot/pose',
            self.position_callback,
            10
        )

        ## traversability cost
        self.cost_sub = self.create_subscription(
            Float32,
            '/cost_traverse',
            self.cost_callback,
            10
        )

        # publishers
        ## costmap
        self.costmap_pub = self.create_publisher(
            OccupancyGrid,
            '/traversability_costmap',
            10
        )

        # SDF stuff
        self.resistance_zones = []
        self.obstacles = []
        self.load_sdf_world()


        # generate initial costmap
        self.generate_initial_costmap()

        # timer for publishing costmap
        self.timer = self.create_timer(0.5, self.publish_costmap)
        self.get_logger().info('Costmap generator started')

    
    def load_sdf_world(self):
        # initialize resistance zones and obstacles
        self.resistance_zones = []
        self.obstacles = []

        # if not found
        if not self.sdf_path or not os.path.exists(self.sdf_path):
            self.get_logger().warn(f'SDF file not found at {self.sdf_path}, scanning world')
            return
        
        try:
            tree = ET.parse(self.sdf_path)
            root = tree.getroot()


            # iterate through all models in the world
            for model in root.findall(".//model"):
                model_name = model.get("name", '')

                # skip ground plane and boundary_wall
                if model_name in ['ground_plane', 'boundary_wall']:
                    continue

                pose_elem = model.find(".//pose")
                if pose_elem is None:
                    continue


                pose_values = pose_elem.text.split()

                # all models are generated with position noise (10% variation)
                x = float(pose_values[0])  * (1 + np.random.normal(0, self.noise_factor_position))
                y = float(pose_values[1])  * (1 + np.random.normal(0, self.noise_factor_position))

                ############ CHECK MODEL TYPE ############
                # In actual experimentation this needs to be adjusted # 
                ##########    BOX OR CYLINDER   ########## 
                cylinder_elem = model.find(".//visual/geometry/cylinder")
                box_elem = model.find(".//visual/geometry/box")

                # check if solid obstacle
                has_collision = model.find(".//collision") is not None

                # is cylinder
                if cylinder_elem is not None:
                    radius_elem = model.find(".//visual/geometry/cylinder/radius")
                    height_elem = model.find(".//visual/geometry/cylinder/length")

                    # default if radius or height not found
                    radius = 1.0
                    height = 0.5

                    if radius_elem is not None:
                        radius = float(radius_elem.text)
                    if height_elem is not None:
                        height = float(height_elem.text)

                    # if solid obstacle
                    if has_collision:
                        self.obstacles.append({
                            'x': x,
                            'y': y,
                            'radius': radius,
                            'height': height,
                            'type': 'cylinder',
                            # cost 95 +/- 5%
                            'cost': 95 * (1 + np.random.normal(0, self.noise_factor_obstacle))
                        })

                    # it's a resistance zone
                    else:
                        transparency_elem = model.find(".//visual/transparency")
                        # default
                        resistance_factor = 0.5

                        if transparency_elem is not None:
                            transparency = float(transparency_elem.text)
                            base_factor = 1.0 - transparency
                            resistance_factor = base_factor * (1 + np.random.normal(0, self.noise_factor_resistance))

                        self.resistance_zones.append({
                            'x': x,
                            'y': y,
                            'radius': radius,
                            'height': height,
                            # 'resistance_factor': resistance_factor,
                            'type': 'cylinder',
                            'cost': resistance_factor * 100
                        })

                # is box
                elif box_elem is not None:
                    size_elem = model.find(".//visual/geometry/box/size")

                    # default if size not found
                    size_x, size_y, size_z = 1.0, 1.0, 1.0

                    if size_elem is not None:
                        size_values = size_elem.text.split()
                        size_x = float(size_values[0]) * (1 + np.random.normal(0, self.noise_factor_obstacle))
                        size_y = float(size_values[1]) * (1 + np.random.normal(0, self.noise_factor_obstacle))
                        size_z = float(size_values[2]) * (1 + np.random.normal(0, self.noise_factor_obstacle))



                        self.obstacles.append({
                            'x': x,
                            'y': y,
                            'size_x': size_x,
                            'size_y': size_y,
                            'size_z': size_z,
                            'type': 'box',
                            'cost': 95 * (1 + np.random.normal(0, self.noise_factor_obstacle))
                        })

            self.get_logger().info(f'Loaded {len(self.resistance_zones)} resistance zones and {len(self.obstacles)} obstacles from SDF')

        except Exception as e:
            self.get_logger().error(f'Error loading SDF file: {str(e)}')
            return



    def generate_initial_costmap(self):
        """
        Generate initial costmap based on SDF
        
        1. Create a base costmap with random noise
        2. Add resistance zones with noise
        3. Add solid obstacles with noise
        """

        self.costmap = np.random.randint(0, int(100 * self.noise_factor_position),
                                         (self.costmap_height, self.costmap_width),
                                         dtype=np.int8)
        
        # add resistance zones
        for zone in self.resistance_zones:
            center_x = int((zone['x'] - self.map_origin_x) / self.map_resolution)
            center_y = int((zone['y'] - self.map_origin_y) / self.map_resolution)
            radius_pixels = int(zone['radius'] / self.map_resolution)

            cost = int(zone['cost'])


            ###### GPT ######
            # Draw circle in the costmap with noisy edges
            for y in range(max(0, center_y - radius_pixels - 3), min(self.costmap_height, center_y + radius_pixels + 4)):
                for x in range(max(0, center_x - radius_pixels - 3), min(self.costmap_width, center_x + radius_pixels + 4)):
                    # Calculate distance from center
                    dx = x - center_x
                    dy = y - center_y
                    distance = math.sqrt(dx*dx + dy*dy)
                    
                    # Core area
                    if distance <= radius_pixels:
                        # Apply resistance with per-cell noise (±20%)
                        noisy_cost = int(cost * (1 + np.random.normal(0, self.noise_factor_resistance)))
                        noisy_cost = max(10, min(99, noisy_cost))  # Clamp to valid range
                        self.costmap[y, x] = noisy_cost
                    # Transition area (fuzzy edge)
                    elif distance <= radius_pixels + 3:
                        # Calculate falloff (1.0 at radius, 0.0 at radius+3)
                        falloff = 1.0 - ((distance - radius_pixels) / 3.0)
                        # Apply reduced cost with high noise
                        edge_cost = int(cost * falloff * (1 + np.random.normal(0, self.noise_factor_resistance * 2)))
                        edge_cost = max(5, min(90, edge_cost))  # Clamp
                        
                        # Only overwrite if new cost is higher than existing
                        if edge_cost > self.costmap[y, x]:
                            self.costmap[y, x] = edge_cost

            ###### GPT ######

        # add obstacles
        for obstacle in self.obstacles:
            center_x = int((obstacle['x'] - self.map_origin_x) / self.map_resolution)
            center_y = int((obstacle['y'] - self.map_origin_y) / self.map_resolution)

            obstacle_cost = int(obstacle['cost'])

            # if cylinder
            if obstacle.get('type', '') == 'cylinder' or 'radius' in obstacle:
                radius_pixels = int(obstacle['radius'] / self.map_resolution)

                # Draw circle in the costmap with noisy edges
                
                # Draw circle in the costmap with noisy edges
                for y in range(max(0, center_y - radius_pixels - 2), min(self.costmap_height, center_y + radius_pixels + 3)):
                    for x in range(max(0, center_x - radius_pixels - 2), min(self.costmap_width, center_x + radius_pixels + 3)):
                        # Calculate distance from center
                        dx = x - center_x
                        dy = y - center_y
                        distance = math.sqrt(dx*dx + dy*dy)
                        
                        # Core area
                        if distance <= radius_pixels:
                            # Apply obstacle cost with small noise
                            noisy_cost = int(obstacle_cost * (1 + np.random.normal(0, self.noise_factor_obstacle)))
                            self.costmap[y, x] = min(100, max(90, noisy_cost))
                        # Transition area (fuzzy edge)
                        elif distance <= radius_pixels + 2:
                            # Calculate falloff
                            falloff = 1.0 - ((distance - radius_pixels) / 2.0)
                            # Apply reduced cost
                            edge_cost = int(obstacle_cost * falloff * (1 + np.random.normal(0, self.noise_factor_obstacle * 1.5)))
                            edge_cost = max(50, min(95, edge_cost))
                            
                            # Only overwrite if new cost is higher than existing
                            if edge_cost > self.costmap[y, x]:
                                self.costmap[y, x] = edge_cost
            else:
                # It's a box (rock)
                half_size_x = int(obstacle.get('size_x', 1.0) / (2 * self.map_resolution))
                half_size_y = int(obstacle.get('size_y', 1.0) / (2 * self.map_resolution))
                
                # Draw box in the costmap with noisy edges
                for y_offset in range(-half_size_y - 2, half_size_y + 3):
                    y = center_y + y_offset
                    if y < 0 or y >= self.costmap_height:
                        continue
                        
                    for x_offset in range(-half_size_x - 2, half_size_x + 3):
                        x = center_x + x_offset
                        if x < 0 or x >= self.costmap_width:
                            continue
                            
                        # Check if inside box core or transition zone
                        if abs(x_offset) <= half_size_x and abs(y_offset) <= half_size_y:
                            # Core box
                            noisy_cost = int(obstacle_cost * (1 + np.random.normal(0, self.noise_factor_obstacle)))
                            self.costmap[y, x] = min(100, max(90, noisy_cost))
                        else:
                            # Calculate distance to nearest edge
                            dx = max(0, abs(x_offset) - half_size_x)
                            dy = max(0, abs(y_offset) - half_size_y)
                            dist = math.sqrt(dx*dx + dy*dy)
                            
                            if dist <= 2:  # Transition zone
                                falloff = 1.0 - (dist / 2.0)
                                edge_cost = int(obstacle_cost * falloff * (1 + np.random.normal(0, self.noise_factor_obstacle * 1.5)))
                                edge_cost = max(50, min(95, edge_cost))
                                
                                # Only overwrite if new cost is higher
                                if edge_cost > self.costmap[y, x]:
                                    self.costmap[y, x] = edge_cost
        
        self.get_logger().info('Initial noisy costmap generated')   



    def world_to_costmap(self, world_x, world_y):
        """
        Convert world coordinates to costmap coordinates
        """
        costmap_x = int((world_x - self.map_origin_x) / self.map_resolution)
        costmap_y = int((world_y - self.map_origin_y) / self.map_resolution)

        return costmap_x, costmap_y
    
    def position_callback(self, msg):
        for transform in msg.transforms:
            # if (transform.header.frame_id == 'resistance_zones' and
                # transform.child_frame_id == 'a200_0000/robot'):
            
            # if transform.child_frame_id == 'base_link':
            if transform.header.frame_id == 'odom' and transform.child_frame_id == 'a200_0000/robot':

                
                # set robot position
                self.robot_x = transform.transform.translation.x
                self.robot_y = transform.transform.translation.y
                break

    def cost_callback(self, msg):
        """Process traversal cost from robot"""
        self.current_traversal_cost = msg.data * 100  # Convert to 0-100 scale
        
        # Update costmap at robot position with collision detection
        costmap_x, costmap_y = self.world_to_costmap(self.robot_x, self.robot_y)
        
        # Check if robot hit an obstacle (by detecting if commanded speed was > 0
        # but actual speed was near 0, implying a collision)
        is_collision = self.current_traversal_cost > 90  # Near-complete resistance
        
        if is_collision:
            # Robot likely hit an obstacle - mark this cell as obstacle (100)
            # and update surrounding cells more aggressively
            self.costmap[costmap_y, costmap_x] = 100
            self.get_logger().info(f'Collision detected at ({self.robot_x:.2f}, {self.robot_y:.2f}), updating map')
            
            # Update surrounding cells in larger kernel with high costs
            kernel_size = 4  # Larger kernel for obstacle detection
            for y in range(max(0, costmap_y - kernel_size), min(self.costmap_height, costmap_y + kernel_size + 1)):
                for x in range(max(0, costmap_x - kernel_size), min(self.costmap_width, costmap_x + kernel_size + 1)):
                    if (x == costmap_x and y == costmap_y):
                        continue  # Skip center point (already updated)
                    
                    # Calculate distance-based weight
                    dx, dy = x - costmap_x, y - costmap_y
                    distance = math.sqrt(dx*dx + dy*dy)
                    if distance <= kernel_size:
                        # Obstacles have sharper falloff than resistance zones
                        weight = (1.0 - (distance / kernel_size)) ** 2  # Quadratic falloff
                        
                        # Calculate cost based on distance from collision point
                        # Closer = higher cost
                        collision_cost = 100 - (50 * distance / kernel_size)
                        
                        # Higher learning rate for obstacle discovery
                        collision_learning_rate = min(0.8, self.learning_rate * 2.0)
                        effective_learning_rate = collision_learning_rate * weight
                        
                        # Update cost - more aggressively around obstacles
                        current_cell_cost = self.costmap[y, x]
                        updated_cell_cost = int((1 - effective_learning_rate) * current_cell_cost + 
                                               effective_learning_rate * collision_cost)
                        self.costmap[y, x] = min(99, updated_cell_cost)  # Cap at 99
        else:
            # Normal resistance update
            # Update with learning rate
            current_cost = self.costmap[costmap_y, costmap_x]
            updated_cost = int((1 - self.learning_rate) * current_cost + self.learning_rate * self.current_traversal_cost)
            
            # Only update if the new cost is significantly different
            cost_difference = abs(updated_cost - current_cost)
            if cost_difference > 5:  # Only update if difference is meaningful
                self.costmap[costmap_y, costmap_x] = min(updated_cost, 99)  # Cap at 99 (100 is for obstacles)
                
                # Also update nearby cells with decreasing effect (smaller kernel)
                kernel_size = 3
                for y in range(max(0, costmap_y - kernel_size), min(self.costmap_height, costmap_y + kernel_size + 1)):
                    for x in range(max(0, costmap_x - kernel_size), min(self.costmap_width, costmap_x + kernel_size + 1)):
                        if (x == costmap_x and y == costmap_y) or self.costmap[y, x] >= 99:
                            continue  # Skip center point (already updated) and obstacles
                        
                        # Calculate distance-based weight
                        dx, dy = x - costmap_x, y - costmap_y
                        distance = math.sqrt(dx*dx + dy*dy)
                        if distance <= kernel_size:
                            weight = (kernel_size - distance) / kernel_size
                            reduced_learning_rate = self.learning_rate * weight
                            
                            # Update cost
                            current_cell_cost = self.costmap[y, x]
                            updated_cell_cost = int((1 - reduced_learning_rate) * current_cell_cost + 
                                                   reduced_learning_rate * self.current_traversal_cost)
                            
                            # Apply update only if the cost would change significantly
                            if abs(updated_cell_cost - current_cell_cost) > 3:
                                self.costmap[y, x] = min(updated_cell_cost, 98)  # Cap just under obstacle threshold
    
    def publish_costmap(self):
        """Publish costmap as OccupancyGrid"""
        costmap_msg = OccupancyGrid()
        # costmap_msg.header.frame_id = 'resistance_zones'
        costmap_msg.header.frame_id = 'map'
        costmap_msg.header.stamp = self.get_clock().now().to_msg()
        
        costmap_msg.info.resolution = self.map_resolution
        costmap_msg.info.width = self.costmap_width
        costmap_msg.info.height = self.costmap_height
        costmap_msg.info.origin.position.x = self.map_origin_x
        costmap_msg.info.origin.position.y = self.map_origin_y
        
        # Convert numpy array to flat list
        # costmap_msg.data = self.costmap.flatten().tolist()
        costmap_msg.data = self.costmap.astype(np.int8).flatten().tolist()

        
        self.costmap_pub.publish(costmap_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CostMapGenerator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()