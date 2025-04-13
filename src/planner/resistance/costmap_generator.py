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
        self.declare_parameter('noise_factor', 0.05,
                              ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE,
                                                 description='Noise factor for the initial costmap'))
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
        self.noise_factor = self.get_parameter('noise_factor').value
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
            self.pose_callback,
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
                x = float(pose_values[0])  * (1 + np.random.normal(0, self.noise_factor))
                y = float(pose_values[1])  * (1 + np.random.normal(0, self.noise_factor))

                ############ CHECK MODEL TYPE ############
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
                            'type': 'cylinder'
                        })

                    # it's a resistance zone
                    else:
                        transparency_elem = model.find(".//visual/transparency")
                        # default
                        resistance_factor = 0.5

                        if transparency_elem is not None:
                            transparency = float(transparency_elem.text)
                            resistance_Factor = 1.0 - transparency

                        self.resistance_zones.append({
                            'x': x,
                            'y': y,
                            'radius': radius,
                            'resistance_factor': resistance_factor,
                            'type': 'cylinder'
                        })

                # is box
                elif box_elem is not None:
                    size_elem = model.find(".//visual/geometry/box/size")

                    # default if size not found
                    size_x, size_y, size_z = 1.0, 1.0, 1.0

                    if size_elem is not None:
                        size_values = size_elem.text.split()
                        size_x, size_y, size_z = float(size_values[0]), float(size_values[1]), float(size_values[2])

                        self.obstacles.append({
                            'x': x,
                            'y': y,
                            'size_x': size_x,
                            'size_y': size_y,
                            'size_z': size_z,
                            'type': 'box'
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

        self.costmap = np.random.randint(0, int(100 * self.noise_factor),
                                         (self.costmap_height, self.costmap_width),
                                         dtype=np.int8)
        

        for zone in self.resistance_zones:

            center_x = int((zone['x'] - self.map_origin_x) / self.map_resolution)
            center_y = int((zone['y'] - self.map_origin_y) / self.map_resolution)
            radius = int(zone['radius'] / self.map_resolution)

            std_zone = 


    """
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
    Generates and updates a costmap based on world geometry and robot traversal data
    """
    def __init__(self):
        super().__init__('costmap_generator')
        
        # Parameters - declaring with descriptors for better documentation
        # ROS2 parameter system requires parameters to be declared before use
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
        self.declare_parameter('noise_factor', 0.05,
                              ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE,
                                                 description='Noise factor for the initial costmap'))
        self.declare_parameter('learning_rate', 0.3,
                              ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE,
                                                 description='Learning rate for updating the costmap'))
        self.declare_parameter('sdf_path', '',
                              ParameterDescriptor(type=ParameterType.PARAMETER_STRING,
                                                 description='Path to the SDF file'))
        
        # Get parameters - retrieving actual values after declaration
        self.map_resolution = self.get_parameter('map_resolution').value
        self.map_width = self.get_parameter('map_width').value
        self.map_height = self.get_parameter('map_height').value
        self.map_origin_x = self.get_parameter('map_origin_x').value
        self.map_origin_y = self.get_parameter('map_origin_y').value
        self.noise_factor = self.get_parameter('noise_factor').value
        self.learning_rate = self.get_parameter('learning_rate').value
        self.sdf_path = self.get_parameter('sdf_path').value
        
        # Initialize costmap
        self.costmap_width = int(self.map_width / self.map_resolution)
        self.costmap_height = int(self.map_height / self.map_resolution)
        self.costmap = np.zeros((self.costmap_height, self.costmap_width), dtype=np.int8)
        
        # Track robot position
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.current_traversal_cost = 0.0
        
        # Subscribers
        self.position_sub = self.create_subscription(
            TFMessage,
            '/model/a200_0000/robot/pose',
            self.position_callback,
            10
        )
        
        self.cost_sub = self.create_subscription(
            Float32,
            '/cost_traverse',
            self.cost_callback,
            10
        )
        
        # Publishers
        self.costmap_pub = self.create_publisher(
            OccupancyGrid,
            '/traversability_costmap',
            10
        )
        
        # Load world data from SDF
        self.resistance_zones = []
        self.obstacles = []
        self.load_sdf_world()
        
        # Generate initial costmap
        self.generate_initial_costmap()
        
        # Timer for publishing costmap
        self.timer = self.create_timer(0.5, self.publish_costmap)
        
        self.get_logger().info('CostMap Generator started')
        
    def load_sdf_world(self):
        """Load world data from SDF file"""
        # Initialize empty collections for zones and obstacles
        self.resistance_zones = []
        self.obstacles = []
        
        if not self.sdf_path or not os.path.exists(self.sdf_path):
            self.get_logger().warn(f'SDF file not found at {self.sdf_path}, scanning world will be empty initially')
            return
            
        try:
            tree = ET.parse(self.sdf_path)
            root = tree.getroot()
            
            # Parse all models in the world
            for model in root.findall(".//model"):
                model_name = model.get('name', '')
                
                # Skip ground_plane and boundary_wall
                if model_name in ['ground_plane', 'boundary_wall']:
                    continue
                
                # Get pose (all models have this)
                pose_elem = model.find(".//pose")
                if pose_elem is None:
                    continue
                    
                pose_values = pose_elem.text.split()
                # Add noise to positions (±10% variation)
                x = float(pose_values[0]) * (1 + np.random.normal(0, 0.05))
                y = float(pose_values[1]) * (1 + np.random.normal(0, 0.05))
                
                # Check model geometry type (cylinder or box)
                cylinder_elem = model.find(".//visual/geometry/cylinder")
                box_elem = model.find(".//visual/geometry/box")
                
                # Check if model has collision - implies it's a solid obstacle
                has_collision = model.find(".//collision") is not None
                
                if cylinder_elem is not None:
                    # Get cylinder radius and height
                    radius_elem = model.find(".//visual/geometry/cylinder/radius")
                    height_elem = model.find(".//visual/geometry/cylinder/length")
                    
                    radius = 1.0  # Default
                    height = 0.5  # Default
                    
                    if radius_elem is not None:
                        # Add noise to radius (±15% variation)
                        radius_base = float(radius_elem.text)
                        radius = radius_base * (1 + np.random.normal(0, 0.10))
                        
                    if height_elem is not None:
                        height = float(height_elem.text)
                    
                    if has_collision:
                        # It's a solid obstacle (like a tree)
                        # Add small noise to perceived size (±5% variation)
                        noisy_radius = radius * (1 + np.random.normal(0, 0.05))
                        self.obstacles.append({
                            'x': x,
                            'y': y,
                            'radius': noisy_radius,
                            'height': height,
                            'type': 'cylinder',
                            # Add noise to cost (95-100 for obstacles)
                            'cost': min(100, 95 + np.random.randint(0, 6))
                        })
                    else:
                        # It's a resistance zone (non-solid)
                        # Calculate resistance factor from transparency
                        transparency_elem = model.find(".//visual/transparency")
                        resistance_factor = 0.5  # Default
                        
                        if transparency_elem is not None:
                            transparency = float(transparency_elem.text)
                            # Base factor from transparency
                            base_factor = 1.0 - transparency
                            # Add significant noise (±30%)
                            resistance_factor = max(0.1, min(0.9, base_factor * (1 + np.random.normal(0, 0.30))))
                        
                        # Add significant noise to resistance zone radius (±20%)
                        noisy_radius = radius * (1 + np.random.normal(0, 0.20))
                        self.resistance_zones.append({
                            'x': x,
                            'y': y,
                            'radius': noisy_radius,
                            'resistance_factor': resistance_factor
                        })
                
                elif box_elem is not None:
                    # Get box size
                    size_elem = model.find(".//visual/geometry/box/size")
                    
                    size_x, size_y, size_z = 1.0, 1.0, 1.0  # Default
                    
                    if size_elem is not None:
                        size_values = size_elem.text.split()
                        # Add noise to sizes (±10% variation)
                        size_x = float(size_values[0]) * (1 + np.random.normal(0, 0.10))
                        size_y = float(size_values[1]) * (1 + np.random.normal(0, 0.10))
                        size_z = float(size_values[2])
                    
                    # Boxes are typically solid obstacles
                    self.obstacles.append({
                        'x': x,
                        'y': y,
                        'size_x': size_x,
                        'size_y': size_y,
                        'size_z': size_z,
                        'type': 'box',
                        # Add noise to cost (95-100 for obstacles)
                        'cost': min(100, 95 + np.random.randint(0, 6))
                    })
            
            self.get_logger().info(f'Loaded {len(self.resistance_zones)} resistance zones and {len(self.obstacles)} obstacles from SDF')
            
        except Exception as e:
            self.get_logger().error(f'Error parsing SDF file: {e}')
            self.get_logger().error(f'Traceback: {str(e)}')
            # No defaults - map will be empty except for noise
    
    def generate_initial_costmap(self):
        """
        Generate initial costmap based on SDF world data
        
        This function:
        1. Creates a base cost map with light random noise
        2. Adds resistance zones with their specific resistance values
        3. Adds solid obstacles as impassable areas
        
        The function is called during initialization to create the starting
        cost map that will be refined as the robot explores the world
        """
        # Initialize with moderate random noise (creates terrain variability)
        self.costmap = np.random.randint(0, int(100 * self.noise_factor * 2), 
                                        (self.costmap_height, self.costmap_width), 
                                        dtype=np.int8)
        
        # Add resistance zones to the costmap
        for zone in self.resistance_zones:
            # Convert world coordinates to costmap indices
            center_x = int((zone['x'] - self.map_origin_x) / self.map_resolution)
            center_y = int((zone['y'] - self.map_origin_y) / self.map_resolution)
            radius_pixels = int(zone['radius'] / self.map_resolution)
            
            # Calculate resistance cost (0-100)
            cost_value = min(int(zone['resistance_factor'] * 100), 99)
            
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
                        noisy_cost = int(cost_value * (1 + np.random.normal(0, 0.2)))
                        noisy_cost = max(10, min(99, noisy_cost))  # Clamp to valid range
                        self.costmap[y, x] = noisy_cost
                    # Transition area (fuzzy edge)
                    elif distance <= radius_pixels + 3:
                        # Calculate falloff (1.0 at radius, 0.0 at radius+3)
                        falloff = 1.0 - ((distance - radius_pixels) / 3.0)
                        # Apply reduced cost with high noise
                        edge_cost = int(cost_value * falloff * (1 + np.random.normal(0, 0.3)))
                        edge_cost = max(5, min(90, edge_cost))  # Clamp
                        
                        # Only overwrite if new cost is higher than existing
                        if edge_cost > self.costmap[y, x]:
                            self.costmap[y, x] = edge_cost
        
        # Add obstacles to the costmap
        for obstacle in self.obstacles:
            # Convert world coordinates to costmap indices
            center_x = int((obstacle['x'] - self.map_origin_x) / self.map_resolution)
            center_y = int((obstacle['y'] - self.map_origin_y) / self.map_resolution)
            
            # Get the obstacle cost (default to 100 if not specified)
            obstacle_cost = obstacle.get('cost', 100)
            
            if obstacle.get('type', '') == 'cylinder' or 'radius' in obstacle:
                # It's a cylinder (tree)
                radius_pixels = int(obstacle['radius'] / self.map_resolution)
                
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
                            noisy_cost = int(obstacle_cost * (1 + np.random.normal(0, 0.05)))
                            self.costmap[y, x] = min(100, max(90, noisy_cost))
                        # Transition area (fuzzy edge)
                        elif distance <= radius_pixels + 2:
                            # Calculate falloff
                            falloff = 1.0 - ((distance - radius_pixels) / 2.0)
                            # Apply reduced cost
                            edge_cost = int(obstacle_cost * falloff * (1 + np.random.normal(0, 0.1)))
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
                            noisy_cost = int(obstacle_cost * (1 + np.random.normal(0, 0.05)))
                            self.costmap[y, x] = min(100, max(90, noisy_cost))
                        else:
                            # Calculate distance to nearest edge
                            dx = max(0, abs(x_offset) - half_size_x)
                            dy = max(0, abs(y_offset) - half_size_y)
                            dist = math.sqrt(dx*dx + dy*dy)
                            
                            if dist <= 2:  # Transition zone
                                falloff = 1.0 - (dist / 2.0)
                                edge_cost = int(obstacle_cost * falloff * (1 + np.random.normal(0, 0.1)))
                                edge_cost = max(50, min(95, edge_cost))
                                
                                # Only overwrite if new cost is higher
                                if edge_cost > self.costmap[y, x]:
                                    self.costmap[y, x] = edge_cost
        
        self.get_logger().info('Initial noisy costmap generated')
    
    def world_to_costmap(self, world_x, world_y):
        """Convert world coordinates to costmap indices"""
        costmap_x = int((world_x - self.map_origin_x) / self.map_resolution)
        costmap_y = int((world_y - self.map_origin_y) / self.map_resolution)
        
        # Ensure within bounds
        costmap_x = max(0, min(costmap_x, self.costmap_width - 1))
        costmap_y = max(0, min(costmap_y, self.costmap_height - 1))
        
        return costmap_x, costmap_y
    
    def position_callback(self, msg):
        """Process robot position"""
        for transform in msg.transforms:
            if (transform.header.frame_id == 'resistance_zones' and
                transform.child_frame_id == 'a200_0000/robot'):
                
                # Update robot position
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
        costmap_msg.header.frame_id = 'resistance_zones'
        costmap_msg.header.stamp = self.get_clock().now().to_msg()
        
        costmap_msg.info.resolution = self.map_resolution
        costmap_msg.info.width = self.costmap_width
        costmap_msg.info.height = self.costmap_height
        costmap_msg.info.origin.position.x = self.map_origin_x
        costmap_msg.info.origin.position.y = self.map_origin_y
        
        # Convert numpy array to flat list
        costmap_msg.data = self.costmap.flatten().tolist()
        
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
    """