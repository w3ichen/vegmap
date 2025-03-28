#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Empty
import numpy as np
import math
from tf2_msgs.msg import TFMessage


# Names of desired vegetation
VEG_NAMES = ["tree", "bush"]


class VegCostmapUpdater(Node):
    def __init__(self):
        super().__init__("veg_costmap_updater")
        self.gradient_cost = False  # Use gradient cost for obstacles
        self.cost_radius_m = 0.5  # Radius for cost point (in meters)

        # Parameters
        self.declare_parameter("map_width", 1000)  # 1000x1000 map
        self.declare_parameter("map_height", 1000)
        self.declare_parameter("map_resolution", 0.05)  # 5cm per cell
        self.declare_parameter(
            "map_origin_x", -25.0
        )  # Center it by (1000*0.05)=50m, 50m/2
        self.declare_parameter("map_origin_y", -25.0)
        self.declare_parameter("update_frequency", 1.0)
        self.declare_parameter("world_tf_topic", "/outdoors_tf")

        self.map_width = (
            self.get_parameter("map_width").get_parameter_value().integer_value
        )
        self.map_height = (
            self.get_parameter("map_height").get_parameter_value().integer_value
        )
        self.map_resolution = (
            self.get_parameter("map_resolution").get_parameter_value().double_value
        )
        self.map_origin_x = (
            self.get_parameter("map_origin_x").get_parameter_value().double_value
        )
        self.map_origin_y = (
            self.get_parameter("map_origin_y").get_parameter_value().double_value
        )
        self.update_frequency = (
            self.get_parameter("update_frequency").get_parameter_value().double_value
        )
        self.world_tf_topic = (
            self.get_parameter("world_tf_topic").get_parameter_value().string_value
        )

        # Create publishers
        self.costmap_pub = self.create_publisher(OccupancyGrid, "/veg_costmap", 10)
        self.update_notifier_pub = self.create_publisher(
            Empty, "/veg_costmap_updated", 10
        )

        # Subscribe to TF messages to get obstacle positions
        self.world_tf_sub = self.create_subscription(
            TFMessage, self.world_tf_topic, self.world_tf_callback, 10
        )

        # Create timer for periodic costmap updates
        self.timer = self.create_timer(
            1.0 / self.update_frequency, self.updater_callback
        )

        # Initialize costmap data
        self.costmap_data = np.zeros((self.map_height, self.map_width), dtype=np.int8)
        self.obstacle_positions = []

        self.get_logger().info("Vegetation Costmap Updater initialized")

    def world_tf_callback(self, msg):
        """Process TF messages to extract obstacle positions from the world"""
        obstacle_positions = []

        for transform in msg.transforms:
            child_frame = transform.child_frame_id

            # Skip if not vegetation
            if not any(substr in child_frame for substr in VEG_NAMES):
                continue

            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z  # Can be used for height-based cost

            # Store position and height for cost calculation
            obstacle_positions.append((x, y, z, child_frame))

        self.obstacle_positions = obstacle_positions

        self.get_logger().debug(
            f"Updated obstacle positions: {len(obstacle_positions)} obstacles"
        )

    def world_to_map(self, wx, wy):
        """Convert world coordinates to map cell coordinates"""
        mx = int((wx - self.map_origin_x) / self.map_resolution)
        my = int((wy - self.map_origin_y) / self.map_resolution)

        # Check if within map bounds
        if 0 <= mx < self.map_width and 0 <= my < self.map_height:
            return mx, my
        # Error: out of bounds
        self.get_logger().warn(
            f"World coordinates ({wx}, {wy}) out of map bounds: ({mx}, {my})"
        )
        return None, None

    def updater_callback(self):
        """Periodically update the costmap"""
        # Reset costmap
        self.costmap_data.fill(0)

        # Update costmap based on obstacle positions
        for x, y, z, frame_id in self.obstacle_positions:
            mx, my = self.world_to_map(x, y)
            if mx is None or my is None:
                continue

            # Cost
            # -1 (represented as 255 in the underlying int8 array): Unknown space
            # 0: Completely free space (definitely traversable)
            # 1 to 99: Partially occupied space (with increasing cost)
            # 100: Completely occupied space (definitely not traversable)
            # Random value between 1 and 100
            cost = 50

            # # Apply cost with a radius - creates a gradient
            radius_cells = max(1, int(self.cost_radius_m / self.map_resolution))

            for i in range(-radius_cells, radius_cells + 1):
                for j in range(-radius_cells, radius_cells + 1):
                    # Skip if outside circle
                    if i * i + j * j > radius_cells * radius_cells:
                        continue

                    # Calculate cell coordinates
                    cell_x = mx + i
                    cell_y = my + j

                    # Check if within map bounds
                    if 0 <= cell_x < self.map_width and 0 <= cell_y < self.map_height:
                        if self.gradient_cost:
                            # Calculate distance-based cost (higher at center, lower at edges)
                            distance = math.sqrt(i * i + j * j)
                            distance_factor = 1.0 - (distance / radius_cells)
                            cell_cost = int(cost * distance_factor)
                        else:
                            # Use constant cost for the radius
                            cell_cost = cost

                        # Apply maximum cost
                        self.costmap_data[cell_y, cell_x] = max(
                            self.costmap_data[cell_y, cell_x], cell_cost
                        )

        # Publish the updated costmap
        self.publish_costmap()

    def publish_costmap(self):
        """Publish the costmap as an OccupancyGrid message"""
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        msg.info.resolution = self.map_resolution
        msg.info.width = self.map_width
        msg.info.height = self.map_height

        msg.info.origin.position.x = self.map_origin_x
        msg.info.origin.position.y = self.map_origin_y
        msg.info.origin.position.z = 0.0

        # Identity quaternion (no rotation)
        msg.info.origin.orientation.w = 1.0

        # Flatten the numpy array to a 1D list
        flat_data = self.costmap_data.flatten().tolist()
        msg.data = flat_data

        # Publish the costmap
        self.costmap_pub.publish(msg)

        # Notify that costmap was updated
        self.update_notifier_pub.publish(Empty())


def main():
    rclpy.init()
    node = VegCostmapUpdater()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
