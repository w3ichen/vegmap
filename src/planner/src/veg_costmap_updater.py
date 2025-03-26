#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Empty
import numpy as np
import math
from tf2_msgs.msg import TFMessage


class VegCostmapUpdater(Node):
    def __init__(self):
        super().__init__("veg_costmap_updater")

        # Parameters
        self.declare_parameter("map_width", 100)
        self.declare_parameter("map_height", 100)
        self.declare_parameter("map_resolution", 0.1)
        self.declare_parameter("map_origin_x", -5.0)
        self.declare_parameter("map_origin_y", -5.0)
        self.declare_parameter("update_frequency", 1.0)
        self.declare_parameter("tf_topic", "/outdoors_tf")

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
        self.tf_topic = (
            self.get_parameter("tf_topic").get_parameter_value().string_value
        )

        # Create publishers
        self.costmap_pub = self.create_publisher(OccupancyGrid, "/veg_updates", 10)
        self.update_notifier_pub = self.create_publisher(
            Empty, "/veg_costmap_updated", 10
        )

        # Subscribe to TF messages to get obstacle positions
        self.tf_sub = self.create_subscription(
            TFMessage, self.tf_topic, self.tf_callback, 10
        )

        # Create timer for periodic costmap updates
        self.timer = self.create_timer(1.0 / self.update_frequency, self.timer_callback)

        # Initialize costmap data
        self.costmap_data = np.zeros((self.map_height, self.map_width), dtype=np.int8)
        self.obstacle_positions = []

        self.get_logger().info("Vegetation Costmap Updater initialized")

    def tf_callback(self, msg):
        """Process TF messages to extract obstacle positions"""
        obstacle_positions = []

        for transform in msg.transforms:
            child_frame = transform.child_frame_id

            # You can filter based on frame ID if needed
            # if not child_frame.startswith('your_prefix'):
            #     continue

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
        return None, None

    def timer_callback(self):
        """Periodically update the costmap based on obstacle positions"""
        # Reset costmap
        self.costmap_data.fill(0)

        # Update costmap based on obstacle positions
        for x, y, z, frame_id in self.obstacle_positions:
            mx, my = self.world_to_map(x, y)
            if mx is None or my is None:
                continue

            # Calculate cost based on height (z value)
            # Taller objects (higher z) have higher cost
            if z < 0.3:
                cost = 50  # Low vegetation
            elif z < 1.0:
                cost = 75  # Medium vegetation
            else:
                cost = 99  # High vegetation/obstacles

            # Apply cost with a radius - creates a gradient
            radius_cells = max(1, int(0.5 / self.map_resolution))  # 0.5m radius

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
                        # Calculate distance-based cost (higher at center, lower at edges)
                        distance = math.sqrt(i * i + j * j)
                        distance_factor = 1.0 - (distance / radius_cells)
                        cell_cost = int(cost * distance_factor)

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
