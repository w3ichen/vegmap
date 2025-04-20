#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
import subprocess
from subprocess import PIPE
import re
import os
import json
from planner_msgs.srv import GetTransforms


class SceneInfoService(Node):
    def __init__(self):
        super().__init__("scene_info_service")

        # Create the service
        self.srv = self.create_service(
            GetTransforms, "/world/get_tf", self.get_tf_callback
        )
        
        self.get_logger().info("Scene info service initialized")
    
    def get_tf_callback(self, request, response):
        world = request.world
        self.get_logger().info(f"Getting scene info for world: {world}")

        # Get scene info using subprocess
        if os.environ.get("GZ_VERSION") == "garden":
            gz_exec = "gz"
        else:
            gz_exec = "ign"

        cmd = f"{gz_exec} service -s /world/{world}/scene/info --reqtype ignition.msgs.Empty --reptype ignition.msgs.Scene --timeout 2000 --req '{{}}'"
        process = subprocess.Popen([cmd], shell=True, stdout=PIPE)
        output = process.communicate()[0].decode("utf-8")

        # Create TF message
        response.transforms = TFMessage()

        # Parse the output - we'll use regex since it's not proper JSON
        # Extract link information
        links = re.finditer(r'link\s*{([^{]*({[^}]*})*[^}]*)}', output, re.DOTALL)
        
        transform_count = 0

        for link_match in links:
            link_data = link_match.group(1)
            
            # Extract link name
            name_match = re.search(r'name:\s*"([^"]*)"', link_data)
            if not name_match:
                continue
                
            link_name = name_match.group(1)
            
            # Extract link pose
            pose_match = re.search(r'pose\s*{([^}]*)}', link_data)
            if not pose_match:
                continue
                
            pose_data = pose_match.group(1)
            
            # Parse position
            position_match = re.search(r'position\s*{([^}]*)}', pose_data)
            position = position_match.group(1) if position_match else ""
            
            pos_x = float(re.search(r'x:\s*([\d.-]+)', position).group(1)) if re.search(r'x:\s*([\d.-]+)', position) else 0.0
            pos_y = float(re.search(r'y:\s*([\d.-]+)', position).group(1)) if re.search(r'y:\s*([\d.-]+)', position) else 0.0
            pos_z = float(re.search(r'z:\s*([\d.-]+)', position).group(1)) if re.search(r'z:\s*([\d.-]+)', position) else 0.0
            
            # Parse orientation
            orientation_match = re.search(r'orientation\s*{([^}]*)}', pose_data)
            orientation = orientation_match.group(1) if orientation_match else ""
            
            quat_x = float(re.search(r'x:\s*([\d.-]+)', orientation).group(1)) if re.search(r'x:\s*([\d.-]+)', orientation) else 0.0
            quat_y = float(re.search(r'y:\s*([\d.-]+)', orientation).group(1)) if re.search(r'y:\s*([\d.-]+)', orientation) else 0.0
            quat_z = float(re.search(r'z:\s*([\d.-]+)', orientation).group(1)) if re.search(r'z:\s*([\d.-]+)', orientation) else 0.0
            quat_w = float(re.search(r'w:\s*([\d.-]+)', orientation).group(1)) if re.search(r'w:\s*([\d.-]+)', orientation) else 1.0
            
            # Create base transform for link
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = "world"
            transform.child_frame_id = link_name
            
            transform.transform.translation.x = pos_x
            transform.transform.translation.y = pos_y
            transform.transform.translation.z = pos_z
            
            transform.transform.rotation.x = quat_x
            transform.transform.rotation.y = quat_y
            transform.transform.rotation.z = quat_z
            transform.transform.rotation.w = quat_w
            
            # Add geometry information to frame_id
            # Extract visual information for this link
            visuals = re.finditer(r'visual\s*{([^{]*({[^}]*})*[^}]*)}', link_data, re.DOTALL)
            
            for visual_match in visuals:
                visual_data = visual_match.group(1)
                
                # Extract visual name
                visual_name_match = re.search(r'name:\s*"([^"]*)"', visual_data)
                if not visual_name_match:
                    continue
                    
                visual_name = visual_name_match.group(1)
                
                # Extract geometry
                geometry_match = re.search(r'geometry\s*{([^{]*({[^}]*})*[^}]*)}', visual_data, re.DOTALL)
                if not geometry_match:
                    continue
                    
                geometry_data = geometry_match.group(1)
                
                # Look for specific geometry types
                cylinder_match = re.search(r'cylinder\s*{([^}]*)}', geometry_data)
                box_match = re.search(r'box\s*{([^}]*)}', geometry_data)
                mesh_match = re.search(r'mesh\s*{([^}]*)}', geometry_data)
                
                # Create a copy of the transform with visual info
                visual_transform = TransformStamped()
                visual_transform.header.stamp = transform.header.stamp
                visual_transform.header.frame_id = transform.header.frame_id
                visual_transform.child_frame_id = f"{link_name}__{visual_name}"
                visual_transform.transform = transform.transform
                
                # Add geometry info to child_frame_id
                if cylinder_match:
                    cylinder_data = cylinder_match.group(1)
                    radius = float(re.search(r'radius:\s*([\d.-]+)', cylinder_data).group(1)) if re.search(r'radius:\s*([\d.-]+)', cylinder_data) else 0.0
                    length = float(re.search(r'length:\s*([\d.-]+)', cylinder_data).group(1)) if re.search(r'length:\s*([\d.-]+)', cylinder_data) else 0.0
                    visual_transform.child_frame_id = f"{link_name}__{visual_name}__cylinder_r{radius}_l{length}"
                
                elif box_match:
                    box_data = box_match.group(1)
                    size_match = re.search(r'size\s*{([^}]*)}', box_data)
                    if size_match:
                        size_data = size_match.group(1)
                        size_x = float(re.search(r'x:\s*([\d.-]+)', size_data).group(1)) if re.search(r'x:\s*([\d.-]+)', size_data) else 0.0
                        size_y = float(re.search(r'y:\s*([\d.-]+)', size_data).group(1)) if re.search(r'y:\s*([\d.-]+)', size_data) else 0.0
                        size_z = float(re.search(r'z:\s*([\d.-]+)', size_data).group(1)) if re.search(r'z:\s*([\d.-]+)', size_data) else 0.0
                        visual_transform.child_frame_id = f"{link_name}__{visual_name}__box_x{size_x}_y{size_y}_z{size_z}"
                
                elif mesh_match:
                    mesh_data = mesh_match.group(1)
                    filename = re.search(r'filename:\s*"([^"]*)"', mesh_data).group(1) if re.search(r'filename:\s*"([^"]*)"', mesh_data) else ""
                    visual_transform.child_frame_id = f"{link_name}__{visual_name}__mesh_{os.path.basename(filename)}"
                
                response.transforms.transforms.append(visual_transform)
                transform_count += 1
            
            # Also add the base link transform
            response.transforms.transforms.append(transform)
            transform_count += 1

        response.success = transform_count > 0
        response.message = (
            f"Found {transform_count} transforms for world '{world}'"
            if transform_count > 0
            else f"No transforms found for world '{world}'"
        )

        self.get_logger().info(
            f"Returning {transform_count} transforms for world '{world}'"
        )
        return response


def main():
    rclpy.init()
    node = SceneInfoService()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()