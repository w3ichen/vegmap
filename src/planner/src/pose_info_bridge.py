#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
import subprocess
from subprocess import PIPE
import re
import os
from planner_msgs.srv import GetTransforms
from tf2_ros import TransformBroadcaster


class PoseInfoService(Node):
    def __init__(self):
        super().__init__("pose_info_service")

        # Create the service
        self.srv = self.create_service(
            GetTransforms, "/world/get_tf", self.get_tf_callback
        )
        
        self.get_logger().info("Pose info service initialized")
    

    def get_tf_callback(self, request, response):
        world = request.world

        self.get_logger().info(f"Getting transforms for world: {world}")

        # Get pose info using subprocess
        if os.environ.get("GZ_VERSION") == "garden":
            gz_exec = "gz"
        else:
            gz_exec = "ign"

        cmd = f"{gz_exec} topic -e -t /world/{world}/pose/info -n1"
        process = subprocess.Popen([cmd], shell=True, stdout=PIPE)
        output = process.communicate()[0].decode("utf-8")

        # Create TF message
        response.transforms = TFMessage()

        # Parse output and extract poses
        poses = re.findall(
            r'pose\s*{[^}]*name:\s*"([^"]*)"[^}]*position\s*{([^}]*)}\s*orientation\s*{([^}]*)}',
            output,
            re.DOTALL,
        )

        for pose_data in poses:
            name, position, orientation = pose_data

            # Parse position
            pos_x = (
                float(re.search(r"x:\s*([\d.-]+)", position).group(1))
                if re.search(r"x:\s*([\d.-]+)", position)
                else 0.0
            )
            pos_y = (
                float(re.search(r"y:\s*([\d.-]+)", position).group(1))
                if re.search(r"y:\s*([\d.-]+)", position)
                else 0.0
            )
            pos_z = (
                float(re.search(r"z:\s*([\d.-]+)", position).group(1))
                if re.search(r"z:\s*([\d.-]+)", position)
                else 0.0
            )

            # Parse orientation
            quat_x = (
                float(re.search(r"x:\s*([\d.-]+)", orientation).group(1))
                if re.search(r"x:\s*([\d.-]+)", orientation)
                else 0.0
            )
            quat_y = (
                float(re.search(r"y:\s*([\d.-]+)", orientation).group(1))
                if re.search(r"y:\s*([\d.-]+)", orientation)
                else 0.0
            )
            quat_z = (
                float(re.search(r"z:\s*([\d.-]+)", orientation).group(1))
                if re.search(r"z:\s*([\d.-]+)", orientation)
                else 0.0
            )
            quat_w = (
                float(re.search(r"w:\s*([\d.-]+)", orientation).group(1))
                if re.search(r"w:\s*([\d.-]+)", orientation)
                else 1.0
            )

            # Create transform
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = "resistance_zone"
            transform.child_frame_id = name

            transform.transform.translation.x = pos_x
            transform.transform.translation.y = pos_y
            transform.transform.translation.z = pos_z

            transform.transform.rotation.x = quat_x
            transform.transform.rotation.y = quat_y
            transform.transform.rotation.z = quat_z
            transform.transform.rotation.w = quat_w

            response.transforms.transforms.append(transform)

        transform_count = len(response.transforms.transforms)

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
    node = PoseInfoService()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()