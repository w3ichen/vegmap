#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
import subprocess
from subprocess import PIPE
import re
import shutil


class PoseInfoBridge(Node):
    def __init__(self):
        super().__init__("pose_info_bridge")

        # Create publisher
        self.pub = self.create_publisher(TFMessage, "/outdoors_tf", 10)

        # Create timer for polling
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info("Pose info bridge initialized")

    def timer_callback(self):
        world = "outdoors"
        # Get pose info using subprocess
        if shutil.which("gz") is not None:
            cmd = f"gz topic -e -t /world/{world}/pose/info -n1"
        else:
            cmd = f"ign topic -e -t /world/{world}/pose/info -n1"
        process = subprocess.Popen([cmd], shell=True, stdout=PIPE)
        output = process.communicate()[0].decode("utf-8")

        # Create TF message
        tf_msg = TFMessage()

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
            transform.header.frame_id = "world"
            transform.child_frame_id = name

            transform.transform.translation.x = pos_x
            transform.transform.translation.y = pos_y
            transform.transform.translation.z = pos_z

            transform.transform.rotation.x = quat_x
            transform.transform.rotation.y = quat_y
            transform.transform.rotation.z = quat_z
            transform.transform.rotation.w = quat_w

            tf_msg.transforms.append(transform)

        # Publish if we have transforms
        if tf_msg.transforms:
            self.pub.publish(tf_msg)
            self.get_logger().debug(f"Published {len(tf_msg.transforms)} transforms")


def main():
    rclpy.init()
    node = PoseInfoBridge()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
