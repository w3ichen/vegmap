from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="model_pose_bridge",
                output="screen",
                parameters=[
                    {
                        "qos_overrides./model/a200_0000/robot/pose.publisher.reliability": "reliable",
                        "qos_overrides./model/a200_0000/robot/pose.publisher.durability": "transient_local",
                        "qos_overrides./world/outdoors/pose/info.publisher.reliability": "reliable",
                        "qos_overrides./world/outdoors/pose/info.publisher.durability": "transient_local",
                    }
                ],
                arguments=[
                    # Robot pose bridge
                    "/model/a200_0000/robot/pose@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V",
                    # outdoors pose info bridge
                    # "/world/outdoors/pose/info@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V",
                ],
            ),
            Node(
                package="planner",
                executable="pose_info_bridge.py",  # Note the .py extension when installed with install(PROGRAMS...)
                name="pose_info_bridge",
            ),
        ]
    )
