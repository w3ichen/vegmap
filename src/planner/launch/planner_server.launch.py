from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="planner",
                executable="planner_server",
                output="screen",
            ),
            Node(
                package="ros_ign_bridge",
                executable="parameter_bridge",
                arguments=["/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist"],
            ),
        ]
    )
