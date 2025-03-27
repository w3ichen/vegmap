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
                executable="pose_info_bridge.py",
                name="pose_info_bridge",
            ),
            # Relay /a200_0000/tf -> /tf
            Node(
                package='topic_tools',
                executable='relay',
                name='tf_relay',
                output={
                    'stdout': 'log',
                    'stderr': 'log',
                },
                arguments=['/a200_0000/tf', '/tf'],
                parameters=[
                    {'qos_reliability': 'reliable'},
                    {'qos_durability': 'volatile'},
                    {'logger_level': 'error'}
                ]
            ),
            # Relay /a200_0000/tf_static -> /tf_static
            Node(
                package='topic_tools',
                executable='relay',
                name='tf_static_relay',
                output='log',
                arguments=['/a200_0000/tf_static', '/tf_static'],
                parameters=[
                    {'qos_reliability': 'reliable'},
                    {'qos_durability': 'transient_local'},
                    {'logger_level': 'error'},
                ]
            ),
            # TF Transforms
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='map_to_odom',
                arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
                output='screen'
            ),
            # tf transformation odom to base link
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='odom_to_base_link',
                arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
                output='screen'
            )
        ]
    )