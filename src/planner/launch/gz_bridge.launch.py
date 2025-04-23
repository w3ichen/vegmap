from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    namespace = LaunchConfiguration('namespace', default='a200_0000')
    return LaunchDescription(
        [
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="model_pose_bridge",
                namespace=namespace,  
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
                namespace=namespace,  
            ),
            # Relay /a200_0000/tf -> /tf
            Node(
                package='topic_tools',
                executable='relay',
                name='tf_relay',
                namespace=namespace,  
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
                namespace=namespace,  
                output='log',
                arguments=['/a200_0000/tf_static', '/tf_static'],
                parameters=[
                    {'qos_reliability': 'reliable'},
                    {'qos_durability': 'transient_local'},
                    {'logger_level': 'error'},
                ]
            )





          
            

        ]
    )