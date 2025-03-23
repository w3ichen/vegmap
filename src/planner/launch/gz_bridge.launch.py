from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='model_pose_bridge',
            output='screen',
            parameters=[{
                'qos_overrides./model/a200_0000/robot/pose.publisher.reliability': 'reliable',
                'qos_overrides./model/a200_0000/robot/pose.publisher.durability': 'transient_local',
            }],
            arguments=[
                # Try with PoseArray instead of Pose
                '/model/a200_0000/robot/pose@geometry_msgs/msg/PoseArray@ignition.msgs.Pose_V',
                
                # Also try with a different message typec
                '/model/a200_0000/robot/pose@ros_gz_interfaces/msg/Pose_V@ignition.msgs.Pose_V',
            ],
        ),
    ])