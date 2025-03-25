# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():
#     return LaunchDescription([
#         Node(
#             package='ros_gz_bridge',
#             executable='parameter_bridge',
#             name='model_pose_bridge',
#             output='screen',
#             parameters=[{
#                 'qos_overrides./model/a200_0000/robot/pose.publisher.reliability': 'reliable',
#                 'qos_overrides./model/a200_0000/robot/pose.publisher.durability': 'transient_local',
#                 # 'qos_overrides./world/warehouse/pose/info.publisher.reliability': 'reliable',
#                 'qos_overrides./world/warehouse/pose/info.publisher.durability': 'transient_local',
#             }],
#             arguments=[
#                 # Robot pose bridge - keeping the working one
#                 '/model/a200_0000/robot/pose@geometry_msgs/msg/PoseArray@ignition.msgs.Pose_V',
                
#                 # Warehouse pose info bridge - using PoseArray instead of Pose_V
#                 '/world/warehouse/pose/info@geometry_msgs/msg/PoseArray@ignition.msgs.Pose_V',
#                 # debugging code: ros2 interface show geometry_msgs/msg/PoseArray
#                 # if this doesn't work, create my own message
#             ],
#         ),
#     ])

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
                'qos_overrides./world/warehouse/pose/info.publisher.reliability': 'reliable',
                'qos_overrides./world/warehouse/pose/info.publisher.durability': 'transient_local',
            }],
            arguments=[
                # Robot pose bridge
                '/model/a200_0000/robot/pose@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
                
                # Warehouse pose info bridge
                '/world/warehouse/pose/info@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
            ],
        ),
    ])