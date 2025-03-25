import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # Get the launch directory
    package_dir = get_package_share_directory("planner")

    # Paths to files and directories
    nav2_params_file = os.path.join(package_dir, "config", "nav2_params.yaml")
    rviz_config_file = os.path.join(package_dir, "config", "nav2.rviz")

    # Launch configuration variables
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    # Define topic mappings: source topic -> destination topic
    remappings = {
        "/a200_0000/tf": "/tf",
        "/a200_0000/tf_static": "/tf_static",
        "/a200_0000/sensors/lidar3d_0/scan": "/scan",
        "/a200_0000/platform/odom/filtered": "/odom",
        # For cmd_vel, we want the opposite direction
        "/cmd_vel": "/a200_0000/cmd_vel",
        # Robot description
        "/a200_0000/robot_description": "/robot_description",
    }

    # Launch Navigation2
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("nav2_bringup"),
                    "launch",
                    "navigation_launch.py",
                )
            ]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": nav2_params_file,
            "remappings": remappings,
        }.items(),
    )

    # Launch RViz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    # Delay starting Nav2 to allow Gazebo and robot spawning to complete
    delayed_nav2 = TimerAction(period=1.0, actions=[nav2])
    ld.add_action(delayed_nav2)
    # Delay starting RViz until everything else is running
    delayed_rviz = TimerAction(period=2.0, actions=[rviz])
    ld.add_action(delayed_rviz)

    return ld
