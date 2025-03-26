import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Get the launch directory
    package_dir = get_package_share_directory("planner")

    # Paths to files and directories
    nav2_params_file = os.path.join(package_dir, "config", "nav2_params.yaml")
    rviz_config_file = os.path.join(package_dir, "config", "nav2.rviz")
    map_config_file = os.path.join(package_dir, "config", "map.yaml")

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

    # Include the gz_bridge launch file
    gz_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(package_dir, "launch", "gz_bridge.launch.py")]
        ),
        # You can pass arguments to the included launch file if needed
        launch_arguments={"param1": "value1", "param2": "value2"}.items(),
    )

    # Start the Vegetation Costmap Updater
    veg_costmap_updater = Node(
        package="planner",
        executable="veg_costmap_updater.py",
        name="veg_costmap_updater",
        output="screen",
        parameters=[
            {"map_width": 100},
            {"map_height": 100},
            {"map_resolution": 0.1},
            {"map_origin_x": -5.0},
            {"map_origin_y": -5.0},
            {"update_frequency": 2.0},
            {"tf_topic": "/outdoors_tf"},
            {"use_sim_time": use_sim_time},
        ],
    )

    # Launch Map Server
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{"yaml_filename": map_config_file}, {"use_sim_time": use_sim_time}],
    )

    # Launch lifecycle manager for map server
    lifecycle_manager_map = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": True},
            {"node_names": ["map_server"]},
        ],
    )

    # Delay starting Nav2 to allow Gazebo and robot spawning to complete
    delayed_nav2 = TimerAction(period=1.0, actions=[nav2])
    # Delay starting RViz until everything else is running
    delayed_rviz = TimerAction(period=2.0, actions=[rviz])

    return LaunchDescription(
        [
            delayed_nav2,
            delayed_rviz,
            gz_bridge_launch,
            veg_costmap_updater,
            map_server,
            lifecycle_manager_map,
        ]
    )
