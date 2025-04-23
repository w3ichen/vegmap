import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    DeclareLaunchArgument,
    GroupAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression,
    PathJoinSubstitution,
)
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from clearpath_config.common.utils.yaml import read_yaml
from clearpath_config.clearpath_config import ClearpathConfig


def generate_launch_description():
    # Packages
    pkg_planner = get_package_share_directory("planner")
    pkg_nav2_bringup = get_package_share_directory("nav2_bringup")

    # Read robot YAML
    config = "src/setup_path/robot.yaml"
    # Parse robot YAML into config
    clearpath_config = ClearpathConfig(config)

    namespace = clearpath_config.system.namespace
    platform_model = clearpath_config.platform.get_platform_model()

    # Get the launch directory
    package_dir = get_package_share_directory("planner")
    pkg_nav2_bringup = get_package_share_directory("nav2_bringup")

    # Paths to files and directories
    rviz_config_file = os.path.join(package_dir, "config", "nav2.rviz")
    map_yaml_file = os.path.join(
        os.path.expanduser("~"), "vegmap", "src", "planner", "maps", "empty_map.yaml"
    )

    if not os.path.exists(map_yaml_file):
        print(f"WARNING: Map file does not exist at: {map_yaml_file}")
        return

    # Launch configuration variables
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    # Define topic remappings with correct namespace handling
    remappings = [
        ("tf", "/tf"),
        ("tf_static", "/tf_static"),
        ("sensors/lidar3d_0/scan", "/scan"),
        ("platform/odom/filtered", "/odom"),
        # For cmd_vel, we want the opposite direction
        ("/cmd_vel", PythonExpression(["'/", namespace, "/cmd_vel'"])),
    ]

    # Instead of including the standard nav2 launch files, include your custom one
    custom_nav_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_planner, "launch", "nav2_bringup.launch.py")]
        ),
        launch_arguments=[("use_sim_time", use_sim_time)],
    )

    # Launch RViz (outside the namespace)
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        parameters=[
            {
                "use_sim_time": use_sim_time,
            }
        ],
        remappings=remappings,
        namespace=namespace,
        output="screen",
    )



    

    # Add this node to establish the map frame
    resistance_zone_to_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='resistance_zone_to_map_publisher',
        namespace=namespace,
        arguments=['0', '0', '0', '0', '0', '0', 'resistance_zone', 'map'],
        parameters=[{'use_sim_time': True}]
    )

    # Add a map to odom transform
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_publisher',
        namespace=namespace,
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': True}]
    )




    # Include the gz_bridge launch file
    gz_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(package_dir, "launch", "gz_bridge.launch.py")]
        ),
        launch_arguments=[("namespace", namespace)],
    )

    # Create push_namespace action
    # push_namespace = PushRosNamespace(namespace=namespace)
    delayed_rviz = TimerAction(period=0.0, actions=[rviz])


    # resistance_monitor.py
    resistance_monitor = Node(
        package="planner",
        executable="resistance_monitor.py",
        name="resistance_monitor",
        namespace=namespace,  
        output='screen',
    )
    

    return LaunchDescription(
        [
            # push_namespace,
            custom_nav_bringup,
            delayed_rviz,
            gz_bridge_launch,
            resistance_monitor,
            resistance_zone_to_map,
            map_to_odom,
        ]
    )
