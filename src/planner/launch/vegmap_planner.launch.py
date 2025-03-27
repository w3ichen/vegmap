import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    # Declare the namespace as a LaunchConfiguration
    declare_namespace = DeclareLaunchArgument(
            'namespace',
            default_value='a200_0000',  # Set a meaningful default value here
            description='Top-level namespace'
        )
    
    # Get the launch directory
    package_dir = get_package_share_directory("planner")

    # Paths to files and directories
    nav2_params_file = os.path.join(package_dir, "config", "nav2_params.yaml")
    rviz_config_file = os.path.join(package_dir, "config", "nav2.rviz")

    map_file = os.path.join(os.path.expanduser("~"), "vegmap", "src", "planner", "maps", "warehouse_map.yaml")
    map_yaml_file = LaunchConfiguration("map", default=map_file)

    # Launch configuration variables
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    namespace = LaunchConfiguration('namespace')

    # Define topic remappings with correct namespace handling
    remappings = [
        ("tf", "/tf"),
        ("tf_static", "/tf_static"),
        ("sensors/lidar3d_0/scan", "/scan"),
        ("platform/odom/filtered", "/odom"),
        # For cmd_vel, we want the opposite direction
        ("/cmd_vel", PythonExpression(["'/", LaunchConfiguration('namespace'), "/cmd_vel'"]))
    ]

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
        }.items(),
    )

    # Launch RViz (outside the namespace)
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        parameters=[{
            "use_sim_time": use_sim_time,
        }],
        remappings=remappings,
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

    # Create push_namespace action
    push_namespace = PushRosNamespace(namespace=namespace)
    
    # Delay starting Nav2 to allow Gazebo and robot spawning to complete
    delayed_nav2 = TimerAction(period=3.0, actions=[nav2])

    # Delay starting RViz (outside namespace)
    delayed_rviz = TimerAction(period=5.0, actions=[rviz])

    return LaunchDescription(
        [
            declare_namespace,
            push_namespace,
            delayed_nav2, 
            gz_bridge_launch,
            veg_costmap_updater,
            delayed_rviz,
        ]
    )