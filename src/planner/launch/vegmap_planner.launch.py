import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from clearpath_config.common.utils.yaml import read_yaml
from clearpath_config.clearpath_config import ClearpathConfig


def generate_launch_description():
    # Packages
    pkg_planner = get_package_share_directory('planner')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')


    # Read robot YAML
    config = 'src/setup_path/robot.yaml'
    # Parse robot YAML into config
    clearpath_config = ClearpathConfig(config)

    namespace = clearpath_config.system.namespace
    platform_model = clearpath_config.platform.get_platform_model()
    
    # Get the launch directory
    package_dir = get_package_share_directory("planner")
    pkg_nav2_bringup = get_package_share_directory("nav2_bringup")

    # Paths to files and directories
    nav2_params_file = PathJoinSubstitution([pkg_planner, 'config', 'nav2_params.yaml'])
    rviz_config_file = os.path.join(package_dir, "config", "nav2.rviz")
    map_yaml_file = os.path.join(os.path.expanduser("~"), "vegmap", "src", "planner", "maps", "empty_map.yaml")
    localization_params_file = PathJoinSubstitution([pkg_planner,'config','localization_params.yaml'])

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
        ("/cmd_vel", PythonExpression(["'/", namespace, "/cmd_vel'"]))
    ]

    # Navigation stack
    # nav2 = GroupAction([
    #     SetRemap('/' + namespace + '/global_costmap/sensors/lidar2d_0/scan',
    #              '/' + namespace + '/sensors/lidar2d_0/scan'),
    #     SetRemap('/' + namespace + '/local_costmap/sensors/lidar2d_0/scan',
    #              '/' + namespace + '/sensors/lidar2d_0/scan'),

    #     IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_nav2_bringup,'launch','navigation_launch.py'])),
    #         launch_arguments=[
    #             ('use_sim_time', use_sim_time),
    #             ('params_file', nav2_params_file),
    #             ('use_composition', 'False'),
    #             # ('namespace', namespace)
    #           ]
    #     ),
    # ])
    # nav2_localization = GroupAction([
    #     IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_nav2_bringup, 'launch', 'localization_launch.py'])),
    #         launch_arguments=[
    #             ('map', map),
    #             ('use_sim_time', use_sim_time),
    #             ('params_file', localization_params_file),
    #             ('use_composition', 'False')
    #           ]
    #     ),
    # ])
    # Group localization nodes
    # nav2_localization = GroupAction([
    #     Node(
    #         package='nav2_map_server',
    #         executable='map_server',
    #         name='map_server',
    #         # namespace=namespace,
    #         output='screen',
    #         parameters=[
    #             {'yaml_filename': map_yaml_file},
    #             {'use_sim_time': use_sim_time}
    #         ],
    #         remappings=[
    #             ('/tf', '/tf'),
    #             ('/tf_static', '/tf_static')
    #         ]
    #     ),
    #     Node(
    #         package='nav2_amcl',
    #         executable='amcl',
    #         name='amcl',
    #         # namespace=namespace,
    #         output='screen',
    #         parameters=[localization_params_file],
    #         remappings=[
    #             ('/tf', '/tf'),
    #             ('/tf_static', '/tf_static'),
    #             ('scan', 'scan')
    #         ]
    #     ),
    #     Node(
    #         package='nav2_lifecycle_manager',
    #         executable='lifecycle_manager',
    #         name='lifecycle_manager_localization',
    #         # namespace=namespace,
    #         output='screen',
    #         parameters=[
    #             {'use_sim_time': use_sim_time},
    #             {'autostart': True},
    #             {'node_names': ['map_server', 'amcl']}
    #         ]
    #     ),
    #     Node(
    #         package='nav2_controller',
    #         executable='controller_server',
    #         name='controller_server',
    #         # namespace=namespace,
    #         output='screen',
    #         parameters=[
    #             nav2_params_file,
    #             {
    #                 'use_sim_time': use_sim_time,
    #                 # Override controller plugin to use RPP instead of DWB
    #                 'controller_plugins': ["FollowPath"],
    #                 'FollowPath.plugin': "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController",
    #                 'FollowPath.desired_linear_vel': 0.5,
    #                 'FollowPath.lookahead_dist': 0.6,
    #                 'FollowPath.min_lookahead_dist': 0.3,
    #                 'FollowPath.max_lookahead_dist': 0.9,
    #                 'FollowPath.lookahead_time': 1.5,
    #                 'FollowPath.transform_tolerance': 0.1,
    #                 'FollowPath.use_velocity_scaled_lookahead_dist': False,
    #                 'FollowPath.min_approach_linear_velocity': 0.05,
    #                 'FollowPath.approach_velocity_scaling_dist': 0.6
    #             }
    #         ]
    #     )
    # ])


    # Instead of including the standard nav2 launch files, include your custom one
    custom_nav_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_planner, "launch", "nav2_bringup.launch.py")]),
        launch_arguments=[('use_sim_time', use_sim_time)]
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
        namespace=namespace,
        output="screen",
    )

    # Include the gz_bridge launch file
    gz_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(package_dir, "launch", "gz_bridge.launch.py")]
        ),
        launch_arguments=[('namespace', namespace)]  
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
            ("namespace", namespace)
        ],
    )

    # Create push_namespace action
    push_namespace = PushRosNamespace(namespace=namespace)

    delayed_rviz = TimerAction(period=0.0, actions=[rviz])
    
    # delayed_nav2_localization = TimerAction(period=2.0, actions=[nav2_localization]) 

    # delayed_nav2 = TimerAction(period=5.0, actions=[nav2])

    return LaunchDescription(
        [
            # push_namespace,
            custom_nav_bringup,
            # delayed_nav2, 
            # delayed_nav2_localization,
            veg_costmap_updater,
            delayed_rviz,
            gz_bridge_launch,
        ]
    )