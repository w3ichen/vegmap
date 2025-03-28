import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    namespace = 'a200_0000'
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    map_yaml_file = os.path.join(os.path.expanduser("~"), "vegmap", "src", "planner", "maps", "empty_map.yaml")
    nav2_params_file = os.path.join(get_package_share_directory('planner'), 'config', 'nav2_params.yaml')

    # Create the map server node
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace=namespace,
        output='screen',
        parameters=[
            {'yaml_filename': map_yaml_file},
            {'use_sim_time': use_sim_time}
        ]
    )

    # Create AMCL node
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        namespace=namespace,
        output='screen',
        parameters=[nav2_params_file]
    )

    # Create localization lifecycle manager
    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        namespace=namespace,
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['map_server', 'amcl']}
        ]
    )

    # Create a regulated pure pursuit controller instead of DWB
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        namespace=namespace,
        output='screen',
        parameters=[
            nav2_params_file,
            {
                'use_sim_time': use_sim_time,
                # Override controller plugin to use RPP instead of DWB
                'controller_plugins': ["FollowPath"],
                'FollowPath.plugin': "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController",
                'FollowPath.desired_linear_vel': 0.5,
                'FollowPath.lookahead_dist': 0.6,
                'FollowPath.min_lookahead_dist': 0.3,
                'FollowPath.max_lookahead_dist': 0.9,
                'FollowPath.lookahead_time': 1.5,
                'FollowPath.transform_tolerance': 0.1,
                'FollowPath.use_velocity_scaled_lookahead_dist': False,
                'FollowPath.min_approach_linear_velocity': 0.05,
                'FollowPath.approach_velocity_scaling_dist': 0.6
            }
        ]
    )

    # Create other required navigation nodes
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        namespace=namespace,
        output='screen',
        parameters=[nav2_params_file]
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        namespace=namespace,
        output='screen',
        parameters=[nav2_params_file]
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        namespace=namespace,
        output='screen',
        parameters=[nav2_params_file]
    )

    # Create the map->odom transform publisher
    tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_publisher',
        namespace=namespace,
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    # Navigation lifecycle manager
    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        namespace=namespace,
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator']}
        ]
    )

    # Group all nodes
    nav_nodes = GroupAction([
        map_server,
        amcl,
        lifecycle_manager_localization,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        tf_publisher,
        lifecycle_manager_navigation
    ])

    return LaunchDescription([nav_nodes])