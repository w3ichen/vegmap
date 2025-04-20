#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # get package directory
    pkg_dir = os.path.expanduser('~/vegmap/src/planner')

    # gz_bridge.launch.py
    gz_bridge_launch = os.path.join(pkg_dir, 'launch', 'gz_bridge.launch.py')

    # python scripts
    resistance_monitor_path = os.path.join(pkg_dir, 'resistance', 'resistance_monitor.py')
    cost_traverse_path = os.path.join(pkg_dir, 'resistance', 'cost_traverse.py')
    costmap_generator_path = os.path.join(pkg_dir, 'resistance', 'costmap_generator.py')
    costmap_visualizer_path = os.path.join(pkg_dir, 'resistance', 'costmap_visualizer.py')

    # sdf_path = os.path.join(pkg_dir, 'worlds', 'resistance_zone.sdf')

    # launch description
    ld = LaunchDescription()

    # gazebo_bridge
    gz_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_bridge_launch]),
    )
    ld.add_action(gz_bridge)


    # resistance_monitor.py
    resistance_monitor = ExecuteProcess(
        cmd=['python3', resistance_monitor_path],
        name='resistance_monitor',
        output='screen',
    )
    ld.add_action(resistance_monitor)

    # cost_traverse.py
    cost_traverse = ExecuteProcess(
        cmd=['python3', cost_traverse_path],
        name='cost_traverse',
        output='screen',
    )
    ld.add_action(cost_traverse)

    # costmap_generator.py
    costmap_generator = ExecuteProcess(
        cmd = ['python3', costmap_generator_path, '--ros-args',
            '-p', 'sdf_path:=/home/patrick/vegmap/src/planner/worlds/resistance_zone.sdf', 
            '-p', 'map_resolution:=0.1',
            '-p', 'learning_rate:=0.3',
            ],
        name='costmap_generator',
        output='screen',
    )
    ld.add_action(costmap_generator)

    # costmap_visualizer.py
    costmap_visualizer = ExecuteProcess(
        cmd=['python3', costmap_visualizer_path],
        name='costmap_visualizer',
        output='screen',
    )
    ld.add_action(costmap_visualizer)

    # transform resistance_zones to map frame
    tf_static_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )
    ld.add_action(tf_static_publisher)
    


    # Rviz config file (map with traversability resistance zone)
    rviz_config_path = os.path.expanduser('~/vegmap/src/planner/launch/resistance_zones.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],

        parameters=[{'use_sim_time': True}],
    )
    ld.add_action(rviz_node)

    return ld