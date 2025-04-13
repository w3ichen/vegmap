#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    # get package directory
    pkg_dir = os.path.expanduser('~/vegmap/src/planner')

    # gz_bridge.launch.py
    gz_bridge_launch = os.path.join(pkg_dir, 'launch', 'gz_bridge.launch.py')

    # python scripts
    resistance_monitor_path = os.path.join(pkg_dir, 'resistance', 'resistance_monitor.py')
    cost_traverse_path = os.path.join(pkg_dir, 'resistance', 'cost_traverse.py')


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

    return ld