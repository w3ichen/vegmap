# For mac
conda activate ros2
source install/setup.bash

gz sim -v 4 -s src/clearpath_simulator/clearpath_gz/worlds/warehouse.sdf
gz sim -g -v 4 --gui-config src/clearpath_simulator/clearpath_gz/config/gui.config

ros2 launch clearpath_gz simulation.launch.py setup_path:=src/setup_path
# ros2 launch clearpath_gz robot_spawn.launch.py setup_path:=src/setup_path


ros2 launch planner vegmap_planner.launch.py


ros2 run controller_manager ros2_control_node --ros-args -r __ns:=/a200_0000

ros2 run topic_tools relay /a200_0000/robot_description /a200_0000/controller_manager/robot_description
ros2 run topic_tools relay /a200_0000/robot_description /robot_description
ros2 topic echo /a200_0000/robot_description --full-length --once

ros2 launch clearpath_control control.launch.py setup_path:=src/setup_path