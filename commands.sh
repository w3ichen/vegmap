# For mac
conda activate ros2
source install/setup.bash

gz sim -v 4 -s src/clearpath_simulator/clearpath_gz/worlds/warehouse.sdf
gz sim -g -v 4 --gui-config src/clearpath_simulator/clearpath_gz/config/gui.config

ros2 launch clearpath_gz simulation.launch.py setup_path:=src/setup_path
# ros2 launch clearpath_gz robot_spawn.launch.py setup_path:=src/setup_path


ros2 launch planner vegmap_planner.launch.py
