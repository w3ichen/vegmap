# For linux
source install/setup.bash


export GZ_SIM_RESOURCE_PATH=/home/patrick/vegmap/src/gazebo/gazebo-vegetation:$GZ_SIM_RESOURCE_PATH
export IGN_GAZEBO_RESOURCE_PATH=/home/patrick/vegmap/src/gazebo/gazebo-vegetation:$IGN_GAZEBO_RESOURCE_PATH
export GAZEBO_MODEL_PATH=/home/patrick/vegmap/src/gazebo/gazebo-vegetation/gazebo_vegetation/models:$GAZEBO_MODEL_PATH
ros2 launch clearpath_gz simulation.launch.py setup_path:=src/setup_path world:=~/vegmap/src/planner/worlds/outdoors

ros2 launch planner vegmap_planner.launch.py


# For mac
conda activate ros2
source install/setup.bash

ros2 launch clearpath_gz simulation.launch.py setup_path:=src/setup_path world:=outdoors


gz sim -v 4 -s src/planner/worlds/outdoors.sdf 
gz sim -g -v 4 --gui-config src/clearpath_simulator/clearpath_gz/config/gui.config

ros2 launch clearpath_gz robot_spawn.launch.py setup_path:=src/setup_path 



ros2 launch planner vegmap_planner.launch.py



ros2 run planner veg_costmap_updater.py 


ros2 run controller_manager ros2_control_node --ros-args -r __ns:=/a200_0000

ros2 run topic_tools relay /a200_0000/robot_description /a200_0000/controller_manager/robot_description
ros2 run topic_tools relay /a200_0000/robot_description /robot_description
ros2 topic echo /a200_0000/robot_description --full-length --once

ros2 launch clearpath_control control.launch.py setup_path:=src/setup_path

ros2 run topic_tools relay  /a200_0000/tf /tf
ros2 run topic_tools relay  /a200_0000/tf_static /tf_static

# View logs
ros2 run rqt_console rqt_console