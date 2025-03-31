# 1. Intallation
```bash
#removed 
clearpath_common 
#and ran 
colcon build 
# but ran into cmake Error
sudo apt install ros-$ROS_DISTRO-moveit
sudo apt install ros-$ROS_DISTRO-moveit-setup-assistant

# another issue was not being able to control the robot in the simulation because there wasn't a controller
cd ~/vegmap/src
git clone https://github.com/ros-controls/gz_ros2_control -b humble
rosdep install -r --from-paths . --ignore-src --rosdistro humble -y

# 2. Robot Movement
```bash
# debugging robot movement
## change topic to
/a200_0000/cmd_vel
## from
/cmd_vel
```
# 3. Bridging Topics

```bash
# want to get robot's ground truth position from map
ign topic -l 
>> /model/a200_0000/robot/pose # contains robot position
>> /world/warehouse/state # state of all entities in the world
>> /world/warehouse/pose/info # pose information of all models

ign topic -t /model/a200_0000/robot/pose -e # this echos ground truth position
ign topic -t /world/warehouse/state -e
ign topic -t /world/warehouse/pose/info -e # contains item names, id, position

ros2 run rviz2 rviz2
 # add > pose > /robot/pose
```


```bash
# Runs simulation
ros2 launch clearpath_gz simulation.launch.py setup_path:=src/setup_path
# Bridges /world/warehouse/pose/info ground truth coordinates of objects
python3 ~/vegmap/src/planner/launch/pose_info_bridge.py
# Bridges /model/a200_0000/robot/pose
ros2 launch planner gz_bridge.launch.py

# Validate
ros2 topic echo
```


# 4. After merging
```bash
# warehouse
ros2 launch clearpath_gz simulation.launch.py setup_path:=src/setup_path

# flat veggies - weichen's testing environment
ros2 launch clearpath_gz simulation.launch.py setup_path:=src/setup_path world:=~/vegmap/src/planner/worlds/outdoors
# replace outdoors with
## grass world - artificial natural environment
## terrains - varying friction terrains

#rubicon world
ros2 launch clearpath_gz simulation.launch.py \
  setup_path:=src/setup_path \
  world:=~/vegmap/src/planner/worlds/rubicon/rubicon_world \
  z:=15

# run veggie planner
ros2 launch planner vegmap_planner.launch.py

# need map.yaml, following code converts map.sdf to map.yaml
python3 sdf_to_map.py ~/vegmap/src/clearpath_simulator/clearpath_gz/worlds/warehouse.sdf ~/vegmap/src/planner/maps warehouse_map

```


# 4. IMU and odometry analysis for determining mobility state

```bash
/a200_0000/platform/joint_states # encoder data
/a200_0000/sensors/imu_0/data # imu
/a200_0000/platform/odom # odometry calculated from wheel encoders
/a200_0000/platform/odom/filtered # filtered odometry, likely fused with IMU

/model/a200_0000/robot_pose # ground truth poistion of the robot
```


```bash
ros2 launch clearpath_gz simulation.launch.py setup_path:=src/setup_path world:=~/vegmap/src/planner/worlds/terrains

ros2 launch planner gz_bridge.launch.py

python3 src/planner/sensors/wheel_slip_monitor.py # shows realtime imu and pose stuf
python3 src/planner/sensors/ground_truth.py # show ground truth 
```