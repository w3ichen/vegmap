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
## resistance_zones

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

```bash
# check twist values or linear velocity
ros2 topic echo /a200_0000/platform/odom --field twist.twist.linear
```

# 5. traverse_cost.launch.py
```bash

# runs resistance zone map
# positions robot upper right corner
ros2 launch clearpath_gz simulation.launch.py setup_path:=src/setup_path world:=~/vegmap/src/planner/worlds/resistance_zone x:=9.0 y:=-9.0 yaw:=2.325

ros2 launch planner traverse_cost.launch.py
# launch file that launches

- gz_bridge.launch.py 
# bridges /model/a200_0000/robot_pose topic from gazebo to ROS2 - this topic contains ground truth position of the robot

- python3 ~/vegmap/src/planner/resistance/resistance_monitor.py 
# monitors robot's position to determine if it has entered a resistance zone
# applies resistance to obstacles from worlds/resistance_zone.sdf

- python3 ~/vegmap/src/planner/resistance/cost_traverse.py 
# compares cmd_vel with actual speed from /model/a200_0000/robot_pose to publish traverse cost
  # /cost_traverse  # topic that continuously publishes traversal cost
  # takes in ground truth position and cmd_velocity
  # calculates real time speed using grouth truth position change, compares to cmd_velocity to publish traversability cost




ros2 topic echo /cost_traverse # monitor real time published cost
```

# X. Costmap (Failed)
planner/maps/resistance_zone_costmap_params.yaml
planner/config/nav2_resistance_params.yaml
planner/launch/resistance_nav2.launch.py
planner/scripts/resistance_costmap_publisher.py
planner/launch/complete_resistance_system.launch.py

# 7. resistance_zone_2.sdf





# Weichen
```bash
# Weichen runs

ros2 launch clearpath_gz simulation.launch.py setup_path:=src/setup_path world:=~/vegmap/src/planner/worlds/outdoors
# and
ros2 launch planner vegmap_planner.launch.py
```


presentaiton 
problem
- assumptions
- steps we took and why
  

Qs
- when to replan (upon collision or constantly)
- save time vs energy
- x/y inplace rotate = false | except goal (DWA (slide), .yaml file)
- 


frame work | 