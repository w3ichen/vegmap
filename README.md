# Vegmap

**Proprioceptive Navigation Through Vegetation**

## Prerequisites

- ROS 2 Humble
- Gazebo Fortress
  - Install on [mac](https://gazebosim.org/docs/fortress/install_osx/): `brew install ignition-fortress`
  - On mac, gazebo installed via homebrew, may need to `conda remove --force libignition-gazebo6` to force use of homebrew gazebo.
- Python 3.10
  - setuptools==77.0.1

## Installation

```bash
# Install dependencies
rosdep install -r --from-paths src -i -y

# Initial Build
make build
# colcon build --symlink-install


# Source the workspace
source install/setup.bash # For bash
source install/setup.zsh # For zsh
```

### When adding submodules

```bash
git submodule add -b humble https://github.com/clearpathrobotics/clearpath_common.git
git submodule update --init --recursive
```

## Usage

1. Launch the `clearpath_gz` simulation:

```bash
ros2 launch clearpath_gz simulation.launch.py setup_path:=src/setup_path
```

2. Running planner server

```bash
ros2 launch planner planner_server.launch.py
```

3. Start the planner
   Either start the planner client or send commands via terminal:

```bash
ros2 launch planner planner_client.launch.py
```

or

```bash
ros2 action send_goal /move_robot nav2_msgs/action/NavigateToPose <msg>
```

## Links

- [Clearpath_common Github](https://github.com/clearpathrobotics/clearpath_common/tree/humble)
- [Clearpath docs](https://docs.clearpathrobotics.com/docs/ros/)
- [Custom planer plugin in ROS2 Nav](https://docs.nav2.org/plugin_tutorials/docs/writing_new_nav2planner_plugin.html#)
  - [ROS Nav2 Sample code](https://github.com/ros-navigation/navigation2_tutorials/tree/humble)

## ROS Troubleshooting

- Robostack (Mac) packages for Humble: https://robostack.github.io/humble.html

- AttributeError: module 'pkgutil' has no attribute 'ImpImporter'. Did you mean: 'zipimporter'?
  - https://stackoverflow.com/questions/77364550/attributeerror-module-pkgutil-has-no-attribute-impimporter-did-you-mean

```bash
python -m ensurepip --upgrade
python -m pip install --upgrade setuptools
pip install --upgrade setuptools
python -m ensurepip --upgrade
```

- "package 'controller_manager' not found

```bash
conda install -c robostack-staging -c conda-forge ros-humble-controller-manager ros-humble-controller-interface ros-humble-controller-manager-msgs
```

- Failed to load entry point 'echo': Error importing numpy: you should not try to import numpy from

```bash
conda remove --force numpy
conda install -c conda-forge numpy
```

- If built bridge from source, need to uninstall conda's bridge

```bash
conda remove --force ros-humble-ros-gz-bridge ros-humble-ros-ign-bridge ros-humble-cv-bridge
```

- [Wrn] [gz.cc:102] Fuel world download failed because Fetch failed. Other errors
  Need to set env paths:

```bash
export GZ_SIM_RESOURCE_PATH=/path/to/vegmap/install/clearpath_gz/share/clearpath_gz/worlds:$GZ_SIM_RESOURCE_PATH
export IGN_GAZEBO_RESOURCE_PATH=/path/to/vegmap/install/clearpath_gz/share/clearpath_gz/worlds:$IGN_GAZEBO_RESOURCE_PATH
```

- libc++abi: terminating due to uncaught exception of type pluginlib::LibraryLoadException: Could not find library corresponding to plugin sdformat_urdf_plugin/SDFormatURDFParser. Make sure that the library 'sdformat_urdf_plugin' actually exists.

In `${CONDA_PREFIX}/lib`, run `ln -s libsdformat_urdf_plugin.so libsdformat_urdf_plugin.dylib`

Source: https://github.com/RoboStack/ros-humble/issues/104#issuecomment-1774209784

- Starting gazebo on mac
  In add `-s` to `gz_sim = IncludeLaunchDescription(` in `gz_sim.launch.py` to run server only.

```bash
# Set paths to meshes
export GZ_SIM_RESOURCE_PATH=/path/to/vegmap/src/clearpath_common:$GZ_SIM_RESOURCE_PATH
export IGN_GAZEBO_RESOURCE_PATH=/path/to/vegmap/src/clearpath_common:$IGN_GAZEBO_RESOURCE_PATH
# Terminal 1: start gz server
(base) gz sim -v 4 -s src/clearpath_simulator/clearpath_gz/worlds/warehouse.sdf
# Terminal 2: start gz gui
(base) gz sim -g -v 4 --gui-config src/clearpath_simulator/clearpath_gz/config/gui.config
# Terminal 3: spawn the robot
(ros2) ros2 launch clearpath_gz robot_spawn.launch.py setup_path:=src/setup_path
```

- To purge a file from git cache, such as `.pyc` files, run:

```bash
git rm --cached "**/*.pyc"
```

- `ImportError: The 'nspektr' package is required; normally this is bundled with this package so if you get this warning, consult the packager of your distribution.`
  Run `pip install --upgrade setuptools`

- `Waiting for '/a200_0000/controller_manager' node to exist`
  Run `conda install -c robostack-staging -c conda-forge --no-deps ros-humble-controller-manager ros-humble-ros2-control`
  Remove the conda install: `conda uninstall --force ros-humble-controller-manager ros-humble-controller-interface ros-humble-hardware-interface ros-humble-controller-manager-msgs ros-humble-ros2-control`
  Build ros2_control from source: `colcon build --cmake-args -DBUILD_TESTING=OFF -DPython3_FIND_VIRTUALENV=ONLY --packages-select controller_interface hardware_interface controller_manager_msgs controller_manager`
