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

## Links

- [Clearpath_common Github](https://github.com/clearpathrobotics/clearpath_common/tree/humble)
- [Clearpath docs](https://docs.clearpathrobotics.com/docs/ros/)

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

- Starting gazebo fortress on mac

```bash
ign gazebo -s -r -v 4 src/clearpath_simulator/clearpath_gz/worlds/warehouse.sdf  --force-version 6
```

- `-s` is server
- `-r` is reset state
- `-v 4` is verbosity level (most verbose)
- `--force-version 6` is to force use of ignition gazebo 6 (homebrew gazebo fortress is version 6)

```bash
ign gazebo -g -v 4 --force-version 6 --gui-config src/clearpath_simulator/clearpath_gz/config/gui.config
```

- Failed to load entry point 'echo': Error importing numpy: you should not try to import numpy from

```bash
conda remove --force numpy
conda install -c conda-forge numpy
```
