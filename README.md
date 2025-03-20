# Vegmap

**Proprioceptive Navigation Through Vegetation**

## Prerequisites

- ROS 2 Humble
- Gazebo Fortress
  - Install on [mac](https://gazebosim.org/docs/fortress/install_osx/): `brew install ignition-fortress`
  - On mac, gazebo installed via homebrew, may need to `conda remove --force libignition-gazebo6` to force use of homebrew gazebo.
- Python 3.10

## Installation

```bash
# Install dependencies
rosdep install -r --from-paths src -i -y

# Initial Build
colcon build --symlink-install

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

- AttributeError: module 'pkgutil' has no attribute 'ImpImporter'. Did you mean: 'zipimporter'?
  - https://stackoverflow.com/questions/77364550/attributeerror-module-pkgutil-has-no-attribute-impimporter-did-you-mean

```bash
python -m ensurepip --upgrade
python -m pip install --upgrade setuptools
pip install --upgrade setuptools
python -m ensurepip --upgrade
```
