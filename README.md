# Vegmap

**Proprioceptive Navigation Through Vegetation**

## Installation

```bash
# Install dependencies
rosdep install -r --from-paths src -i -y

# Initial Build
colcon build --symlink-install --packages-skip clearpath_generator_common clearpath_generator_gz

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
ros2 launch clearpath_gz simulation.launch.py
```

## Links

- [Clearpath_common Github](https://github.com/clearpathrobotics/clearpath_common/tree/humble)
- [Clearpath docs](https://docs.clearpathrobotics.com/docs/ros/)
