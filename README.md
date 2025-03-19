# Vegmap

**Proprioceptive Navigation Through Vegetation**

## Installation

```bash
# Install dependencies
rosdep install -r --from-paths src -i -y --verbose

# Initial Build
colcon build --packages-skip clearpath_generator_common

colcon_build
```

## When adding submodules

```bash
git submodule add -b humble https://github.com/clearpathrobotics/clearpath_common.git
git submodule update --init --recursive
```
