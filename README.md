# Vegmap

**Proprioceptive Navigation Through Vegetation**

## Installation

```bash
# Install dependencies
rosdep install -r --from-paths src -i -y --rosdistro humble
# Build
colcon_build
```

## When adding submodules

```bash
git submodule add -b humble https://github.com/clearpathrobotics/clearpath_common.git
git submodule update --init --recursive
```
