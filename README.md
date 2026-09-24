# MIMOSA ROS 2 Workspace

This repository is a complete ROS 2 workspace for
[MIMOSA](src/mimosa/README.md), a tightly coupled multi-modal sensor-fusion
framework. The workspace includes its source dependencies:

- `mimosa` and `mimosa_msgs`
- `gtsam`
- `gtsam_points`
- `config_utilities`

## Requirements

- Ubuntu 24.04
- ROS 2 Jazzy
- `colcon`
- `rosdep`
- A C++ compiler and standard build tools

Install ROS 2 Jazzy by following the
[official Ubuntu installation guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).
Then install the workspace tools if they are not already available:

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep build-essential
```

## Clone

Using HTTPS:

```bash
git clone https://github.com/talgoldman-ctrl/mimosa_ws.git
cd mimosa_ws
```

Alternatively, using SSH:

```bash
git clone git@github.com:talgoldman-ctrl/mimosa_ws.git
cd mimosa_ws
```

## Install dependencies

Initialize `rosdep` once on a new system:

```bash
sudo rosdep init
rosdep update
```

If `rosdep` is already initialized, skip `sudo rosdep init`. From the workspace
root, install all package dependencies:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

## Build

Build all packages from the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build \
  --symlink-install \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTING=OFF
```

The workspace contains the following packages:

```text
config_utilities
gtsam
gtsam_points
mimosa
mimosa_msgs
```

After a successful build, load the workspace environment:

```bash
source install/setup.bash
```

This command must be run in each new terminal after sourcing ROS 2:

```bash
source /opt/ros/jazzy/setup.bash
source /path/to/mimosa_ws/install/setup.bash
```

## Run

For example, launch MIMOSA with the `hornbill` sensor profile:

```bash
ros2 launch mimosa mimosa.launch.py profile:=hornbill viz:=true
```

For configuration, sensor profiles, rosbag playback, and dataset examples, see
the [MIMOSA package documentation](src/mimosa/README.md).

## Clean rebuild

To perform a clean build, remove the generated workspace directories and build
again:

```bash
rm -rf build install log
source /opt/ros/jazzy/setup.bash
colcon build \
  --symlink-install \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTING=OFF
```
