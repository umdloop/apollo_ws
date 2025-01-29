# apollo_ws

ROS Repository for UMDLoop's 24-25 Rover.

## Setup

## Install ROS Packages

```bash
rosdep install --from-paths src --ignore-src -r
```

## Building MAVSDK

You must build/install the MAVSDK from source or install a prebuilt version. See instructions [here](https://mavsdk.mavlink.io/v2.0/en/cpp/guide/installation.html).

## Building

```bash
colcon build
source install/setup.bash
ros2 launch apollo_gz_bringup apollo_gz.launch.py
```
