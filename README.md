# apollo_ws

ROS Repository for UMDLoop's 24-25 Rover.

To install packages:
```bash
rosdep install --from-paths src --ignore-src -r
```

For testing, please build using ``colcon build``, source the workspace using ``source install/setup.bash``, and:



```bash
ros2 launch apollo_ignition_bringup apollo_ignition.launch.py
```