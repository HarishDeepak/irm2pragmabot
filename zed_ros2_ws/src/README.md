# zed_ros2_ws/src

Packages built on the **host** (not in the container), against `/opt/ros/humble`.

| Package | Purpose |
|---|---|
| `zed-ros2-wrapper/` | ZED2 driver. Locally modified: positional tracking + TF publishing disabled (the FR3 owns the TF tree), raw IMU enabled, `depth_mode: NEURAL_PLUS`. |
| `ros2-aruco-detector-main/` | ArUco marker detection — **required for hand-eye calibration**. |

`easy_handeye2` is **not duplicated here**. An identical copy lives at
`ros2_ws/franka_ros2/easy_handeye2/`. Symlink or copy it into this workspace if
you prefer to build it host-side:

```bash
ln -s ../../ros2_ws/franka_ros2/easy_handeye2 easy_handeye2
```

## Build

```bash
source /opt/ros/humble/setup.bash
cd zed_ros2_ws && colcon build --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=7      # required, or ZED topics are invisible
```
