# This is the vendored source copy. It is never built.

Nothing here is compiled, installed, or run. It is a source snapshot kept in
the repo for reference and licensing (see `THIRD_PARTY_LICENSES.md`).

**The working install is `$HOME/zed_ros2_ws`.**

    ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2

Running or building from this directory silently does nothing useful - there
is no `install/`, so `ros2 launch` will not find the packages and edits here
never reach the robot.

For this repo's own ROS 2 packages (`pragmabot_bridge`,
`pragmabot_interfaces`), edit them under `pragmabot/ros2_ws/src/` and deploy
with `pragmabot/scripts/deploy_to_container.sh`.
