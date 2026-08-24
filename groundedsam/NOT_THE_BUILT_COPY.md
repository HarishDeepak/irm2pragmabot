# This is the vendored source copy. It is never built.

Nothing here is compiled, installed, or run. It is a source snapshot kept in
the repo for reference and licensing (see `THIRD_PARTY_LICENSES.md`).

**The working install is `$HOME/groundedsam`.**

    $HOME/groundedsam/.venv/bin/python ...

Running or building from this directory silently does nothing useful - there
is no `install/`, so `ros2 launch` will not find the packages and edits here
never reach the robot.

For this repo's own ROS 2 packages (`pragmabot_bridge`,
`pragmabot_interfaces`), edit them under `pragmabot/ros2_ws/src/` and deploy
with `pragmabot/scripts/deploy_to_container.sh`.
