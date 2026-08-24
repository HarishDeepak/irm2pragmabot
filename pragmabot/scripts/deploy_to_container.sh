#!/usr/bin/env bash
# Sync this repo's ROS 2 packages into the tree the container actually builds,
# then rebuild them.
#
# WHY THIS EXISTS
# ---------------
# Development happens in the repo:
#     ~/irm2pragmabot/pragmabot/ros2_ws/src/{pragmabot_bridge,pragmabot_interfaces}
# The robot runs what the container built, and the container builds only what
# is bind-mounted into it:
#     ~/ros2_ws/franka_ros2  ->  (docker bind mount)  ->  /ros2_ws/src
#                            ->  colcon              ->  /ros2_ws/build  (imported at runtime)
# The repo is NOT on that path. Editing the repo changes nothing the robot
# runs until this script copies it across and rebuilds.
#
# That gap went unnoticed from 2026-08-14 to 2026-08-24: the container was
# running a 10-day-old bridge_node.py with no live perception, no placement,
# no fr3_limits.
#
#   ./scripts/deploy_to_container.sh --check    # report drift, change nothing
#   ./scripts/deploy_to_container.sh            # sync + rebuild
set -euo pipefail

REPO_SRC="$HOME/irm2pragmabot/pragmabot/ros2_ws/src"
MOUNT_SRC="$HOME/ros2_ws/franka_ros2"
CONTAINER="franka_ros2_humble"
PKGS=(pragmabot_interfaces pragmabot_bridge)   # interfaces first: bridge depends on it

check_only=0
[ "${1:-}" = "--check" ] && check_only=1

if [ ! -d "$MOUNT_SRC" ]; then
    echo "ERROR: $MOUNT_SRC missing - that is the bind-mounted build tree." >&2
    exit 1
fi

# Guard against the vendored look-alikes. ~/irm2pragmabot/ros2_ws/franka_ros2
# is a source-only copy of $MOUNT_SRC that is never built; deploying into it
# would look like it worked and change nothing.
if [ "$(readlink -f "$MOUNT_SRC")" != "$(readlink -f "$HOME/ros2_ws/franka_ros2")" ]; then
    echo "ERROR: refusing to deploy outside ~/ros2_ws/franka_ros2." >&2
    exit 1
fi

drift=0
for p in "${PKGS[@]}"; do
    if [ ! -d "$REPO_SRC/$p" ]; then
        echo "ERROR: $REPO_SRC/$p missing" >&2
        exit 1
    fi
    if diff -rq --exclude=__pycache__ --exclude='*.pyc' \
            "$REPO_SRC/$p" "$MOUNT_SRC/$p" >/dev/null 2>&1; then
        echo "  same    $p"
    else
        echo "  DRIFT   $p"
        { diff -rq --exclude=__pycache__ --exclude='*.pyc' \
            "$REPO_SRC/$p" "$MOUNT_SRC/$p" 2>&1 || true; } | sed 's/^/            /'
        drift=1
    fi
done

if [ "$check_only" = 1 ]; then
    [ "$drift" = 0 ] && echo "In sync - the container builds what the repo holds."
    exit $drift
fi

if [ "$drift" = 0 ]; then
    echo "Nothing to sync."
else
    for p in "${PKGS[@]}"; do
        rsync -a --delete --exclude=__pycache__ --exclude='*.pyc' \
            "$REPO_SRC/$p/" "$MOUNT_SRC/$p/"
        echo "  synced  $p"
    done
fi

# Drop these two packages' stale build/install trees first. The Aug-14 build
# was a plain (copying) ament_python build; re-running over it with
# --symlink-install leaves both layouts in place and python can import the
# stale copy instead of the symlink. Only these two packages are removed -
# franka_ros2's own build, which is expensive, is untouched.
echo "Rebuilding ${PKGS[*]} in $CONTAINER ..."
docker exec "$CONTAINER" bash -lc "
    set -e
    for p in ${PKGS[*]}; do rm -rf /ros2_ws/build/\$p /ros2_ws/install/\$p; done
    source /opt/ros/humble/setup.bash
    cd /ros2_ws
    colcon build --packages-select ${PKGS[*]} --symlink-install
"
echo
echo "Done. Verify the running code is the code you edited:"
echo "  md5sum $REPO_SRC/pragmabot_bridge/pragmabot_bridge/bridge_node.py"
echo "  docker exec $CONTAINER md5sum /ros2_ws/build/pragmabot_bridge/pragmabot_bridge/bridge_node.py"
