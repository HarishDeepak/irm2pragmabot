#!/usr/bin/env bash
# record_object.sh - one-command capture -> segment -> point-cloud -> grasp
# recording, so grasp-selection logic (bridge_node's tilt/width/cross-axis
# gates) can be tuned OFFLINE later against real recorded data, without
# the robot, the camera, or being physically at the lab.
#
# Chains the three separate venvs this project already uses (see
# CLAUDE.md): system python3 for capture_scene.py, GroundedSAM's own venv
# for detect_object.py, mask_to_pointcloud.py's numpy-only step (any
# python3), and GraspGen's own venv for graspgen_client.py - exactly the
# "Full chain" already documented in CLAUDE.md, just as one command
# instead of four manual ones, and writing everything into ONE
# extracted/<name>_<timestamp>/ folder so a recording is one
# self-contained thing to git-commit / copy / replay from home.
#
# USAGE
#   calibration/record_object.sh <name> "<prompt>"
# e.g.
#   calibration/record_object.sh banana "banana."
#   calibration/record_object.sh spray_bottle "spray bottle."
#
# Requires graspgen_server.py already running (localhost:5556 by
# default) - this script does not start it, same as graspgen_client.py
# itself.

set -euo pipefail

if [ $# -lt 2 ]; then
    echo "usage: $0 <name> <prompt>" >&2
    echo '  e.g. record_object.sh banana "banana."' >&2
    exit 1
fi

NAME="$1"
PROMPT="$2"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="$REPO_ROOT/extracted/${NAME}_${STAMP}"
mkdir -p "$OUT"

echo "==> [1/4] capturing live RGB-D scene -> $OUT"
python3 "$REPO_ROOT/calibration/capture_scene.py" --out "$OUT"

echo "==> [2/4] segmenting '$PROMPT' (GroundedSAM, its own venv)"
~/groundedsam/.venv/bin/python "$REPO_ROOT/calibration/detect_object.py" \
    --rgb "$OUT/rgb.png" --prompt "$PROMPT" --out-dir "$OUT/detections"

echo "==> [3/4] back-projecting mask -> object point cloud"
python3 "$REPO_ROOT/calibration/mask_to_pointcloud.py" \
    --depth "$OUT/depth.npy" --intrinsics "$OUT/intrinsics.json" \
    --mask "$OUT/detections/mask.npy" --out "$OUT/object_pcd.npy"

echo "==> [4/4] querying GraspGen (its own venv) -> grasps.npz"
~/GraspGen/.venv/bin/python ~/GraspGen/client-server/graspgen_client.py \
    --pcd_file "$OUT/object_pcd.npy" --save_grasps "$OUT/grasps.npz"

echo
echo "recorded: $OUT"
ls -la "$OUT"
echo
echo "to re-run GraspGen later against this exact recording (from home, once"
echo "reachable - see graspgen_server.py running on Alonnisos):"
echo "  ~/GraspGen/.venv/bin/python ~/GraspGen/client-server/graspgen_client.py \\"
echo "      --pcd_file $OUT/object_pcd.npy --host <alonnisos-ip> --save_grasps <out.npz>"
