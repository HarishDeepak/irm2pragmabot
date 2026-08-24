#!/usr/bin/env bash
# test_zmq_pipeline.sh — glue only, no new logic.
#
# Chains the two already-built, already-tested ZMQ clients:
#   1. perception_client.py  (GroundedSAM detection server, port 5557)
#   2. GraspGen's graspgen_client.py (GraspGen server, port 5556)
# with one shared text prompt, so the whole GroundedSAM->GraspGen pipeline
# can be exercised from a single command against a saved scene.
#
# Requires both servers already running separately:
#   source ~/groundedsam/.venv/bin/activate && python3 calibration/perception_server.py
#   source ~/GraspGen/.venv/bin/activate && python3 ~/GraspGen/client-server/graspgen_server.py \
#       --gripper_config ~/GraspGen/GraspGenModels/checkpoints/graspgen_franka_panda.yml
#
# perception_client.py is pure stdlib+numpy+zmq/msgpack (its own docstring:
# "NO torch, NO cv2"), so it runs fine under GraspGen's venv too - no need
# to switch venvs mid-script.
#
# When VLM scene understanding (e.g. Claude API) replaces the manual prompt
# later: this script's $PROMPT is exactly the string that call would
# produce. Nothing downstream changes.
#
# Usage: test_zmq_pipeline.sh "cup" [scene_dir] [grasps_out.npz]
# (third arg is a bare output path -- the script adds --save_grasps itself)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROMPT="${1:?usage: test_zmq_pipeline.sh <prompt> [scene_dir]}"
SCENE_DIR="${2:-$SCRIPT_DIR/../extracted/red_cup}"
SAVE_GRASPS="${3:-}"

TMP_PCD="$(mktemp --suffix=.npy)"
trap 'rm -f "$TMP_PCD"' EXIT

source ~/GraspGen/.venv/bin/activate

echo "== 1/2: perception_client.py (GroundedSAM over ZMQ, prompt=${PROMPT@Q}) =="
python3 "$SCRIPT_DIR/perception_client.py" \
    --rgb "$SCENE_DIR/rgb.png" \
    --depth "$SCENE_DIR/depth.npy" \
    --intrinsics "$SCENE_DIR/intrinsics.json" \
    --prompt "$PROMPT" \
    --out "$TMP_PCD"

echo
echo "== 2/2: graspgen_client.py (GraspGen over ZMQ) =="
if [ -n "$SAVE_GRASPS" ]; then
    python3 ~/GraspGen/client-server/graspgen_client.py \
        --pcd_file "$TMP_PCD" --save_grasps "$SAVE_GRASPS"
else
    python3 ~/GraspGen/client-server/graspgen_client.py --pcd_file "$TMP_PCD"
fi
