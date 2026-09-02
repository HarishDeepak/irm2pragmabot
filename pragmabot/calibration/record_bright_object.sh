#!/usr/bin/env bash
# record_bright_object.sh - same output as record_object.sh, but skips
# GroundingDINO entirely and segments by brightness within a fixed pixel
# ROI instead.
#
# WHY THIS EXISTS (2026-09-02, green AA battery). GroundingDINO kept
# matching a background desk object (a power bank - genuinely
# battery-shaped/labeled, not a random false positive) instead of the
# actual small glossy battery on the table, across three different
# prompts and two box_threshold values. No prompt wording fixed it,
# because the competing match is semantically real, not a phrasing
# miss. For a small, glossy, thin object sitting on the near-black
# cloth, thresholding brightness within a hand-picked ROI around its
# known screen position is simpler and more reliable than fighting
# text-prompted detection - the object is a bright highlight against a
# near-black background, so this is a real segmentation, not a hack.
#
# USAGE
#   calibration/record_bright_object.sh <name> [x0 y0 x1 y1] [threshold]
# e.g. re-record the green AA battery at its last known screen position
# (defaults below == where it was captured on 2026-09-02):
#   calibration/record_bright_object.sh green_aa_battery
# or if it has moved on the table, give a fresh ROI (open the rgb.png
# from a plain capture first to read pixel coordinates off it):
#   calibration/record_bright_object.sh green_aa_battery 500 300 620 380
#
# `threshold` is on sum(R+G+B), range 0-765 - higher rejects more of the
# background, lower keeps more of the object but risks including cloth.
# Check `back-projected N points, extent ...` in the output: a real AA
# battery should land around 4-5 x 1-1.5 x 0.5-1 cm. Wildly larger means
# the ROI or threshold is too loose; wildly smaller/fewer points means
# too tight - adjust and rerun.
#
# Requires graspgen_server.py already running (localhost:5556 by
# default), same as record_object.sh.

set -euo pipefail

if [ $# -lt 1 ]; then
    echo "usage: $0 <name> [x0 y0 x1 y1] [threshold]" >&2
    exit 1
fi

NAME="$1"
X0="${2:-660}"
Y0="${3:-180}"
X1="${4:-780}"
Y1="${5:-250}"
THRESH="${6:-260}"

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="$REPO_ROOT/extracted/${NAME}_${STAMP}"
mkdir -p "$OUT/detections"

echo "==> [1/4] capturing live RGB-D scene -> $OUT"
python3 "$REPO_ROOT/calibration/capture_scene.py" --out "$OUT"

echo "==> [2/4] segmenting by brightness (ROI x:[$X0,$X1] y:[$Y0,$Y1], threshold=$THRESH)"
python3 - "$OUT" "$X0" "$Y0" "$X1" "$Y1" "$THRESH" <<'PYEOF'
import sys, json
import numpy as np
from PIL import Image

out, x0, y0, x1, y1, thresh = sys.argv[1], *map(int, sys.argv[2:6]), int(sys.argv[6])
rgb = np.array(Image.open(f"{out}/rgb.png").convert("RGB"))

roi = np.zeros(rgb.shape[:2], dtype=bool)
roi[y0:y1, x0:x1] = True
bright = rgb.astype(np.int32).sum(axis=2) > thresh
mask = roi & bright

if mask.sum() < 10:
    sys.exit(f"only {mask.sum()} pixels passed - lower --threshold or widen the ROI")

np.save(f"{out}/detections/mask.npy", mask)

overlay = rgb.copy()
overlay[mask] = (0.4 * overlay[mask] + 0.6 * np.array([255, 0, 255])).astype(np.uint8)
Image.fromarray(overlay).save(f"{out}/detections/annotated.jpg")

ys, xs = np.where(mask)
bbox = [int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())]
json.dump({
    "image_path": f"{out}/rgb.png",
    "select": "MANUAL - brightness threshold, not GroundingDINO",
    "roi_xyxy": [x0, y0, x1, y1],
    "brightness_threshold": thresh,
    "detections": [{"bbox_xyxy": bbox, "mask_pixel_count": int(mask.sum())}],
}, open(f"{out}/detections/detections.json", "w"), indent=2)
print(f"mask pixels: {mask.sum()}, bbox: {bbox}")
PYEOF

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
echo "check detections/annotated.jpg to confirm the magenta overlay actually"
echo "landed on the object before trusting this recording."
