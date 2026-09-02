# Recorded object data (2026-09-02)

Real captures from Alonnisos (ZED + GroundedSAM + GraspGen), taken so grasp
*selection* logic (`ros2_ws/src/pragmabot_bridge/pragmabot_bridge/
grasp_transform.py`, `bridge_node.py`) can be tuned offline from a laptop,
without the robot, the camera, or a GPU — no network reachability to
Alonnisos needed for that. Re-running GraspGen's actual model on new
inputs still needs its GPU (Alonnisos only); see "What you *can't* do
offline" below.

## How each recording was made

Two scripts in `calibration/`, both requiring `graspgen_server.py` and the
ZED wrapper already running on Alonnisos:

- **`record_object.sh <name> "<prompt>."`** — the normal path. Chains:
  1. `capture_scene.py` — one live RGB-D frame (`rgb.png`, `depth.npy`,
     `intrinsics.json`).
  2. `detect_object.py` (GroundedSAM, its own venv) — text-prompted
     segmentation (`detections/mask.npy`, `annotated.jpg`,
     `detections.json`).
  3. `mask_to_pointcloud.py` — back-projects the mask to a ~2000-point
     object cloud in camera frame (`object_pcd.npy`).
  4. `graspgen_client.py --save_grasps` (GraspGen, its own venv) — 100
     candidate grasps + confidences (`grasps.npz`).

- **`record_bright_object.sh <name> [x0 y0 x1 y1] [threshold]`** — bypasses
  GroundingDINO entirely and segments by brightness within a pixel ROI
  instead. Built for the green AA battery (see "Known issues" below);
  reusable for any small/glossy/thin object against the near-black cloth
  where text-prompted detection keeps losing to a background object.

Each run writes one self-contained folder, `<name>_<timestamp>/`, with
everything above plus `object_pcd.npy` and `grasps.npz`.

## Inventory

| folder | points | extent (cm, xyz) | GraspGen conf | detection |
|---|---|---|---|---|
| `carrot_20260902_172315` | 1227 | 2.2 x 11.5 x 6.8 | 0.88-0.97 | GroundedSAM conf=0.27 (needed `--box-threshold 0.15`, small/far object) |
| `carrot_20260902_172425` | 1233 | 2.2 x 11.5 x 6.9 | 0.88-0.96 | GroundedSAM conf=0.33 — **near-duplicate of the above, carrot wasn't moved between captures** |
| `green_aa_battery_20260902_173352` | 198 | 4.0 x 0.9 x 0.4 | 0.51-0.68 | **manual** brightness mask (see Known issues) — sparse, highlight-only cloud |
| `green_aa_battery_20260902_174019` | 175 | 1.1 x 4.0 x 1.6 | 0.54-0.73 | **manual** brightness mask, via `record_bright_object.sh` at its default ROI |
| `green_bowl_20260902_172514` | 2000 | 11.2 x 11.1 x 4.9 | 0.86-0.96 | GroundedSAM conf=0.77 |
| `green_pepper_20260902_173202` | 1768 | 5.2 x 5.5 x 3.5 | 0.62-0.91 | GroundedSAM conf=0.62 (prompt `"green bell pepper."`, `--box-threshold 0.2` — plain `"green pepper."` found nothing, small scalloped toy shape) |
| `marker_pen_20260902_172543` | 1073 | 1.5 x 12.5 x 6.1 | 0.84-0.97 | GroundedSAM conf=0.42 |
| `marker_pen_20260902_172630` | 1009 | 13.5 x 1.7 x 1.4 | 0.78-0.97 | GroundedSAM conf=0.53 — different orientation (~90 deg rotated) from the one above, real variety |
| `red_bowl_20260902_172012` | 2000 | 15.1 x 12.8 x 5.3 | 0.91-0.98 | GroundedSAM conf=0.86 |
| `red_cube_20260902_172829` | 2000 | 4.9 x 6.5 x 3.7 | 0.67-0.90 | GroundedSAM conf=0.74 |
| `red_cup_20260902_171936` | 2000 | 11.1 x 10.1 x 6.3 | 0.60-0.85 | GroundedSAM conf=0.95 |
| `spray_bottle_20260902_171131` | 2000 | 4.6 x 18.1 x 7.8 | 0.87-0.97 | GroundedSAM conf=0.61 |
| `spray_bottle_20260902_171234` | 2000 | 18.8 x 4.3 x 2.4 | 0.85-0.97 | GroundedSAM conf=0.59 — different orientation from the row above |
| `spray_bottle_20260902_172145` | 2000 | 4.5 x 18.1 x 6.8 | 0.84-0.97 | GroundedSAM conf=0.58 |
| `spray_bottle_20260902_172244` | 2000 | 18.7 x 5.1 x 2.3 | 0.82-0.93 | GroundedSAM conf=0.69 |
| `wooden_cube_20260902_173025` | 1878 | 4.8 x 5.6 x 3.3 | 0.68-0.94 | GroundedSAM conf=0.45, prompt `"small wood block."` (see Known issues) |
| `yellow_cube_20260902_173126` | 2000 | 5.4 x 6.5 x 3.9 | 0.73-0.96 | GroundedSAM conf=0.72 |

A `spray_bottle_20260902_170508` recording was made but stopped right
after capture (empty `detections/`, no point cloud or grasps) — deleted
rather than pushed as junk.

## Known issues hit while recording (don't re-derive these)

- **A word in the prompt can match the *background* instead of the
  object.** `"wooden cube."` scored a near-full-frame false positive
  (0.39) matching the background desk, narrowly beating the real small
  cube (0.38) — caught only because `mask_to_pointcloud.py` warns when
  the largest extent exceeds 60 cm, and GraspGen then refused to run on
  the resulting 83 cm "point cloud." Fixed by rephrasing to
  `"small wood block."`. Avoid prompt words that could also describe the
  surroundings (table material, wall, desk) — a plain shape/color word
  is safer than a compound description.
- **A real competing object beats prompt engineering, not just phrasing.**
  `"green AA battery."` and `"small battery."` (at two different
  `box_threshold` values, across two separate captures under different
  lighting) all matched the same background object instead of the actual
  battery — zoomed in and confirmed it's a power bank, i.e. a real
  battery-shaped/labeled product that legitimately outscores a tiny AA
  cell almost every time. No prompt wording reliably beats a real
  semantic competitor like this. Worked around with
  `record_bright_object.sh`'s brightness-threshold segmentation instead.
  The durable fix (not yet built) is restricting detections to the
  physical table's known 3D bounds (`bridge_node.py`'s
  `table_center_x`/`table_size_x`/`table_size_y`), since the confusable
  object sits on the side desk, off the table entirely.
- **Small/far objects need a lower `--box-threshold`.** Default 0.35
  found nothing for the carrot; 0.15 worked. `record_object.sh` doesn't
  expose this flag — fall back to running `detect_object.py` +
  `mask_to_pointcloud.py` + `graspgen_client.py` by hand (three commands,
  same as `record_object.sh`'s body) if a capture fails this way.

## Loading these for offline tuning

Pure numpy/scipy, no ROS/GPU/network needed for any of this:

```python
import numpy as np
from grasp_transform import load_all_grasps  # pragmabot_bridge/grasp_transform.py

object_pcd = np.load("spray_bottle_20260902_171234/object_pcd.npy")   # (N,3), camera frame
grasps = load_all_grasps("spray_bottle_20260902_171234/grasps.npz")   # (100,4,4), camera frame, centroid already re-added
confidences = np.load("spray_bottle_20260902_171234/grasps.npz")["confidences"]
```

`rank_grasp_indices`, `_crossaxis_factor`, `estimate_gripper_width`, and
`center_grasp_on_object` (all in `grasp_transform.py`) work directly on
these arrays. Note they're in **camera frame**, not the gravity-aligned
base frame (`fr3_link0`) the tilt gate assumes on the real robot — fine
for testing selection *mechanics* (ranking, cross-axis weighting, width
gates), but tilt/"vertical" comparisons won't mean the same thing until
transformed to base frame, which needs the live TF tree (robot-side
only).

## What you *can't* do offline

Re-running GraspGen's model on a genuinely new input (different
`num_grasps`/`grasp_threshold`/`max_tries`, or a new capture) needs its
GPU, which only exists on Alonnisos. `graspgen_client.py` itself is
lightweight (numpy/zmq/msgpack/trimesh, no torch on that code path), so
if Alonnisos becomes reachable from home later (VPN/SSH tunnel to
`graspgen_server.py`'s port, 5556 by default) it can run locally against
that. Until then, treat the point clouds and confidences already saved
here as fixed inputs.
