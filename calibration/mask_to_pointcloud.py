"""
mask_to_pointcloud.py — back-project a 2D detection mask + depth into an
object-scale 3D point cloud, ready for GraspGen.

This is the missing link CLAUDE.md's "GraspGen real-data test" flagged:
previously the crop going into GraspGen was a hand-picked bounding box
("There is no object segmentation in this loop yet"). Now that
detect_object.py (Grounded-SAM-2) produces a real mask, this script turns
that mask + depth.npy + intrinsics.json into the xyz.npy GraspGen's own
client already knows how to consume.

Only needs numpy — run with any Python that has it (system python3, or
either tool's venv).

USAGE (after running detect_object.py to get mask.npy):

    python3 calibration/mask_to_pointcloud.py \\
        --depth extracted/scene_single_cup/depth.npy \\
        --intrinsics extracted/scene_single_cup/intrinsics.json \\
        --mask extracted/scene_single_cup/detections/mask.npy \\
        --out extracted/scene_single_cup/detections/object_pcd.npy

Stereo depth cameras (ZED included) produce "flying pixel" noise right at
an object's silhouette edge — the stereo matcher can't cleanly resolve a
boundary pixel, so it interpolates between the near (object) and far
(background) depth, producing a comet-tail of bogus points trailing off
the object. Two defenses, both on by default:
  --erode-px    shrinks the mask inward before back-projecting, dropping
                the boundary ring where this noise lives.
  --z-outlier-mad  drops any remaining point whose depth is more than N
                scaled-MAD (median absolute deviation) from the median
                depth of the (already-eroded) cloud — robust to the
                skewed, one-sided outliers this artifact produces (a
                plain mean/std filter gets dragged by the same outliers
                it's supposed to catch).

Then feed straight into GraspGen (separate venv, separate step):

    source ~/GraspGen/.venv/bin/activate
    python3 ~/GraspGen/client-server/graspgen_server.py \\
        --gripper_config ~/GraspGen/GraspGenModels/checkpoints/graspgen_franka_panda.yml &

    python3 ~/GraspGen/client-server/graspgen_client.py \\
        --pcd_file extracted/scene_single_cup/detections/object_pcd.npy \\
        --visualize
"""

import argparse
import json
from pathlib import Path

import numpy as np


def erode_mask(mask: np.ndarray, iterations: int) -> np.ndarray:
    """Shrink a boolean mask inward by `iterations` pixels using a 3x3
    cross kernel (pure numpy — no cv2/scipy dependency, since this script
    needs to run in venvs that don't have either)."""
    m = mask
    for _ in range(iterations):
        padded = np.pad(m, 1, mode="constant", constant_values=False)
        m = (
            padded[1:-1, 1:-1] & padded[:-2, 1:-1] & padded[2:, 1:-1]
            & padded[1:-1, :-2] & padded[1:-1, 2:]
        )
    return m


def remove_z_outliers(xyz: np.ndarray, mad_thresh: float) -> np.ndarray:
    """Drop points whose z is more than `mad_thresh` scaled-MAD from the
    median z. MAD-based rather than mean/std-based because the flying-pixel
    artifact this targets is one-sided and would otherwise drag the mean/std
    used to judge it."""
    z = xyz[:, 2]
    median = np.median(z)
    mad = np.median(np.abs(z - median))
    if mad == 0:
        return xyz
    scaled_mad = 1.4826 * mad  # normal-consistent scaling, standard MAD convention
    keep = np.abs(z - median) <= mad_thresh * scaled_mad
    return xyz[keep]


def backproject(depth: np.ndarray, mask: np.ndarray, K: dict, max_range: float,
                 erode_px: int, z_outlier_mad: float, target_points: int,
                 seed: int = 0) -> np.ndarray:
    if depth.shape != mask.shape:
        raise ValueError(f"depth shape {depth.shape} != mask shape {mask.shape}")

    if erode_px > 0:
        mask = erode_mask(mask, erode_px)

    vs, us = np.nonzero(mask)
    z = depth[vs, us].astype(np.float32)

    valid = np.isfinite(z) & (z > 0) & (z <= max_range)
    us, vs, z = us[valid], vs[valid], z[valid]
    if len(z) == 0:
        raise RuntimeError("No valid depth points inside the mask — check --max-range, "
                            "--erode-px (too aggressive can erase a small/thin object "
                            "entirely), or whether mask/depth are actually aligned/same frame.")

    x = (us - K["cx"]) * z / K["fx"]
    y = (vs - K["cy"]) * z / K["fy"]
    xyz = np.stack([x, y, z], axis=1).astype(np.float32)

    if z_outlier_mad > 0:
        before = len(xyz)
        xyz = remove_z_outliers(xyz, z_outlier_mad)
        print(f"z-outlier filter: dropped {before - len(xyz)}/{before} points")

    if len(xyz) > target_points:
        rng = np.random.default_rng(seed)
        idx = rng.choice(len(xyz), size=target_points, replace=False)
        xyz = xyz[idx]

    return xyz


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--depth", required=True, help="depth.npy, float32 meters, shape (H, W)")
    parser.add_argument("--intrinsics", required=True, help="intrinsics.json with fx,fy,cx,cy")
    parser.add_argument("--mask", required=True, help="bool mask.npy, shape (H, W), from detect_object.py")
    parser.add_argument("--out", required=True, help="output .npy path — (N, 3) float32 camera-frame XYZ, meters")
    parser.add_argument("--max-range", type=float, default=3.0,
                         help="drop points farther than this (m) — filters background leaking into a loose mask")
    parser.add_argument("--erode-px", type=int, default=3,
                         help="shrink the mask inward by this many pixels before back-projecting, to drop the "
                              "silhouette-edge ring where stereo flying-pixel noise lives (0 to disable)")
    parser.add_argument("--z-outlier-mad", type=float, default=3.5,
                         help="drop points whose depth is more than this many scaled-MAD from the median depth "
                              "(0 to disable)")
    parser.add_argument("--target-points", type=int, default=2000,
                         help="downsample to at most this many points (GraspGen was verified working at ~2000 pts; "
                              "much larger clouds have caused CUDA OOM / discriminator crashes — see CLAUDE.md)")
    args = parser.parse_args()

    depth = np.load(args.depth)
    mask = np.load(args.mask).astype(bool)
    K = json.loads(Path(args.intrinsics).read_text())

    xyz = backproject(depth, mask, K, args.max_range, args.erode_px,
                       args.z_outlier_mad, args.target_points)

    out_path = Path(args.out).expanduser().resolve()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    np.save(out_path, xyz)

    print(f"{len(xyz)} points, extent (m): "
          f"x[{xyz[:,0].min():.3f}, {xyz[:,0].max():.3f}] "
          f"y[{xyz[:,1].min():.3f}, {xyz[:,1].max():.3f}] "
          f"z[{xyz[:,2].min():.3f}, {xyz[:,2].max():.3f}]")
    print(f"Wrote {out_path}")


if __name__ == "__main__":
    main()
