#!/usr/bin/env python3
"""
make_object_pointcloud.py   (version 2 - with axis-aligned bounding box)
------------------------------------------------------------------------
Turns the 2D mask into the 3D object point cloud that GraspGen consumes,
AND removes junk points using an axis-aligned bounding box (AABB).

    mask.png + depth.npy + intrinsics.json  -->  object_pc.npy

WHY THE BOUNDING BOX
--------------------
The mask from SAM is not pixel perfect. A few pixels around the edge land on
the background instead of the object. Those pixels become 3D points that are
metres away from the real object. They:
    * make the object look enormous
    * drag the average position (the "centre") away from the real object
    * make GraspGen produce shifted, wrong grasps

The fix: work out where the object really is, draw a box around it, and throw
away every point outside that box.

To decide where the box goes we ignore the most extreme few percent of points
(--percentile), because those extremes ARE the junk we are trying to remove.

USAGE
-----
    python3 make_object_pointcloud.py --dir ~/pragmabot_data/extracted/scene_single_cup

    # see what it would remove, without changing anything:
    python3 make_object_pointcloud.py --dir ... --no-crop

    # if it is cutting off part of the real object, be less aggressive:
    python3 make_object_pointcloud.py --dir ... --percentile 1

    # if junk is still getting through, be more aggressive:
    python3 make_object_pointcloud.py --dir ... --percentile 5

    # also save a .ply you can open in MeshLab / CloudCompare:
    python3 make_object_pointcloud.py --dir ... --ply

REQUIREMENTS
------------
    pip install numpy opencv-python
"""

import argparse
import json
import os
import sys

import numpy as np

try:
    import cv2
except ImportError:
    sys.exit("ERROR: opencv-python not installed.  Run: pip install opencv-python")


# Rough real-world sizes, used only to warn you when something looks wrong.
PLAUSIBLE_MIN_CM = 1.0     # smaller than 1 cm is almost certainly noise
PLAUSIBLE_MAX_CM = 60.0    # bigger than 60 cm is almost certainly background


def write_ply(path, pts, colors=None):
    n = len(pts)
    with open(path, "w") as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {n}\n")
        f.write("property float x\nproperty float y\nproperty float z\n")
        if colors is not None:
            f.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
        f.write("end_header\n")
        if colors is None:
            for pt in pts:
                f.write(f"{pt[0]:.6f} {pt[1]:.6f} {pt[2]:.6f}\n")
        else:
            for pt, c in zip(pts, colors):
                f.write(f"{pt[0]:.6f} {pt[1]:.6f} {pt[2]:.6f} "
                        f"{int(c[2])} {int(c[1])} {int(c[0])}\n")


def describe(pts, title):
    """Print size and centre of a set of points, in centimetres."""
    lo = pts.min(axis=0)
    hi = pts.max(axis=0)
    size_cm = (hi - lo) * 100.0
    centre = pts.mean(axis=0)
    print(f"  {title}")
    print(f"     points : {len(pts)}")
    print(f"     size   : {size_cm[0]:6.1f} x {size_cm[1]:6.1f} x {size_cm[2]:6.1f} cm")
    print(f"     centre : x={centre[0]:+.3f}  y={centre[1]:+.3f}  z={centre[2]:+.3f} m")
    return size_cm, centre


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", required=True, help="scene folder")
    ap.add_argument("--percentile", type=float, default=2.0,
                    help="ignore this %% of extreme points when placing the box "
                         "(default 2). Higher = more aggressive cleaning.")
    ap.add_argument("--pad", type=float, default=0.02,
                    help="grow the box by this many metres so the real object "
                         "is not clipped (default 0.02 = 2 cm)")
    ap.add_argument("--no-crop", action="store_true",
                    help="show what WOULD be removed, but keep every point")
    ap.add_argument("--ply", action="store_true", help="also write object_pc.ply")
    ap.add_argument("--max-points", type=int, default=0,
                    help="randomly reduce to at most this many points (0 = keep all)")
    args = ap.parse_args()

    scene = os.path.expanduser(args.dir)
    paths = {n: os.path.join(scene, n) for n in
             ("mask.png", "depth.npy", "intrinsics.json", "rgb.png")}

    for n in ("mask.png", "depth.npy", "intrinsics.json"):
        if not os.path.isfile(paths[n]):
            sys.exit(f"ERROR: {paths[n]} not found.\n"
                     "  mask.png comes from segment_object.py\n"
                     "  depth.npy / intrinsics.json come from extract_bag.py")

    # ---------------------------------------------------------------- load
    mask = cv2.imread(paths["mask.png"], cv2.IMREAD_GRAYSCALE) > 127
    depth = np.load(paths["depth.npy"])
    intr = json.load(open(paths["intrinsics.json"]))
    fx, fy, cx, cy = intr["fx"], intr["fy"], intr["cx"], intr["cy"]

    if mask.shape != depth.shape:
        print(f"Resizing mask {mask.shape} -> depth {depth.shape}")
        mask = cv2.resize(mask.astype(np.uint8), (depth.shape[1], depth.shape[0]),
                          interpolation=cv2.INTER_NEAREST).astype(bool)

    if intr.get("width") and intr["width"] != depth.shape[1]:
        s = depth.shape[1] / float(intr["width"])
        print(f"Scaling intrinsics by {s:.4f}")
        fx, fy, cx, cy = fx * s, fy * s, cx * s, cy * s

    # -------------------------------------------------- pixels -> 3D points
    valid = mask & np.isfinite(depth) & (depth > 0)
    n_mask, n_valid = int(mask.sum()), int(valid.sum())
    print(f"\nMask pixels        : {n_mask}")
    print(f"  with valid depth : {n_valid} ({100.0 * n_valid / max(n_mask, 1):.1f}%)")

    if n_valid == 0:
        sys.exit("ERROR: this object has NO valid depth at all.\n"
                 "  The camera could not measure it (shiny? transparent? too close?)")
    if n_valid < 200:
        print("  >> WARNING: very few points. GraspGen will struggle.")

    rows, cols = np.nonzero(valid)
    z = depth[rows, cols].astype(np.float32)
    x = (cols.astype(np.float32) - cx) * z / fx
    y = (rows.astype(np.float32) - cy) * z / fy
    pts = np.stack([x, y, z], axis=1).astype(np.float32)

    raw_centre = pts.mean(axis=0)

    print("\nBEFORE cleaning:")
    describe(pts, "raw points from the mask")

    # ------------------------------------------- axis-aligned bounding box
    # We place the box using percentiles instead of the true min/max, because
    # the true min/max ARE the junk points we want to get rid of.
    lo = np.percentile(pts, args.percentile, axis=0) - args.pad
    hi = np.percentile(pts, 100.0 - args.percentile, axis=0) + args.pad

    print(f"\nAXIS-ALIGNED BOUNDING BOX  "
          f"(from the {args.percentile:.0f}-{100 - args.percentile:.0f}% range, "
          f"padded by {args.pad * 100:.0f} cm)")
    for i, axis in enumerate("xyz"):
        print(f"  {axis}: {lo[i]:+.3f} m  ->  {hi[i]:+.3f} m   "
              f"(width {(hi[i] - lo[i]) * 100:.1f} cm)")

    inside = np.all((pts >= lo) & (pts <= hi), axis=1)
    n_out = int((~inside).sum())
    print(f"\n  points inside the box : {int(inside.sum())}")
    print(f"  points OUTSIDE (junk) : {n_out} ({100.0 * n_out / len(pts):.1f}%)")

    if args.no_crop:
        print("\n  --no-crop given: keeping everything (nothing removed).")
    else:
        pts = pts[inside]
        rows, cols = rows[inside], cols[inside]
        if len(pts) == 0:
            sys.exit("ERROR: cropping removed everything. Try --percentile 0.5")

        print("\nAFTER cleaning:")
        _, centre_after = describe(pts, "cropped points")

        shift_cm = float(np.linalg.norm(centre_after - raw_centre) * 100.0)
        print(f"\n  the centre moved by {shift_cm:.2f} cm after cleaning")
        if shift_cm > 1.0:
            print("     >> Big shift. Those junk points were badly distorting the")
            print("        object centre - which is exactly what makes GraspGen")
            print("        produce shifted, wrong grasps.")
        else:
            print("     >> Small shift. This scene was already fairly clean.")

    # ------------------------------------------------- optional downsample
    if args.max_points and len(pts) > args.max_points:
        idx = np.random.choice(len(pts), args.max_points, replace=False)
        pts, rows, cols = pts[idx], rows[idx], cols[idx]
        print(f"\n  downsampled to {args.max_points} points")

    # ---------------------------------------------------- plausibility check
    final_size = (pts.max(axis=0) - pts.min(axis=0)) * 100.0
    biggest = float(final_size.max())
    print("\nSANITY CHECK")
    print(f"  object size : {final_size[0]:.1f} x {final_size[1]:.1f} x "
          f"{final_size[2]:.1f} cm")
    if biggest > PLAUSIBLE_MAX_CM:
        print(f"  >> TOO BIG (over {PLAUSIBLE_MAX_CM:.0f} cm). Background is still")
        print("     getting through. Try a larger --percentile (e.g. 5), or open")
        print("     mask_overlay.png - SAM may have selected the table.")
    elif biggest < PLAUSIBLE_MIN_CM:
        print(f"  >> TOO SMALL (under {PLAUSIBLE_MIN_CM:.0f} cm). Probably noise,")
        print("     not a real object.")
    else:
        print("  >> Looks plausible for a graspable object. Good.")

    # --------------------------------------------------------------- save
    out_npy = os.path.join(scene, "object_pc.npy")
    np.save(out_npy, pts)
    print(f"\nWrote {out_npy}   ({len(pts)} points)")

    box_json = os.path.join(scene, "bbox.json")
    with open(box_json, "w") as f:
        json.dump({
            "type": "axis_aligned",
            "frame": intr.get("frame_id", "camera"),
            "min_xyz_m": [float(v) for v in lo],
            "max_xyz_m": [float(v) for v in hi],
            "size_cm": [float(v) for v in final_size],
            "centre_xyz_m": [float(v) for v in pts.mean(axis=0)],
        }, f, indent=2)
    print(f"Wrote {box_json}")

    if args.ply:
        colors = None
        if os.path.isfile(paths["rgb.png"]):
            rgb = cv2.imread(paths["rgb.png"])
            if rgb.shape[:2] == depth.shape:
                colors = rgb[rows, cols]
        out_ply = os.path.join(scene, "object_pc.ply")
        write_ply(out_ply, pts, colors)
        print(f"Wrote {out_ply}")

    print("\n(points are in the CAMERA frame, in metres - not the robot frame)")


if __name__ == "__main__":
    main()