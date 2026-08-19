"""
mask_to_pointcloud.py — back-project a 2D detection mask + depth into an
object-scale 3D point cloud, ready for GraspGen.

Turns a Grounded-SAM-2 mask + depth.npy + intrinsics.json into the xyz.npy
GraspGen's client already knows how to consume.

Only needs numpy — run with any Python that has it.

USAGE

    python3 calibration/mask_to_pointcloud.py \\
        --depth extracted/scene_single_cup/depth.npy \\
        --intrinsics extracted/scene_single_cup/intrinsics.json \\
        --mask extracted/scene_single_cup/detections/mask.npy \\
        --out extracted/scene_single_cup/detections/object_pcd.npy

    # find out which filter is actually doing the work, write nothing:
    python3 calibration/mask_to_pointcloud.py --depth ... --intrinsics ... \\
        --mask ... --out /dev/null --diagnose

FILTERS
-------
Stereo depth cameras (ZED included) produce "flying pixel" noise right at an
object's silhouette edge — the matcher interpolates between the near (object)
and far (background) depth, producing a comet-tail of bogus points trailing
off the object. Three defenses, all on by default:

  --erode-px       shrinks the mask inward before back-projecting, dropping
                   the boundary ring where this noise lives.

  --z-outlier-mad  drops points whose depth is more than N scaled-MAD from
                   the median depth — robust to the skewed, one-sided
                   outliers this artifact produces (a plain mean/std filter
                   gets dragged by the same outliers it is meant to catch).

  --aabb-pad-frac  crops to an axis-aligned bounding box built from the
                   robust percentile range of the cloud. This is the only
                   filter acting on x and y: the MAD filter is depth-only,
                   so mask leakage sideways onto the table — which has a
                   perfectly normal depth — survives it untouched while
                   still stretching the cloud laterally and dragging the
                   centroid off the object. GraspGen re-centres its input on
                   the cloud mean before denoising, so a centroid pulled
                   off-object shifts every grasp it generates.

ON THE AABB PADDING
-------------------
The pad is expressed as a FRACTION of the object's own percentile span, not
as an absolute distance. An absolute pad cannot serve both a 30 cm box and a
4.5 cm cube: 2 cm of margin is reasonable on the former and larger than the
object itself on the latter, which silently turns the filter into a no-op.
Use --aabb-pad-abs only if you specifically want a fixed margin.
"""

import argparse
import json
from pathlib import Path

import numpy as np

# Rough real-world bounds for a graspable tabletop object, used only to warn.
PLAUSIBLE_MIN_CM = 1.0
PLAUSIBLE_MAX_CM = 60.0


def erode_mask(mask: np.ndarray, iterations: int) -> np.ndarray:
    """Shrink a boolean mask inward by `iterations` pixels using a 3x3 cross
    kernel (pure numpy — no cv2/scipy, since this must run in venvs lacking
    both)."""
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
    median z."""
    z = xyz[:, 2]
    median = np.median(z)
    mad = np.median(np.abs(z - median))
    if mad == 0:
        return xyz
    scaled_mad = 1.4826 * mad  # normal-consistent scaling, standard convention
    keep = np.abs(z - median) <= mad_thresh * scaled_mad
    return xyz[keep]


def aabb_bounds(xyz: np.ndarray, percentile: float, pad_frac: float,
                pad_abs: float):
    """Axis-aligned box over the robust core of the cloud.

    Corners come from the [percentile, 100-percentile] range rather than the
    true min/max, because the true min/max ARE the stray points we want to
    discard — a box built from them would enclose them by construction.

    The box is then grown by `pad_frac` of its OWN span per axis (plus any
    absolute `pad_abs`), so the margin scales with the object instead of
    dwarfing small ones.
    """
    lo = np.percentile(xyz, percentile, axis=0)
    hi = np.percentile(xyz, 100.0 - percentile, axis=0)
    span = hi - lo
    pad = span * pad_frac + pad_abs
    return lo - pad, hi + pad


def aabb_crop(xyz: np.ndarray, percentile: float, pad_frac: float,
              pad_abs: float):
    lo, hi = aabb_bounds(xyz, percentile, pad_frac, pad_abs)
    inside = np.all((xyz >= lo) & (xyz <= hi), axis=1)
    return xyz[inside], lo, hi, int((~inside).sum())


def _extent_cm(xyz: np.ndarray) -> np.ndarray:
    return (xyz.max(axis=0) - xyz.min(axis=0)) * 100.0


def _fmt_extent(xyz: np.ndarray) -> str:
    e = _extent_cm(xyz)
    return f"{e[0]:6.1f} x {e[1]:6.1f} x {e[2]:6.1f} cm"


def raw_cloud(depth, mask, K, max_range, erode_px):
    """Back-project with no statistical filtering applied."""
    if depth.shape != mask.shape:
        raise ValueError(f"depth shape {depth.shape} != mask shape {mask.shape}")
    if erode_px > 0:
        mask = erode_mask(mask, erode_px)

    vs, us = np.nonzero(mask)
    z = depth[vs, us].astype(np.float32)
    valid = np.isfinite(z) & (z > 0) & (z <= max_range)
    us, vs, z = us[valid], vs[valid], z[valid]
    if len(z) == 0:
        raise RuntimeError(
            "No valid depth points inside the mask — check --max-range, "
            "--erode-px (too aggressive can erase a small/thin object entirely), "
            "or whether mask and depth are actually the same frame/resolution.")

    x = (us - K["cx"]) * z / K["fx"]
    y = (vs - K["cy"]) * z / K["fy"]
    return np.stack([x, y, z], axis=1).astype(np.float32)


def diagnose(depth, mask, K, args):
    """Report what EACH filter would remove on its own, so you can see which
    one is doing the work and which is a no-op. Writes nothing."""
    print("=" * 68)
    print("DIAGNOSTIC MODE — no output file will be written")
    print("=" * 68)

    n_mask = int(mask.sum())
    print(f"\nmask pixels                     : {n_mask}")
    if args.erode_px > 0:
        eroded = erode_mask(mask, args.erode_px)
        print(f"after --erode-px {args.erode_px}               : {int(eroded.sum())} "
              f"({n_mask - int(eroded.sum())} boundary pixels removed)")

    # Unfiltered baseline: no erosion, no stats.
    base = raw_cloud(depth, mask, K, args.max_range, 0)
    print(f"\nUNFILTERED cloud                : {len(base)} points")
    print(f"  extent                        : {_fmt_extent(base)}")
    print(f"  depth range                   : {base[:,2].min():.3f} - "
          f"{base[:,2].max():.3f} m")
    print(f"  centroid z                    : {base[:,2].mean():.3f} m")

    print("\nEACH FILTER APPLIED ALONE, to the unfiltered cloud:")

    if args.erode_px > 0:
        c = raw_cloud(depth, mask, K, args.max_range, args.erode_px)
        print(f"  erode {args.erode_px}px    -> {len(c):6d} pts "
              f"({len(base)-len(c):5d} dropped)  {_fmt_extent(c)}")

    if args.z_outlier_mad > 0:
        c = remove_z_outliers(base, args.z_outlier_mad)
        print(f"  z-MAD {args.z_outlier_mad:<4g}  -> {len(c):6d} pts "
              f"({len(base)-len(c):5d} dropped)  {_fmt_extent(c)}")

    if args.aabb_percentile > 0 and len(base) >= 20:
        c, lo, hi, dropped = aabb_crop(base, args.aabb_percentile,
                                       args.aabb_pad_frac, args.aabb_pad_abs)
        print(f"  aabb {args.aabb_percentile:g}%    -> {len(c):6d} pts "
              f"({dropped:5d} dropped)  {_fmt_extent(c)}")
        box = (hi - lo) * 100.0
        print(f"      box size  : {box[0]:.1f} x {box[1]:.1f} x {box[2]:.1f} cm")
        for i, axis in enumerate("xyz"):
            print(f"      {axis}: [{lo[i]:+.4f}, {hi[i]:+.4f}] m")
        if dropped == 0:
            print("      -> dropped nothing. Either the cloud is already clean,")
            print("         or the box is too loose: raise --aabb-percentile,")
            print("         or lower --aabb-pad-frac.")

    print("\nSCALE CHECK")
    print("  Compare the extents above against the object measured with a ruler.")
    print("  If they disagree badly, the problem is the mask or the depth,")
    print("  not the filters — no amount of tuning will fix it here.")
    print("=" * 68)


def backproject(depth, mask, K, max_range, erode_px, z_outlier_mad,
                aabb_percentile, aabb_pad_frac, aabb_pad_abs,
                target_points, seed=0) -> np.ndarray:
    xyz = raw_cloud(depth, mask, K, max_range, erode_px)

    centroid_raw = xyz.mean(axis=0)
    print(f"back-projected {len(xyz)} points, extent {_fmt_extent(xyz)}")

    if z_outlier_mad > 0:
        before = len(xyz)
        xyz = remove_z_outliers(xyz, z_outlier_mad)
        print(f"z-outlier filter: dropped {before - len(xyz)}/{before} points")

    if aabb_percentile > 0:
        if len(xyz) < 20:
            print("aabb filter: skipped, too few points to estimate percentiles")
        else:
            before = len(xyz)
            xyz, lo, hi, dropped = aabb_crop(xyz, aabb_percentile,
                                             aabb_pad_frac, aabb_pad_abs)
            if len(xyz) == 0:
                raise RuntimeError("AABB crop removed every point — lower "
                                   "--aabb-percentile or raise --aabb-pad-frac.")
            box = (hi - lo) * 100.0
            print(f"aabb filter: dropped {dropped}/{before} points "
                  f"(box {box[0]:.1f} x {box[1]:.1f} x {box[2]:.1f} cm, from the "
                  f"{aabb_percentile:g}-{100 - aabb_percentile:g}% range "
                  f"+ {aabb_pad_frac * 100:.0f}% pad)")
            if dropped == 0:
                print("    (nothing outside the box — the cloud was already "
                      "clean here, or the box is too loose. Run --diagnose.)")

    shift_cm = float(np.linalg.norm(xyz.mean(axis=0) - centroid_raw) * 100.0)
    print(f"after filtering: {len(xyz)} points, extent {_fmt_extent(xyz)}")
    print(f"centroid moved {shift_cm:.2f} cm vs the unfiltered cloud")
    if shift_cm > 1.0:
        print("    NOTE: a shift this large means the discarded points were "
              "meaningfully skewing the cloud centre. GraspGen re-centres on "
              "that mean, so this would have offset every generated grasp.")

    biggest = float(_extent_cm(xyz).max())
    if biggest > PLAUSIBLE_MAX_CM:
        print(f"    WARNING: largest extent {biggest:.1f} cm exceeds "
              f"{PLAUSIBLE_MAX_CM:.0f} cm — background is likely still leaking "
              "through. Inspect the mask, or raise --aabb-percentile.")
    elif biggest < PLAUSIBLE_MIN_CM:
        print(f"    WARNING: largest extent {biggest:.1f} cm is under "
              f"{PLAUSIBLE_MIN_CM:.0f} cm — likely noise, or --erode-px ate a "
              "small object.")

    if len(xyz) > target_points:
        rng = np.random.default_rng(seed)
        idx = rng.choice(len(xyz), size=target_points, replace=False)
        xyz = xyz[idx]

    return xyz


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--depth", required=True,
                        help="depth.npy, float32 meters, shape (H, W)")
    parser.add_argument("--intrinsics", required=True,
                        help="intrinsics.json with fx,fy,cx,cy")
    parser.add_argument("--mask", required=True,
                        help="bool mask.npy, shape (H, W), from detect_object.py")
    parser.add_argument("--out", required=True,
                        help="output .npy path — (N,3) float32 camera-frame XYZ, meters")
    parser.add_argument("--max-range", type=float, default=3.0,
                        help="drop points farther than this (m)")
    parser.add_argument("--erode-px", type=int, default=3,
                        help="shrink the mask inward by this many pixels before "
                             "back-projecting (0 to disable)")
    parser.add_argument("--z-outlier-mad", type=float, default=3.5,
                        help="drop points whose depth is more than this many "
                             "scaled-MAD from the median depth (0 to disable)")
    parser.add_argument("--aabb-percentile", type=float, default=2.0,
                        help="build the box from this percentile to its complement, "
                             "in x, y AND z (0 to disable). Raise it if background "
                             "still leaks through")
    parser.add_argument("--aabb-pad-frac", type=float, default=0.05,
                        help="grow the box by this FRACTION of its own span per axis "
                             "(default 0.05 = 5%%). Scale-aware, so it behaves the "
                             "same on a 4 cm cube and a 30 cm box")
    parser.add_argument("--aabb-pad-abs", type=float, default=0.0,
                        help="additional fixed margin in metres, on top of the "
                             "fractional pad. Usually leave at 0")
    parser.add_argument("--target-points", type=int, default=2000,
                        help="downsample to at most this many points (GraspGen "
                             "verified at ~2000; larger clouds have caused CUDA OOM)")
    parser.add_argument("--diagnose", action="store_true",
                        help="report what each filter removes on its own, then exit "
                             "without writing an output file")
    args = parser.parse_args()

    depth = np.load(args.depth)
    mask = np.load(args.mask).astype(bool)
    K = json.loads(Path(args.intrinsics).read_text())

    if args.diagnose:
        diagnose(depth, mask, K, args)
        return

    xyz = backproject(depth, mask, K, args.max_range, args.erode_px,
                      args.z_outlier_mad, args.aabb_percentile,
                      args.aabb_pad_frac, args.aabb_pad_abs,
                      args.target_points)

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