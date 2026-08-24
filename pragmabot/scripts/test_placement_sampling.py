#!/usr/bin/env python3
"""test_placement_sampling.py — offline checks for the perceived-placement path.

Two halves, both robot-free:

  1. PURE (default) — synthetic masks only, no GPU, no servers. Checks the
     properties the placement point must have: it is inside the mask, it is
     away from the edges, plain FPS is genuinely worse, and a narrow surface
     degrades to a reported relaxed margin instead of failing.

  2. --live — the real thing against a saved scene: asks the running
     perception_server for a placement SURFACE, runs the same selection the
     bridge runs, back-projects with mask_to_pointcloud's own pinhole code,
     and transforms into fr3_link0 using a TF matrix read from the live tree
     (pass it with --tf, four rows of four numbers). Needs
     perception_server.py running; needs no robot motion.

        python3 scripts/test_placement_sampling.py
        python3 scripts/test_placement_sampling.py --live \\
            --scene extracted/red_cup --prompt "table." \\
            --tf "0.050 0.884 -0.466 0.912 0.999 -0.043 0.026 -0.064 \\
                  0.003 -0.466 -0.885 0.498 0 0 0 1"
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "calibration"))

import mask_sampling  # noqa: E402
import mask_to_pointcloud as m2p  # noqa: E402

PASS, FAIL = [], []


def check(name, ok, detail=""):
    (PASS if ok else FAIL).append(name)
    print(f"  {'ok  ' if ok else 'FAIL'} {name}" + (f"  — {detail}" if detail else ""))


# ----------------------------------------------------------------- pure

def rect_mask(h, w, y0, y1, x0, x1):
    m = np.zeros((h, w), dtype=bool)
    m[y0:y1, x0:x1] = True
    return m


def test_pure():
    print("\nPURE — synthetic masks, no GPU\n" + "-" * 60)

    # A 200x300 rectangular "table" in a 480x640 frame.
    mask = rect_mask(480, 640, 100, 300, 150, 450)

    d = mask_sampling.interior_distance(mask)
    check("interior_distance is 0 outside the mask", d[~mask].max() == 0)
    check("interior_distance is 1 on the boundary ring",
          d[100, 300] == 1 and d[299, 300] == 1)
    # The true distance at the centre is 100 px; the map deliberately
    # saturates at MAX_INTERIOR_PX because "deeper than 64 px inside" needs
    # no further resolution — every such pixel is equally safe, and the tie
    # is then broken by FPS order, i.e. by spread.
    check("interior_distance rises to the saturation cap at the centre",
          d[200, 300] == mask_sampling.MAX_INTERIOR_PX,
          f"centre={d[200, 300]} px, cap={mask_sampling.MAX_INTERIOR_PX} px")
    check("interior_distance grows monotonically inward",
          d[100, 300] < d[120, 300] < d[160, 300] <= d[200, 300],
          f"{d[100,300]} < {d[120,300]} < {d[160,300]} <= {d[200,300]}")

    u, v, info = mask_sampling.select_placement_pixel(mask)
    check("chosen pixel is inside the mask", bool(mask[v, u]), f"(u={u}, v={v})")
    check("chosen pixel clears the requested margin",
          info["interior_px"] >= mask_sampling.DEFAULT_MIN_INTERIOR_PX,
          f"{info['interior_px']} px from the edge")
    check("margin was not relaxed on a healthy surface", not info["relaxed"])

    # The point of the exercise: plain FPS picks corners, this does not.
    fps_only = mask_sampling.farthest_point_sampling(mask, n_points=16)
    fps_dist = d[fps_only[:, 1], fps_only[:, 0]]
    check("plain FPS puts candidates on the rim (that is the bug)",
          fps_dist.min() <= 2, f"worst plain-FPS candidate is {fps_dist.min()} px in")
    check("selection beats the plain-FPS worst case",
          info["interior_px"] > fps_dist.min(),
          f"{info['interior_px']} px vs {fps_dist.min()} px")

    # An L-shape: the centroid is OUTSIDE the mask, so any centroid-based
    # shortcut would place off the surface entirely.
    ell = rect_mask(480, 640, 100, 400, 150, 250) | rect_mask(480, 640, 300, 400, 150, 500)
    cy, cx = np.argwhere(ell).mean(axis=0).astype(int)
    u, v, info = mask_sampling.select_placement_pixel(ell)
    check("L-shaped surface: mask centroid is off the mask",
          not ell[cy, cx], f"centroid ({cx}, {cy})")
    check("L-shaped surface: chosen pixel is still on the mask", bool(ell[v, u]))

    # A 6px-wide strip cannot satisfy an 8px margin: it must relax and say so.
    strip = rect_mask(480, 640, 200, 206, 100, 500)
    u, v, info = mask_sampling.select_placement_pixel(strip)
    check("narrow surface relaxes the margin instead of failing",
          info["relaxed"] and bool(strip[v, u]),
          f"margin fell to {info['margin_px']} px")

    # Depth holes must be excluded before the choice, not after.
    valid = np.ones((480, 640), dtype=bool)
    valid[:, 300:] = False          # right half of the table has no depth
    u, v, info = mask_sampling.select_placement_pixel(mask, valid=valid)
    check("pixels without valid depth are never chosen", u < 300,
          f"(u={u}, v={v}) with depth only for u<300")

    try:
        mask_sampling.select_placement_pixel(np.zeros((10, 10), bool))
        check("empty mask raises", False)
    except ValueError:
        check("empty mask raises ValueError rather than returning a pixel", True)


# ----------------------------------------------------------------- live

def test_live(scene: Path, prompt: str, T_base_cam, out_png: Path):
    print(f"\nLIVE — {scene} / prompt {prompt!r}\n" + "-" * 60)

    from perception_client import PerceptionClient

    from PIL import Image
    rgb = np.array(Image.open(scene / "rgb.png").convert("RGB"))
    depth = np.load(scene / "depth.npy")
    K = json.loads((scene / "intrinsics.json").read_text())

    with PerceptionClient() as client:
        res = client.detect(rgb, depth, K, prompt, return_mask=True)

    if not res.ok:
        check(f"perception server answered for {prompt!r}", False, res.reason)
        return
    check(f"perception server answered for {prompt!r}", True,
          f"{res.label!r} conf={res.confidence:.3f}, {res.n_points} cloud points")
    check("server returned the 2D mask (return_mask)", res.mask is not None)
    if res.mask is None:
        return
    print(f"       mask covers {res.mask.sum()} px "
          f"({100.0 * res.mask.mean():.2f}% of the image)")

    valid = np.isfinite(depth) & (depth > 0) & (depth <= 3.0)
    u, v, info = mask_sampling.select_placement_pixel(res.mask, valid=valid)
    print(f"       chosen pixel (u={u}, v={v}), {info['interior_px']} px from the "
          f"edge, best of {info['n_candidates']} FPS candidates")
    check("chosen pixel is on the detected surface", bool(res.mask[v, u]))
    check("chosen pixel has valid depth", bool(valid[v, u]),
          f"depth={depth[v, u]:.3f} m")

    d = mask_sampling.interior_distance(res.mask & valid)
    fps_only = mask_sampling.farthest_point_sampling(res.mask & valid, 16)
    fps_dist = d[fps_only[:, 1], fps_only[:, 0]]
    check("beats plain FPS on this real mask too",
          info["interior_px"] >= fps_dist.max(),
          f"{info['interior_px']} px vs plain-FPS range "
          f"{fps_dist.min()}-{fps_dist.max()} px")

    # Back-project via the SAME code path the object cloud uses.
    half = 5 // 2
    patch = np.zeros(depth.shape, bool)
    patch[max(v - half, 0):v + half + 1, max(u - half, 0):u + half + 1] = True
    patch &= res.mask & valid
    patch_xyz = m2p.raw_cloud(depth, patch, K, 3.0, 0)
    point_cam = np.median(patch_xyz.astype(np.float64), axis=0)
    print(f"       camera frame : [{point_cam[0]:+.4f}, {point_cam[1]:+.4f}, "
          f"{point_cam[2]:+.4f}] m  (median of {len(patch_xyz)} patch points)")

    from perception_client import _as_intrinsics_dict
    K3 = np.array([[K["fx"], 0, K["cx"]], [0, K["fy"], K["cy"]], [0, 0, 1]])
    same = m2p.raw_cloud(depth, patch, _as_intrinsics_dict(K3), 3.0, 0)
    check("intrinsics given as a 3x3 matrix back-project identically",
          bool(np.allclose(same, patch_xyz)))

    lo, hi = m2p.aabb_bounds(res.points.astype(np.float64), 2.0, 0.05, 0.0)
    inside = bool(np.all((point_cam >= lo) & (point_cam <= hi)))
    check("point lands inside the surface's own visible extent", inside,
          f"x[{lo[0]:+.3f},{hi[0]:+.3f}] y[{lo[1]:+.3f},{hi[1]:+.3f}] "
          f"z[{lo[2]:+.3f},{hi[2]:+.3f}] m")

    point_base = (T_base_cam @ np.append(point_cam, 1.0))[:3]
    print(f"       fr3_link0    : [{point_base[0]:+.4f}, {point_base[1]:+.4f}, "
          f"{point_base[2]:+.4f}] m")

    # FR3 reach is ~0.855 m; anything outside that is not placeable and the
    # number is more likely a frame error than a real target.
    reach = float(np.linalg.norm(point_base[:2]))
    check("placement point is within the FR3's planar reach", reach <= 0.855,
          f"{reach:.3f} m from the base axis")

    # Whole-surface cross-check: transform the surface cloud too, so the
    # chosen point can be read against the surface's extent in ROBOT frame.
    cloud_base = (T_base_cam[:3, :3] @ res.points.astype(np.float64).T).T + T_base_cam[:3, 3]
    b_lo, b_hi = cloud_base.min(axis=0), cloud_base.max(axis=0)
    print(f"       surface in fr3_link0: x[{b_lo[0]:+.3f},{b_hi[0]:+.3f}] "
          f"y[{b_lo[1]:+.3f},{b_hi[1]:+.3f}] z[{b_lo[2]:+.3f},{b_hi[2]:+.3f}] m")
    check("chosen point is inside the surface's robot-frame extent",
          bool(np.all((point_base >= b_lo - 1e-6) & (point_base <= b_hi + 1e-6))))

    try:
        import cv2
        ann = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR).copy()
        cont, _ = cv2.findContours(res.mask.astype(np.uint8) * 255,
                                   cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(ann, cont, -1, (0, 255, 0), 2)
        for cu, cv_ in fps_only:
            cv2.circle(ann, (int(cu), int(cv_)), 6, (0, 0, 255), -1)
        cv2.drawMarker(ann, (u, v), (255, 255, 0), cv2.MARKER_TILTED_CROSS, 34, 4)
        cv2.imwrite(str(out_png), ann)
        print(f"       wrote {out_png} (green=mask, red=plain FPS, cyan X=chosen)")
    except ImportError:
        pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--live", action="store_true")
    ap.add_argument("--scene", default=str(REPO / "extracted" / "red_cup"))
    ap.add_argument("--prompt", default="table.")
    ap.add_argument("--tf", default=None,
                    help="16 numbers, row-major fr3_link0 <- camera optical")
    ap.add_argument("--out", default="/tmp/placement_check.png")
    args = ap.parse_args()

    test_pure()
    if args.live:
        if not args.tf:
            raise SystemExit("--live needs --tf (read it from "
                             "`ros2 run tf2_ros tf2_echo fr3_link0 "
                             "zed_left_camera_frame_optical`)")
        T = np.array([float(x) for x in args.tf.replace(",", " ").split()]).reshape(4, 4)
        test_live(Path(args.scene), args.prompt, T, Path(args.out))

    print("\n" + "=" * 60)
    print(f"{len(PASS)} passed, {len(FAIL)} failed")
    if FAIL:
        for f in FAIL:
            print(f"  FAILED: {f}")
        raise SystemExit(1)


if __name__ == "__main__":
    main()
