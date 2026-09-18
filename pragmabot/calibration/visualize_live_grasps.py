"""visualize_live_grasps.py — view the ACTUAL grasp candidates from the last
live pick, not a fresh (stochastic) re-query.

bridge_node.py saves two files on every live pick attempt (see
live_perception.py's grasps_for() caller in bridge_node.py):

    /tmp/pragmabot_live_grasps.npz   grasps (recentred, camera frame),
                                      confidences, centroid
    /tmp/pragmabot_live_cloud.npy    the segmented object cloud, camera
                                      frame (uncentred)

Running `graspgen_client.py --visualize` separately re-queries the GraspGen
server fresh — since it's a diffusion model, that draws a NEW random sample
and can show different candidates than what the live pick actually got and
selected. This script instead loads exactly what was used for the last real
pick, so what you see here is what the robot actually chose from.

Needs GraspGen's own venv (viser, torch, trimesh):

    ~/GraspGen/.venv/bin/python3 calibration/visualize_live_grasps.py

Then open http://localhost:8080 (or --port) in a browser. Ctrl+C to exit.
"""

import argparse
import sys
import tempfile
from pathlib import Path

import numpy as np


def parse_args():
    tmp = Path(tempfile.gettempdir())
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--grasps", default=str(tmp / "pragmabot_live_grasps.npz"),
                  help="Path to the saved grasps npz (grasps, confidences, centroid)")
    p.add_argument("--cloud", default=str(tmp / "pragmabot_live_cloud.npy"),
                  help="Path to the saved object point cloud (camera frame)")
    p.add_argument("--gripper_name", default="franka_panda")
    p.add_argument("--topk", type=int, default=0,
                  help="Show only the top-K candidates by confidence "
                       "(0 = show all, which can look cluttered - the "
                       "photo you compared against a moment ago is exactly "
                       "what 'all' looks like on a 1200-candidate response)")
    p.add_argument("--max_tilt_deg", type=float, default=0.0,
                  help="Keep only candidates whose approach axis (GraspGen "
                       "convention: local +Z) is within this many degrees "
                       "of straight-on to the object's OWN fitted normal - "
                       "the same tilt gate bridge_node.py applies (see "
                       "rank_grasp_indices), just computed here straight "
                       "from the point cloud instead of TF, so this script "
                       "needs no ROS. 0 = no tilt filter (the default, "
                       "matches the unfiltered photo).")
    p.add_argument("--max_crossaxis_deg", type=float, default=0.0,
                  help="Keep only candidates whose finger-closing axis "
                       "(local +X) is within this many degrees of "
                       "perfectly perpendicular to the object's fitted "
                       "long axis - same idea as bridge_node.py's "
                       "max_crossaxis_angle_deg. 0 = no filter.")
    p.add_argument("--port", type=int, default=8080)
    return p.parse_args()


def _fit_object_frame(points: np.ndarray, approach_axes: np.ndarray):
    """Self-contained (no grasp_transform import - this script runs under
    GraspGen's own venv, which has no ROS/rclpy, and grasp_transform.py
    imports rclpy at module level) re-derivation of the SAME object frame
    bridge_node.py builds: normal = smallest-eigenvalue direction of the
    cloud's 3D covariance (grasp_transform.fit_plane_normal's math), long
    axis = largest-variance direction WITHIN the plane perpendicular to
    that normal (grasp_transform.principal_axis_xy's math, generalised
    from a flat x/y assumption to an arbitrary camera-frame cloud).

    Sign of the normal is otherwise ambiguous (an eigenvector's sign is
    arbitrary, and this script has no TF to anchor "up" against, unlike
    bridge_node.py's `up_hint=[0,0,1]` in base frame) - resolved instead
    by picking whichever sign the MAJORITY of the candidates' own approach
    axes roughly oppose, since a top-down capture's grasps should mostly
    point back toward the camera/away from the object's far side.
    """
    p = points - points.mean(axis=0)
    cov = (p.T @ p) / len(p)
    vals, vecs = np.linalg.eigh(cov)  # ascending eigenvalues
    normal = vecs[:, 0]
    if float((approach_axes @ normal).mean()) > 0.0:
        normal = -normal

    p_in_plane = p - np.outer(p @ normal, normal)
    cov2 = (p_in_plane.T @ p_in_plane) / len(p_in_plane)
    vals2, vecs2 = np.linalg.eigh(cov2)
    long_axis = vecs2[:, -1]
    long_axis = long_axis - normal * float(long_axis @ normal)
    long_axis = long_axis / np.linalg.norm(long_axis)
    return normal, long_axis


def _filter_grasps(grasps, confidences, cloud, max_tilt_deg, max_crossaxis_deg):
    if max_tilt_deg <= 0.0 and max_crossaxis_deg <= 0.0:
        return grasps, confidences

    approach = grasps[:, :3, 2]   # local +Z, GraspGen's approach axis
    finger = grasps[:, :3, 0]     # local +X, GraspGen's finger-closing axis
    normal, long_axis = _fit_object_frame(cloud, approach)
    print(f"Fitted object normal (Z): {np.round(normal, 3)}, "
          f"long axis (X): {np.round(long_axis, 3)}")

    keep = np.ones(len(grasps), dtype=bool)

    if max_tilt_deg > 0.0:
        tilt_deg = np.degrees(np.arccos(np.clip(-(approach @ normal), -1.0, 1.0)))
        keep &= tilt_deg <= max_tilt_deg

    if max_crossaxis_deg > 0.0:
        f_in_plane = finger - normal * (finger @ normal)[:, None]
        f_norm = np.linalg.norm(f_in_plane, axis=1)
        f_norm[f_norm < 1e-9] = 1e-9
        f_in_plane = f_in_plane / f_norm[:, None]
        along = np.clip(np.abs(f_in_plane @ long_axis), 0.0, 1.0)
        crossaxis_deg = np.degrees(np.arcsin(along))
        keep &= crossaxis_deg <= max_crossaxis_deg

    print(f"Filter kept {int(keep.sum())}/{len(grasps)} candidates "
          f"(max_tilt_deg={max_tilt_deg:g}, max_crossaxis_deg={max_crossaxis_deg:g})")
    return grasps[keep], confidences[keep]


def main():
    args = parse_args()

    if not Path(args.grasps).is_file():
        sys.exit(f"{args.grasps} not found - run a live pick first "
                 "(bridge_node saves this on every attempt)")
    if not Path(args.cloud).is_file():
        sys.exit(f"{args.cloud} not found - run a live pick first")

    from grasp_gen.utils.viser_utils import (
        create_visualizer,
        get_color_from_score,
        visualize_grasp,
        visualize_pointcloud,
    )

    data = np.load(args.grasps)
    # Same convention as grasp_transform.load_all_grasps(): grasps are
    # saved relative to the recentred cloud, centroid must be added back.
    grasps = data["grasps"].astype(np.float64).copy()
    grasps[:, :3, 3] += data["centroid"]
    confidences = data["confidences"].astype(np.float64)

    cloud = np.load(args.cloud)[:, :3].astype(np.float64)

    grasps, confidences = _filter_grasps(
        grasps, confidences, cloud, args.max_tilt_deg, args.max_crossaxis_deg)
    if len(grasps) == 0:
        sys.exit("Filter left 0 candidates - loosen --max_tilt_deg / "
                 "--max_crossaxis_deg and try again")

    if args.topk > 0 and len(grasps) > args.topk:
        order = np.argsort(-confidences)[:args.topk]
        grasps = grasps[order]
        confidences = confidences[order]

    print(f"{len(grasps)} grasps loaded (conf {confidences.min():.3f}-"
          f"{confidences.max():.3f}), {len(cloud)} object points")

    vis = create_visualizer(port=args.port)

    pc_color = np.ones((len(cloud), 3), dtype=np.uint8) * 200
    visualize_pointcloud(vis, "point_cloud", cloud, pc_color, size=0.003)

    colors = get_color_from_score(confidences, use_255_scale=True)
    for i, grasp in enumerate(grasps):
        grasp = grasp.copy()
        grasp[3, 3] = 1.0
        visualize_grasp(
            vis,
            f"grasps/{i:03d}",
            grasp,
            color=colors[i],
            gripper_name=args.gripper_name,
            linewidth=0.6,
        )

    # The single highest-confidence candidate, drawn thicker and in a fixed
    # colour so it stands out from the confidence-coloured cloud of others.
    best = int(np.argmax(confidences))
    best_grasp = grasps[best].copy()
    best_grasp[3, 3] = 1.0
    visualize_grasp(
        vis, "grasps/best", best_grasp,
        color=[0, 120, 255], gripper_name=args.gripper_name, linewidth=2.5,
    )
    print(f"Best-confidence candidate ({confidences[best]:.3f}) highlighted in blue")

    print(f"\nViser visualization running at http://localhost:{args.port}")
    print("Red = low confidence, green = high confidence. Ctrl+C to exit.")
    import time
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
