"""Live viewer for the grasps the bridge just computed.

Every pick, the bridge writes the object's cloud and all GraspGen grasps it
got for it (camera frame) to:
    /tmp/pragmabot_live_cloud.npy
    /tmp/pragmabot_live_grasps.npz
This script watches those two files and redraws whenever they change, so
reloading the browser always shows the current object. It only reads files -
it does not call the GraspGen server or touch the robot.

Grasps are drawn in GraspGen's gripper convention (as GraspGen scored them);
colour = confidence (red low -> green high). These are all candidates, before
the bridge's tilt/width/IK filtering.

Run with GraspGen's venv:
    ~/GraspGen/.venv/bin/python3 calibration/visualize_live_grasps.py
    then open http://localhost:8080
"""

import argparse
import time
from datetime import datetime
from pathlib import Path

import numpy as np
from grasp_gen.utils.viser_utils import (
    create_visualizer,
    get_color_from_score,
    visualize_grasp,
    visualize_pointcloud,
)

CLOUD = Path("/tmp/pragmabot_live_cloud.npy")
GRASPS = Path("/tmp/pragmabot_live_grasps.npz")

# Camera optical frame (x right, y down, z forward) -> z-up display frame.
OPTICAL_TO_DISPLAY = np.array([[1, 0, 0, 0], [0, 0, 1, 0], [0, -1, 0, 0], [0, 0, 0, 1]], float)


def load():
    cloud = np.load(CLOUD).astype(np.float64)[:, :3]
    data = np.load(GRASPS)
    grasps = data["grasps"].astype(np.float64).copy()
    grasps[:, :3, 3] += data["centroid"]
    conf = np.asarray(data["confidences"], float).reshape(-1)
    return cloud, grasps, conf


def draw(vis, cloud, grasps, conf, top_k, min_conf, stamp):
    vis.scene.reset()
    to_disp = OPTICAL_TO_DISPLAY.copy()
    to_disp[:3, 3] = -(OPTICAL_TO_DISPLAY[:3, :3] @ cloud.mean(axis=0))

    pts = (to_disp[:3, :3] @ cloud.T).T + to_disp[:3, 3]
    visualize_pointcloud(vis, "object", pts, np.full((len(pts), 3), 200, np.uint8), size=0.003)

    order = np.argsort(-conf)
    order = order[conf[order] >= min_conf][:top_k]
    colors = get_color_from_score(conf[order], use_255_scale=True)
    for n, i in enumerate(order):
        g = to_disp @ grasps[i]
        visualize_grasp(vis, f"grasps/{n:04d}", g, color=colors[n], linewidth=0.6)

    msg = (f"{stamp}: {len(cloud)} points, showing {len(order)}/{len(grasps)} grasps "
           f"(conf {conf[order].min():.2f}-{conf[order].max():.2f})" if len(order)
           else f"{stamp}: {len(cloud)} points, no grasps >= {min_conf}")
    print(msg, flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--top_k", type=int, default=100, help="draw only the N most confident grasps")
    ap.add_argument("--min_conf", type=float, default=0.0)
    args = ap.parse_args()

    vis = create_visualizer(port=args.port)
    print(f"Watching {CLOUD} and {GRASPS} - open http://localhost:{args.port}", flush=True)
    shown = None
    while True:
        try:
            key = (CLOUD.stat().st_mtime, GRASPS.stat().st_mtime)
            if key != shown and time.time() - max(key) > 0.5:  # let the bridge finish writing
                stamp = datetime.fromtimestamp(max(key)).strftime("%H:%M:%S")
                draw(vis, *load(), args.top_k, args.min_conf, stamp)
                shown = key
        except (FileNotFoundError, ValueError, OSError, KeyError):
            pass  # not written yet, or caught mid-write - retry next poll
        time.sleep(1.0)


if __name__ == "__main__":
    main()
