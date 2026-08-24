#!/usr/bin/env python3
"""capture_scene.py — grab one live RGB-D frame from the ZED, as arrays.

Two jobs, one implementation:

  1. CLI: save a scene to disk in exactly the layout extracted/<scene>/
     already uses (rgb.png, depth.npy, intrinsics.json), so every offline
     tool in calibration/ works on a freshly captured frame unchanged.

  2. `capture()` is the callable bridge_node's `_scene_source` needs:
     it returns (rgb, depth, intrinsics) and nothing else.

NO cv_bridge. It is not installed in every environment this must run in,
and the two encodings we care about decode to numpy in three lines each.

DEPTH ENCODING — the 1000x trap. ZED publishes depth as either
`32FC1` (float32 METRES) or `16UC1` (uint16 MILLIMETRES) depending on how
the wrapper is configured. Everything downstream (mask_to_pointcloud,
GraspGen, the TF composition) assumes metres. Reading 16UC1 as metres
silently puts the table 600 metres away and every guard still passes,
because the SHAPE of the cloud is right and only its scale is wrong. This
module therefore branches on `msg.encoding` explicitly and prints which
one it saw, rather than trusting `passthrough`.

QoS — the silent-subscription trap. The ZED wrapper publishes sensor data
BEST_EFFORT. A default (RELIABLE) subscription matches nothing and simply
never receives a message, with no error and no warning. The QoS below is
explicit for that reason; see scene_observer.py, which hit the same thing
during the ROS 1 -> 2 port.

USAGE
    python3 calibration/capture_scene.py --out extracted/live_$(date +%H%M%S)
    python3 calibration/capture_scene.py --out /tmp/scene --timeout 15
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

DEFAULT_COLOR = "/zed/zed_node/rgb/color/rect/image"
DEFAULT_DEPTH = "/zed/zed_node/depth/depth_registered"
DEFAULT_INFO = "/zed/zed_node/rgb/color/rect/image/camera_info"


def _decode_image(msg) -> np.ndarray:
    """sensor_msgs/Image -> numpy, for the encodings the ZED actually emits."""
    enc = msg.encoding.lower()
    buf = np.frombuffer(msg.data, dtype=np.uint8)

    if enc in ("rgb8", "bgr8"):
        img = buf.reshape(msg.height, msg.step // 1)[:, : msg.width * 3]
        img = img.reshape(msg.height, msg.width, 3)
        return img[:, :, ::-1] if enc == "bgr8" else img

    if enc in ("rgba8", "bgra8"):
        img = buf.reshape(msg.height, msg.step)[:, : msg.width * 4]
        img = img.reshape(msg.height, msg.width, 4)[:, :, :3]
        return img[:, :, ::-1] if enc == "bgra8" else img

    if enc == "32fc1":
        d = buf.view(np.float32).reshape(msg.height, msg.step // 4)
        return d[:, : msg.width].astype(np.float32)          # already metres

    if enc == "16uc1":
        d = buf.view(np.uint16).reshape(msg.height, msg.step // 2)
        # millimetres -> metres. The whole reason this branch is explicit.
        return (d[:, : msg.width].astype(np.float32)) / 1000.0

    if enc == "mono8":
        return buf.reshape(msg.height, msg.step)[:, : msg.width]

    raise ValueError(f"unhandled image encoding {msg.encoding!r}")


def capture(color_topic: str = DEFAULT_COLOR, depth_topic: str = DEFAULT_DEPTH,
            info_topic: str = DEFAULT_INFO, timeout_s: float = 15.0,
            node=None, verbose: bool = True):
    """Return (rgb, depth, intrinsics) from one live frame.

    `rgb` is HxWx3 uint8 RGB, `depth` is HxW float32 METRES, `intrinsics`
    is the flat {"fx","fy","cx","cy",...} dict every tool in calibration/
    already indexes.

    Pass `node` to reuse an existing rclpy node (bridge_node does this, so
    it does not spin up a second one). Otherwise a throwaway node is
    created and destroyed.
    """
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import CameraInfo, Image

    owns_context = not rclpy.ok()
    if owns_context:
        rclpy.init()
    owns_node = node is None
    if owns_node:
        node = Node("pragmabot_scene_capture")

    # BEST_EFFORT is mandatory here - see the module docstring.
    qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                     reliability=ReliabilityPolicy.BEST_EFFORT)

    got = {}
    subs = [
        node.create_subscription(Image, color_topic,
                                 lambda m: got.setdefault("rgb", m), qos),
        node.create_subscription(Image, depth_topic,
                                 lambda m: got.setdefault("depth", m), qos),
        node.create_subscription(CameraInfo, info_topic,
                                 lambda m: got.setdefault("info", m), qos),
    ]

    deadline = node.get_clock().now().nanoseconds + int(timeout_s * 1e9)
    while len(got) < 3 and node.get_clock().now().nanoseconds < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    for s in subs:
        node.destroy_subscription(s)

    missing = {"rgb": color_topic, "depth": depth_topic,
               "info": info_topic}.keys() - got.keys()
    if missing:
        names = {"rgb": color_topic, "depth": depth_topic, "info": info_topic}
        if owns_node:
            node.destroy_node()
        if owns_context:
            rclpy.shutdown()
        raise TimeoutError(
            "no message within %.0fs on: %s. The ZED publishes BEST_EFFORT - "
            "a RELIABLE subscription receives nothing silently. Check "
            "ROS_DOMAIN_ID=7 and `ros2 topic hz <topic>`."
            % (timeout_s, ", ".join(names[k] for k in missing)))

    rgb = _decode_image(got["rgb"])
    depth = _decode_image(got["depth"])
    info = got["info"]
    K = {"fx": float(info.k[0]), "fy": float(info.k[4]),
         "cx": float(info.k[2]), "cy": float(info.k[5]),
         "width": int(info.width), "height": int(info.height),
         "frame_id": info.header.frame_id}

    if verbose:
        finite = np.isfinite(depth) & (depth > 0)
        print(f"rgb   {rgb.shape} {rgb.dtype}  encoding={got['rgb'].encoding}")
        print(f"depth {depth.shape} {depth.dtype} encoding="
              f"{got['depth'].encoding} -> metres")
        print(f"      valid {finite.mean() * 100:.1f}%, "
              f"range {depth[finite].min():.3f}-{depth[finite].max():.3f} m")
        print(f"K     fx={K['fx']:.3f} fy={K['fy']:.3f} "
              f"cx={K['cx']:.3f} cy={K['cy']:.3f}  frame={K['frame_id']}")

    if owns_node:
        node.destroy_node()
    if owns_context:
        rclpy.shutdown()

    return rgb, depth, K


def main() -> None:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--out", required=True,
                   help="directory to write rgb.png / depth.npy / intrinsics.json")
    p.add_argument("--color-topic", default=DEFAULT_COLOR)
    p.add_argument("--depth-topic", default=DEFAULT_DEPTH)
    p.add_argument("--info-topic", default=DEFAULT_INFO)
    p.add_argument("--timeout", type=float, default=15.0)
    args = p.parse_args()

    rgb, depth, K = capture(args.color_topic, args.depth_topic,
                            args.info_topic, args.timeout)

    out = Path(args.out).expanduser().resolve()
    out.mkdir(parents=True, exist_ok=True)
    np.save(out / "depth.npy", depth)
    (out / "intrinsics.json").write_text(json.dumps(K, indent=2))
    try:
        from PIL import Image as PILImage
        PILImage.fromarray(rgb).save(out / "rgb.png")
    except ImportError:
        np.save(out / "rgb.npy", rgb)
        print("PIL not available - wrote rgb.npy instead of rgb.png")

    print(f"\nwrote {out}")
    print("  ls:", ", ".join(sorted(f.name for f in out.iterdir())))


if __name__ == "__main__":
    main()
