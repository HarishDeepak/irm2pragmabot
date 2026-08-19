#!/usr/bin/env python3
"""
capture_calib_frame.py — grab ONE live, time-synced RGB+depth+intrinsics
frame from the ZED and save it in the same format extract_bag.py produces
(rgb.png, depth.npy, intrinsics.json), so detect_object.py and
mask_to_pointcloud.py work on it unchanged.

Run with system ROS2 python (NOT any of the tool venvs), and remember the
ZED only appears under domain 7 (see CLAUDE.md):

    ROS_DOMAIN_ID=7 python3 calibration/capture_calib_frame.py \\
        --out extracted/calib_frame_0

If your topic names differ from the /zed/zed_node/... default, override
with --rgb-topic/--depth-topic/--info-topic (see extract_bag.py for the
/zedxm/... variant seen before).
"""

import argparse
import json
import os
import sys

import numpy as np

try:
    import cv2
except ImportError:
    sys.exit("ERROR: opencv-python not installed. pip install opencv-python")

import rclpy
from rclpy.node import Node
import message_filters
from sensor_msgs.msg import Image, CameraInfo


def decode_rgb(msg: Image) -> np.ndarray:
    enc = msg.encoding.lower()
    if enc in ("rgb8", "bgr8"):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if enc == "rgb8" else img
    if enc in ("rgba8", "bgra8"):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 4)
        return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR if enc == "bgra8" else cv2.COLOR_RGBA2BGR)
    raise RuntimeError(f"Unsupported colour encoding '{msg.encoding}'")


def decode_depth(msg: Image) -> np.ndarray:
    """Return depth as float32 in METRES. ZED normally publishes 32FC1 already
    in metres; 16UC1 (millimetres) is converted if seen."""
    enc = msg.encoding.lower()
    if enc == "32fc1":
        return np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width).copy()
    if enc == "16uc1":
        mm = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        return (mm.astype(np.float32)) / 1000.0
    raise RuntimeError(f"Unsupported depth encoding '{msg.encoding}'")


class SyncedCapture(Node):
    def __init__(self, rgb_topic, depth_topic, info_topic):
        super().__init__("calib_frame_capture")
        self.result = None

        rgb_sub = message_filters.Subscriber(self, Image, rgb_topic)
        depth_sub = message_filters.Subscriber(self, Image, depth_topic)
        info_sub = message_filters.Subscriber(self, CameraInfo, info_topic)

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub, info_sub], queue_size=10, slop=0.05
        )
        self.ts.registerCallback(self._cb)

    def _cb(self, rgb_msg, depth_msg, info_msg):
        if self.result is not None:
            return
        self.result = {
            "rgb": decode_rgb(rgb_msg),
            "depth": decode_depth(depth_msg),
            "K": info_msg.k,  # row-major 3x3
            "width": info_msg.width,
            "height": info_msg.height,
            "frame_id": rgb_msg.header.frame_id,
        }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", required=True,
                         help="output directory, e.g. extracted/calib_frame_0")
    parser.add_argument("--rgb-topic", default="/zed/zed_node/rgb/color/rect/image")
    parser.add_argument("--depth-topic", default="/zed/zed_node/depth/depth_registered")
    parser.add_argument("--info-topic", default="/zed/zed_node/rgb/color/rect/camera_info")
    parser.add_argument("--timeout", type=float, default=15.0)
    args = parser.parse_args()

    os.makedirs(args.out, exist_ok=True)

    rclpy.init()
    node = SyncedCapture(args.rgb_topic, args.depth_topic, args.info_topic)

    print(f"Waiting for a synced RGB+depth+info frame on:\n"
          f"  {args.rgb_topic}\n  {args.depth_topic}\n  {args.info_topic}\n"
          f"(timeout {args.timeout}s — if this hangs, check ROS_DOMAIN_ID=7 "
          f"and that the ZED node is actually running)")

    import time
    start = time.time()
    while rclpy.ok() and node.result is None and (time.time() - start) < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.2)

    if node.result is None:
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(f"ERROR: no synced frame received within {args.timeout}s")

    result = node.result
    node.destroy_node()
    rclpy.shutdown()

    rgb_path = os.path.join(args.out, "rgb.png")
    depth_path = os.path.join(args.out, "depth.npy")
    intrinsics_path = os.path.join(args.out, "intrinsics.json")

    cv2.imwrite(rgb_path, result["rgb"])
    np.save(depth_path, result["depth"])

    K = result["K"]
    with open(intrinsics_path, "w") as f:
        json.dump({
            "fx": K[0], "fy": K[4], "cx": K[2], "cy": K[5],
            "width": result["width"], "height": result["height"],
            "frame_id": result["frame_id"],
        }, f, indent=2)

    print(f"Saved:\n  {rgb_path}\n  {depth_path}\n  {intrinsics_path}")
    print(f"depth range (m): [{np.nanmin(result['depth']):.3f}, "
          f"{np.nanmax(result['depth']):.3f}]")


if __name__ == "__main__":
    main()
