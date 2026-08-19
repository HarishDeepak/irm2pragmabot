#!/usr/bin/env python3
"""Capture ONE frame from the live ZED point cloud topic and save it as .npy
for GraspGen's client (`--pcd_file`). Minimal, one-shot — not a pipeline.

Run with system ROS2 python (not the GraspGen .venv):
    ROS_DOMAIN_ID=7 python3 scripts/capture_zed_frame.py -o /tmp/zed_frame.npy
"""
import argparse
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--topic", default="/zed/zed_node/point_cloud/cloud_registered"
    )
    parser.add_argument("-o", "--output", default="/tmp/zed_frame.npy")
    parser.add_argument(
        "--max_range", type=float, default=3.0, help="Drop points farther than this (m) or NaN/Inf"
    )
    args = parser.parse_args()

    rclpy.init()
    node = Node("zed_frame_capture")
    captured = {}

    def cb(msg: PointCloud2):
        pts = point_cloud2.read_points_numpy(
            msg, field_names=("x", "y", "z"), skip_nans=True
        )
        captured["xyz"] = pts.astype(np.float32)
        captured["frame_id"] = msg.header.frame_id

    sub = node.create_subscription(PointCloud2, args.topic, cb, 1)

    print(f"Waiting for one message on {args.topic} ...")
    while rclpy.ok() and "xyz" not in captured:
        rclpy.spin_once(node, timeout_sec=1.0)

    node.destroy_node()
    rclpy.shutdown()

    xyz = captured["xyz"]
    finite = np.isfinite(xyz).all(axis=1)
    xyz = xyz[finite]
    within_range = np.linalg.norm(xyz, axis=1) < args.max_range
    xyz = xyz[within_range]

    if len(xyz) == 0:
        print("ERROR: no valid points captured", file=sys.stderr)
        sys.exit(1)

    np.save(args.output, xyz)
    print(f"frame_id       : {captured['frame_id']}")
    print(f"points saved   : {len(xyz)}")
    print(f"x range (m)    : [{xyz[:,0].min():.3f}, {xyz[:,0].max():.3f}]")
    print(f"y range (m)    : [{xyz[:,1].min():.3f}, {xyz[:,1].max():.3f}]")
    print(f"z range (m)    : [{xyz[:,2].min():.3f}, {xyz[:,2].max():.3f}]")
    print(f"saved to       : {args.output}")


if __name__ == "__main__":
    main()
