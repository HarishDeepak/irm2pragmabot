#!/usr/bin/env python3
"""
extract_bag.py
--------------
Extracts ONE good frame from a ROS 2 bag recorded from a ZED camera and saves it
as three plain files that need no ROS to use:

    rgb.png          the colour image
    depth.npy        depth as a float32 numpy array, in METRES
    intrinsics.json  fx, fy, cx, cy  (needed to build a 3D point cloud)

Run this AFTER you have recorded a bag with `ros2 bag record`.

USAGE
-----
    python3 extract_bag.py \
        --bag      ~/pragmabot_data/bags/scene_single_cup \
        --out      ~/pragmabot_data/extracted/scene_single_cup

    # If your topic names differ, override them:
    python3 extract_bag.py --bag ... --out ... \
        --rgb-topic   /zedxm/zed_node/rgb/image_rect_color/compressed \
        --depth-topic /zedxm/zed_node/depth/depth_registered \
        --info-topic  /zedxm/zed_node/rgb/camera_info

    # To skip early frames (e.g. auto-exposure still settling), take a later one:
    python3 extract_bag.py --bag ... --out ... --skip 10

REQUIREMENTS
------------
    Must be run with your ROS 2 environment sourced, e.g.:
        source /opt/ros/humble/setup.bash
    Python packages: numpy, opencv-python
        pip install numpy opencv-python

TIP: if you do not know your topic names, run:
        ros2 bag info ~/pragmabot_data/bags/scene_single_cup
     and copy the names it prints.
"""

import argparse
import json
import os
import sys

import numpy as np

try:
    import cv2
except ImportError:
    sys.exit("ERROR: opencv-python is not installed.  Run:  pip install opencv-python")

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError:
    sys.exit(
        "ERROR: could not import ROS 2 python packages.\n"
        "Did you source your ROS 2 setup file?  e.g.\n"
        "    source /opt/ros/humble/setup.bash"
    )


# --------------------------------------------------------------------------
# helpers
# --------------------------------------------------------------------------
def detect_storage_id(bag_dir):
    """Look at metadata.yaml to see whether the bag is sqlite3 or mcap."""
    meta = os.path.join(bag_dir, "metadata.yaml")
    if os.path.isfile(meta):
        text = open(meta, "r").read()
        if "mcap" in text:
            return "mcap"
    return "sqlite3"


def open_bag(bag_dir):
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(
        uri=bag_dir, storage_id=detect_storage_id(bag_dir)
    )
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader.open(storage_options, converter_options)
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    return reader, type_map


def decode_rgb(msg, topic_name):
    """Handle both CompressedImage and raw Image messages."""
    if hasattr(msg, "format"):                       # CompressedImage
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if img is None:
            raise RuntimeError(f"Failed to decode compressed image on {topic_name}")
        return img

    # raw sensor_msgs/Image
    enc = msg.encoding.lower()
    if enc in ("rgb8", "bgr8"):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        if enc == "rgb8":
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        return img
    if enc in ("rgba8", "bgra8"):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 4)
        return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR if enc == "bgra8" else cv2.COLOR_RGBA2BGR)
    raise RuntimeError(f"Unsupported colour encoding '{msg.encoding}' on {topic_name}")


def decode_depth(msg):
    """
    Return depth as float32 in METRES.

    ZED normally publishes 32FC1 already in metres.
    Some drivers publish 16UC1 in millimetres -> we convert.
    """
    enc = msg.encoding.lower()
    if enc == "32fc1":
        depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        return depth.astype(np.float32), "32FC1 (already metres)"
    if enc == "16uc1":
        depth_mm = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        return depth_mm.astype(np.float32) / 1000.0, "16UC1 millimetres -> converted to metres"
    raise RuntimeError(f"Unsupported depth encoding '{msg.encoding}'")


# --------------------------------------------------------------------------
# main
# --------------------------------------------------------------------------
def main():
    p = argparse.ArgumentParser(description="Extract one frame from a ROS 2 bag.")
    p.add_argument("--bag", required=True, help="path to the bag FOLDER")
    p.add_argument("--out", required=True, help="output folder to create")
    p.add_argument("--rgb-topic",
                   default="/zedxm/zed_node/rgb/image_rect_color/compressed")
    p.add_argument("--depth-topic",
                   default="/zedxm/zed_node/depth/depth_registered")
    p.add_argument("--info-topic",
                   default="/zedxm/zed_node/rgb/camera_info")
    p.add_argument("--skip", type=int, default=0,
                   help="skip this many frames before grabbing one (default 0)")
    args = p.parse_args()

    bag_dir = os.path.expanduser(args.bag)
    out_dir = os.path.expanduser(args.out)

    if not os.path.isdir(bag_dir):
        sys.exit(f"ERROR: bag folder not found: {bag_dir}")
    os.makedirs(out_dir, exist_ok=True)

    reader, type_map = open_bag(bag_dir)

    # Warn early if the requested topics are not in the bag.
    for t in (args.rgb_topic, args.depth_topic, args.info_topic):
        if t not in type_map:
            print(f"WARNING: topic not found in bag: {t}")
    print("Topics present in this bag:")
    for name, typ in type_map.items():
        print(f"   {name}   ({typ})")
    print()

    rgb_msg = depth_msg = info_msg = None
    rgb_seen = depth_seen = 0

    while reader.has_next():
        topic, data, _stamp = reader.read_next()

        if topic == args.info_topic and info_msg is None:
            info_msg = deserialize_message(data, get_message(type_map[topic]))

        elif topic == args.rgb_topic and rgb_msg is None:
            if rgb_seen >= args.skip:
                rgb_msg = deserialize_message(data, get_message(type_map[topic]))
            rgb_seen += 1

        elif topic == args.depth_topic and depth_msg is None:
            if depth_seen >= args.skip:
                depth_msg = deserialize_message(data, get_message(type_map[topic]))
            depth_seen += 1

        if rgb_msg is not None and depth_msg is not None and info_msg is not None:
            break

    missing = [n for n, m in (("rgb", rgb_msg), ("depth", depth_msg),
                              ("camera_info", info_msg)) if m is None]
    if missing:
        sys.exit(
            "ERROR: could not find these in the bag: " + ", ".join(missing) +
            "\nCheck the topic names above and pass the right ones with "
            "--rgb-topic / --depth-topic / --info-topic"
        )

    # ---- colour -----------------------------------------------------------
    rgb = decode_rgb(rgb_msg, args.rgb_topic)
    rgb_path = os.path.join(out_dir, "rgb.png")
    cv2.imwrite(rgb_path, rgb)

    # ---- depth ------------------------------------------------------------
    depth, depth_note = decode_depth(depth_msg)
    depth_path = os.path.join(out_dir, "depth.npy")
    np.save(depth_path, depth)

    # ---- intrinsics -------------------------------------------------------
    # camera_info.k is the 3x3 matrix laid out as
    #   [ fx  0  cx
    #      0 fy  cy
    #      0  0   1 ]  ->  indices 0, 4, 2, 5
    K = info_msg.k if hasattr(info_msg, "k") else info_msg.K
    intr = {
        "fx": float(K[0]),
        "fy": float(K[4]),
        "cx": float(K[2]),
        "cy": float(K[5]),
        "width": int(info_msg.width),
        "height": int(info_msg.height),
        "frame_id": str(info_msg.header.frame_id),
    }
    intr_path = os.path.join(out_dir, "intrinsics.json")
    with open(intr_path, "w") as f:
        json.dump(intr, f, indent=2)

    # ---- report -----------------------------------------------------------
    valid = np.isfinite(depth) & (depth > 0)
    print("Wrote:")
    print(f"   {rgb_path}      {rgb.shape[1]}x{rgb.shape[0]}")
    print(f"   {depth_path}    {depth.shape[1]}x{depth.shape[0]}   [{depth_note}]")
    print(f"   {intr_path}")
    print()
    print("Sanity check on depth:")
    if valid.any():
        print(f"   valid pixels : {100.0 * valid.mean():.1f} %")
        print(f"   min depth    : {depth[valid].min():.3f} m")
        print(f"   max depth    : {depth[valid].max():.3f} m")
        print(f"   median depth : {np.median(depth[valid]):.3f} m")
        print()
        print("   -> The median should roughly match the real camera-to-table")
        print("      distance. If it looks like 600 instead of 0.6, your depth")
        print("      is in millimetres and needs dividing by 1000.")
    else:
        print("   WARNING: no valid depth pixels found! Check the depth topic.")
    print()
    print("Camera frame id:", intr["frame_id"])
    print("(remember: these 3D points are in the CAMERA frame, not the robot frame)")


if __name__ == "__main__":
    main()
