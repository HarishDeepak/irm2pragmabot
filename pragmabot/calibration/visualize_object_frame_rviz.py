#!/usr/bin/env python3
"""Standalone rviz2 test for the object coordinate frame (see
pragmabot_bridge.grasp_transform.object_frame_T_base /
object_frame_markers), with NO bridge_node or pick attempt required.

WHY THIS EXISTS. bridge_node's grasp_object_markers topic only appears
once pragmabot_bridge is running AND an execute_pick() has actually
reached the object-frame computation (needs the gripper homed, an
object_pcd_file, etc) - too much machinery in the way just to check "does
the frame arrows/marker rendering look right". This script publishes the
exact same MarkerArray (grasp_transform.object_frame_markers, the same
function bridge_node calls) and a PointCloud2 of the object cloud, from a
bare rclpy node with a timer - so you can see it in rviz first, confirm
it looks right, and only then wire it into a real pick.

USAGE
    ~/GraspGen/.venv/bin/python3 calibration/visualize_object_frame_rviz.py \\
        [--pcd_file PATH] [--frame_id world] \\
        [--transform_to_base] [--camera_frame zed_left_camera_frame_optical] \\
        [--base_frame fr3_link0] [--axis_len 0.08]

TWO MODES.

1. Default (no --transform_to_base): publishes the cloud/frame exactly as
   loaded, in whatever frame the capture was made in (camera frame for
   pragmabot_live_cloud.npy), under the label --frame_id (default
   "world"). Needs NO TF tree at all - set rviz's Fixed Frame to the same
   string. This only tests that the marker/cloud RENDERING and the
   object-frame MATH look right; the cloud will NOT appear at the robot's
   real object location, since no camera->base transform is applied.

2. --transform_to_base: looks up the live TF chain (fr3_link0 <-
   --camera_frame, e.g. via easy_handeye2 + the ZED wrapper - the SAME
   lookup bridge_node.execute_pick() does) and transforms the cloud and
   object frame into --base_frame before publishing. Requires that TF
   chain to actually be up (echo `ros2 run tf2_ros tf2_echo fr3_link0
   zed_left_camera_frame_optical` first if unsure). Publishes under
   --base_frame regardless of --frame_id, since that is what the pose is
   now expressed in - set rviz's Fixed Frame to --base_frame (e.g.
   fr3_link0) to see it positioned realistically, next to the real robot
   model / planning scene if you already have those displays up.

Defaults to /tmp/pragmabot_live_cloud.npy (the real object cloud saved by
bridge_node's last pick attempt, if one exists) so the shape and axes you
see are from actual captured data, not a toy. Falls back to a synthetic
elongated cloud (a banana-shaped stand-in) if that file is missing, so
this always runs even with no prior pick on this machine.

TROUBLESHOOTING "no error but I don't see anything": the axis arrows are
only --axis_len (default 8cm) long, tiny next to a robot-scale rviz view.
Zoom/pan close to the object's logged origin (printed on startup, and in
the node's log line) before assuming nothing is being published; also
check the MarkerArray display's "Namespaces" list in the Displays panel
has "grasp_object_axes" ticked - rviz sometimes leaves a namespace
unticked when a display first recovers from an error state.
"""

import argparse

import numpy as np
import rclpy
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import MarkerArray

from pragmabot_bridge import grasp_transform


def _synthetic_object_cloud(n: int = 400) -> np.ndarray:
    """Elongated (banana-like) stand-in cloud, used only when no real
    capture is available. Long along X, mildly curved in Z, so the
    principal axis / object-frame math has something non-trivial to do."""
    t = np.linspace(-0.09, 0.09, n)
    x = t
    y = 0.015 * np.random.randn(n)
    z = 0.03 + 0.02 * (1 - (t / 0.09) ** 2) + 0.004 * np.random.randn(n)
    return np.stack([x, y, z], axis=1)


class ObjectFrameRvizTest(Node):
    def __init__(self, pcd: np.ndarray, frame_id: str, axis_len: float,
                transform_to_base: bool, camera_frame: str, base_frame: str):
        super().__init__("object_frame_rviz_test")
        self._axis_len = axis_len
        self._marker_pub = self.create_publisher(MarkerArray, "grasp_object_markers", 10)
        self._cloud_pub = self.create_publisher(PointCloud2, "object_cloud", 10)

        if transform_to_base:
            # SAME lookup bridge_node.execute_pick() does before ranking
            # grasps - see grasp_transform module docstring for the frame
            # chain this resolves (easy_handeye2 + ZED wrapper).
            tf_buffer = Buffer()
            TransformListener(tf_buffer, self)
            self.get_logger().info(
                f"Waiting for TF {base_frame} <- {camera_frame} ...")
            start = self.get_clock().now()
            while not tf_buffer.can_transform(
                    base_frame, camera_frame, rclpy.time.Time()):
                rclpy.spin_once(self, timeout_sec=0.1)
                if (self.get_clock().now() - start).nanoseconds > 10e9:
                    raise RuntimeError(
                        f"TF {base_frame} <- {camera_frame} not available after "
                        "10s - is easy_handeye2 publish.launch.py running, and "
                        "is the ZED wrapper's robot_state_publisher up? Verify "
                        f"with: ros2 run tf2_ros tf2_echo {base_frame} {camera_frame}"
                    )
            stamped = tf_buffer.lookup_transform(
                base_frame, camera_frame, rclpy.time.Time())
            T_base_from_cam = grasp_transform.transform_to_matrix(stamped)
            pcd = (T_base_from_cam[:3, :3] @ pcd.T).T + T_base_from_cam[:3, 3]
            self._frame_id = base_frame
            self.get_logger().info(
                f"Transformed cloud into {base_frame} using the live TF chain")
        else:
            self._frame_id = frame_id

        self._pcd = pcd
        axis, elong = grasp_transform.principal_axis_xy(pcd)
        self._object_T = grasp_transform.object_frame_T_base(pcd, long_axis=axis)
        self.get_logger().info(
            f"Loaded {len(pcd)} points, footprint elongation {elong:.2f}:1, "
            f"object frame origin {np.round(self._object_T[:3, 3], 3)} "
            f"(frame: {self._frame_id}) - point rviz's Fixed Frame and "
            "camera view there if you see no error but nothing rendered"
        )

        self.create_timer(0.5, self._publish)

    def _publish(self):
        stamp = self.get_clock().now().to_msg()

        markers = grasp_transform.object_frame_markers(
            self._object_T, self._frame_id, stamp=stamp, axis_len=self._axis_len)
        self._marker_pub.publish(markers)

        header = Header(stamp=stamp, frame_id=self._frame_id)
        cloud_msg = pc2.create_cloud_xyz32(header, self._pcd.astype(np.float32).tolist())
        self._cloud_pub.publish(cloud_msg)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--pcd_file", default="/tmp/pragmabot_live_cloud.npy")
    parser.add_argument("--frame_id", default="world",
                        help="Frame label when NOT using --transform_to_base")
    parser.add_argument("--transform_to_base", action="store_true",
                        help="Look up the live TF chain and transform the cloud "
                             "into --base_frame, same as bridge_node does for real")
    parser.add_argument("--camera_frame", default="zed_left_camera_frame_optical")
    parser.add_argument("--base_frame", default="fr3_link0")
    parser.add_argument("--axis_len", type=float, default=0.08)
    args = parser.parse_args()

    try:
        pcd = np.load(args.pcd_file).astype(np.float64)[:, :3]
        print(f"Loaded real object cloud from {args.pcd_file} ({len(pcd)} points)")
    except (FileNotFoundError, OSError):
        pcd = _synthetic_object_cloud()
        print(
            f"{args.pcd_file} not found - using a synthetic banana-shaped "
            f"cloud instead ({len(pcd)} points)"
        )

    rclpy.init()
    node = ObjectFrameRvizTest(
        pcd, args.frame_id, args.axis_len,
        args.transform_to_base, args.camera_frame, args.base_frame)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
