"""
calibrate_extrinsic.py — ZED2 -> fr3_link0 static extrinsic calibration
using a known marked cube and FoundationPose.

Method (see CLAUDE.md for the verified facts behind these choices):
  T_cam_in_base = T_cube_in_base @ inv(T_cube_in_camera)

T_cube_in_base comes from hand measurement (you supply it via CLI args,
having measured the cube's placement with calipers relative to fr3_link0).
T_cube_in_camera comes from averaging several independent FoundationPose
register() calls while the cube sits stationary.

USAGE (run inside Container 3, once FoundationPose + its cube mesh from
mesh_gen.py are ready):

    python3 calibrate_extrinsic.py \\
        --mesh cube.obj \\
        --cube-xyz-in-base 0.45 0.10 0.03 \\
        --cube-rpy-in-base 0 0 0 \\
        --num-frames 8 \\
        --camera-topic-ns /zed/zed_node

--cube-xyz-in-base / --cube-rpy-in-base: your hand-measured ground truth,
in meters / radians, cube center relative to fr3_link0. If you placed the
cube with faces aligned to the base axes (recommended — makes measurement
and error-checking much easier), rpy is likely close to [0,0,0] or a
multiple of pi/2.

FLAGGED, NOT VERIFIED: the exact FoundationPose class/method signature
below (see `run_foundationpose_register`) is written generically based on
the demo script's register()/track() naming convention, but the precise
API (constructor args, K-matrix format, mask requirements) was not
confirmed against the actual repo in this conversation. Check
`repo_reference` or FoundationPose's own run_demo.py once you have it
cloned tomorrow, and adjust that one function accordingly — everything
else in this script (averaging, extrinsic composition, IMU check, TF
publish) is not dependent on that API and is safe to trust as-is.
"""

import argparse

import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Imu
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped


# ---------------------------------------------------------------------------
# Pose averaging
# ---------------------------------------------------------------------------

def average_poses(poses_4x4: list[np.ndarray]) -> np.ndarray:
    """
    Average N independent 4x4 pose estimates. Translation: arithmetic mean.
    Rotation: chordal L2 mean via scipy (handles rotation averaging
    correctly, unlike naively averaging rotation matrices or Euler angles).
    Outliers beyond 2 sigma from the mean (by rotation angle distance) are
    dropped before the final average.
    """
    translations = np.array([T[:3, 3] for T in poses_4x4])
    rotations = R.from_matrix([T[:3, :3] for T in poses_4x4])

    mean_rot_initial = rotations.mean()
    angle_dists = np.array([
        (mean_rot_initial.inv() * r).magnitude() for r in rotations
    ])
    keep = angle_dists < (angle_dists.mean() + 2 * angle_dists.std() + 1e-9)
    if keep.sum() < len(poses_4x4):
        print(f"Dropped {len(poses_4x4) - keep.sum()} outlier frame(s) "
              f"beyond 2-sigma rotation distance.")

    t_mean = translations[keep].mean(axis=0)
    r_mean = R.from_matrix([T[:3, :3] for T, k in zip(poses_4x4, keep) if k]).mean()

    T_mean = np.eye(4)
    T_mean[:3, :3] = r_mean.as_matrix()
    T_mean[:3, 3] = t_mean
    return T_mean


def pose_from_xyz_rpy(xyz, rpy) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R.from_euler("xyz", rpy).as_matrix()
    T[:3, 3] = xyz
    return T


# ---------------------------------------------------------------------------
# FoundationPose adapter — SEE MODULE DOCSTRING, NOT VERIFIED AGAINST REPO
# ---------------------------------------------------------------------------

def run_foundationpose_register(mesh_path: str, rgb: np.ndarray,
                                 depth: np.ndarray, K: np.ndarray,
                                 mask: np.ndarray) -> np.ndarray:
    """
    TODO: confirm against the actual FoundationPose repo tomorrow.
    Expected shape, based on run_demo.py's general pattern:

        from estimater import FoundationPose  # or similar — VERIFY
        est = FoundationPose(mesh_path=mesh_path, ...)
        pose = est.register(K=K, rgb=rgb, depth=depth, ob_mask=mask)
        return pose  # 4x4 np.ndarray, object-in-camera-frame

    `mask` is the cube's segmentation mask in the RGB frame — get this
    from GroundedSAM (already built as part of Container 3) prompted with
    the cube's color/description, or a simple manual ROI for calibration
    purposes since the scene is controlled and static.
    """
    raise NotImplementedError(
        "Fill in against the real FoundationPose API once confirmed — "
        "see this function's docstring and the module-level warning."
    )


# ---------------------------------------------------------------------------
# IMU sanity check — roll/pitch only, per CLAUDE.md
# ---------------------------------------------------------------------------

def roll_pitch_from_matrix(T: np.ndarray) -> tuple[float, float]:
    roll, pitch, _yaw = R.from_matrix(T[:3, :3]).as_euler("xyz")
    return roll, pitch


class ImuSanityCheckNode(Node):
    """Grabs one IMU orientation reading for a roll/pitch cross-check.
    Does NOT use IMU for translation or yaw — see CLAUDE.md."""

    def __init__(self, topic: str):
        super().__init__("calib_imu_check")
        self.orientation = None
        self._sub = self.create_subscription(Imu, topic, self._cb, 10)

    def _cb(self, msg: Imu):
        self.orientation = msg.orientation
        self.destroy_subscription(self._sub)

    def wait_for_reading(self, timeout_sec=5.0):
        rclpy.spin_until_future_complete(
            self, rclpy.task.Future(), timeout_sec=timeout_sec
        ) if False else None  # placeholder pattern; see note below
        # Simple spin loop — replace with a proper future/executor pattern
        # if integrating into a larger node.
        import time
        start = time.time()
        while self.orientation is None and (time.time() - start) < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.orientation


def imu_roll_pitch(orientation_msg) -> tuple[float, float]:
    q = [orientation_msg.x, orientation_msg.y, orientation_msg.z, orientation_msg.w]
    roll, pitch, _yaw = R.from_quat(q).as_euler("xyz")
    return roll, pitch


# ---------------------------------------------------------------------------
# TF publishing
# ---------------------------------------------------------------------------

class ExtrinsicPublisher(Node):
    def __init__(self, T_cam_in_base: np.ndarray, parent_frame: str, child_frame: str):
        super().__init__("calib_extrinsic_publisher")
        self._broadcaster = StaticTransformBroadcaster(self)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame

        t.transform.translation.x = float(T_cam_in_base[0, 3])
        t.transform.translation.y = float(T_cam_in_base[1, 3])
        t.transform.translation.z = float(T_cam_in_base[2, 3])

        quat = R.from_matrix(T_cam_in_base[:3, :3]).as_quat()  # x,y,z,w
        t.transform.rotation.x = float(quat[0])
        t.transform.rotation.y = float(quat[1])
        t.transform.rotation.z = float(quat[2])
        t.transform.rotation.w = float(quat[3])

        self._broadcaster.sendTransform(t)
        self.get_logger().info(
            f"Published static transform {parent_frame} -> {child_frame}"
        )


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mesh", required=True)
    parser.add_argument("--cube-xyz-in-base", nargs=3, type=float, required=True,
                         metavar=("X", "Y", "Z"))
    parser.add_argument("--cube-rpy-in-base", nargs=3, type=float, default=[0, 0, 0],
                         metavar=("R", "P", "Y"))
    parser.add_argument("--num-frames", type=int, default=8)
    parser.add_argument("--camera-topic-ns", default="/zed/zed_node")
    parser.add_argument("--parent-frame", default="fr3_link0")
    parser.add_argument("--child-frame", default="zed_left_camera_frame_optical")
    args = parser.parse_args()

    T_cube_in_base = pose_from_xyz_rpy(args.cube_xyz_in_base, args.cube_rpy_in_base)

    print(f"Capturing {args.num_frames} independent register() frames — "
          f"keep the cube stationary throughout.")
    # TODO: wire this loop to actual RGB/depth/K capture from the ZED topics
    # (message_filters.ApproximateTimeSynchronizer, same pattern as the
    # existing scene_observer.py) and a real segmentation mask from
    # GroundedSAM. Left as a loop stub since it depends on the
    # FoundationPose adapter above being filled in first.
    poses_cube_in_camera = []
    for i in range(args.num_frames):
        # rgb, depth, K, mask = capture_synced_frame(args.camera_topic_ns)
        # pose = run_foundationpose_register(args.mesh, rgb, depth, K, mask)
        # poses_cube_in_camera.append(pose)
        raise NotImplementedError(
            "Wire up frame capture + run_foundationpose_register() once "
            "FoundationPose is confirmed working standalone tomorrow."
        )

    T_cube_in_camera = average_poses(poses_cube_in_camera)
    T_cam_in_base = T_cube_in_base @ np.linalg.inv(T_cube_in_camera)

    # --- IMU sanity check: roll/pitch only ---
    rclpy.init()
    imu_node = ImuSanityCheckNode(f"{args.camera_topic_ns}/imu/data")
    orientation = imu_node.wait_for_reading()
    if orientation is not None:
        imu_roll, imu_pitch = imu_roll_pitch(orientation)
        calib_roll, calib_pitch = roll_pitch_from_matrix(T_cam_in_base)
        print(f"IMU roll/pitch:   {np.degrees(imu_roll):.2f}, {np.degrees(imu_pitch):.2f} deg")
        print(f"Calib roll/pitch: {np.degrees(calib_roll):.2f}, {np.degrees(calib_pitch):.2f} deg")
        print("(Large disagreement here suggests a problem with the "
              "calibration OR the base isn't actually mounted level — "
              "investigate before trusting the result. Yaw is NOT checked, "
              "IMU can't observe it.)")
    else:
        print("WARNING: no IMU reading received — skipping sanity check.")
    imu_node.destroy_node()

    publisher = ExtrinsicPublisher(T_cam_in_base, args.parent_frame, args.child_frame)
    rclpy.spin_once(publisher, timeout_sec=1.0)

    print("\nFinal T_cam_in_base:")
    print(T_cam_in_base)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
