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

FoundationPose runs in the `foundationpose_calib` Docker container (CUDA
11.8 image — verified 2026-07-25 the RTX 4080 needs 11.8+ for compute_89;
the two other pulled images are stuck on CUDA 11.3 and will not compile),
never natively on host. `run_foundationpose_register()` below talks to it
purely over files — writes rgb/depth/K/mask into the container's mounted
calib_io/ dir, runs register_once.py via `docker exec`, reads the pose
back. No ROS2/DDS ever crosses the container boundary (that's the exact
bug class that cost a full day before — see CLAUDE.md).
"""

import argparse
import json
import subprocess
import tempfile
import time
from pathlib import Path

import cv2
import message_filters
import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Imu, Image, CameraInfo
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

FP_CONTAINER = "foundationpose_calib"
FP_CALIB_IO_HOST = Path.home() / "foundationpose" / "FoundationPose" / "calib_io"
FP_CALIB_IO_CONTAINER = "/workspace/FoundationPose/calib_io"
FP_PYTHON = "/opt/conda/envs/my/bin/python"
GROUNDEDSAM_PYTHON = Path.home() / "groundedsam" / ".venv" / "bin" / "python"
DETECT_OBJECT_SCRIPT = Path(__file__).parent / "detect_object.py"


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
                                 mask: np.ndarray,
                                 frame_tag: str = "frame") -> np.ndarray:
    """
    Runs FoundationPose.register() inside the foundationpose_calib
    container via file exchange (see module docstring). Requires:
      - `docker start foundationpose_calib` already done (container is a
        named, persistent instance — see CLAUDE.md — not `--rm`)
      - mesh_path readable from the HOST at the same relative path the
        container sees it at (simplest: put meshes under
        ~/foundationpose/FoundationPose/calib_io/ directly)

    `mask` is the cube's segmentation mask — from detect_object.py
    (GroundedSAM) prompted with the cube's description.
    """
    FP_CALIB_IO_HOST.mkdir(parents=True, exist_ok=True)

    mesh_host_path = Path(mesh_path).resolve()
    try:
        mesh_rel = mesh_host_path.relative_to(FP_CALIB_IO_HOST.resolve())
    except ValueError:
        raise RuntimeError(
            f"--mesh must live under {FP_CALIB_IO_HOST} (the container's "
            f"mounted calib_io/ dir) so it's visible inside the container "
            f"at the equivalent path — got {mesh_host_path}")
    mesh_container_path = f"{FP_CALIB_IO_CONTAINER}/{mesh_rel.as_posix()}"

    rgb_path = FP_CALIB_IO_HOST / f"{frame_tag}_rgb.png"
    depth_path = FP_CALIB_IO_HOST / f"{frame_tag}_depth.npy"
    mask_path = FP_CALIB_IO_HOST / f"{frame_tag}_mask.npy"
    intrinsics_path = FP_CALIB_IO_HOST / f"{frame_tag}_intrinsics.json"
    pose_path = FP_CALIB_IO_HOST / f"{frame_tag}_pose.txt"

    cv2.imwrite(str(rgb_path), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    np.save(depth_path, depth.astype(np.float32))
    np.save(mask_path, mask.astype(bool))
    with open(intrinsics_path, "w") as f:
        json.dump({"fx": float(K[0, 0]), "fy": float(K[1, 1]),
                   "cx": float(K[0, 2]), "cy": float(K[1, 2])}, f)

    container_prefix = f"{FP_CALIB_IO_CONTAINER}/{frame_tag}"
    cmd = [
        "docker", "exec", "-e", "PYTHONPATH=/workspace/FoundationPose",
        FP_CONTAINER, FP_PYTHON,
        f"{FP_CALIB_IO_CONTAINER}/register_once.py",
        "--mesh", mesh_container_path,
        "--rgb", f"{container_prefix}_rgb.png",
        "--depth", f"{container_prefix}_depth.npy",
        "--intrinsics", f"{container_prefix}_intrinsics.json",
        "--mask", f"{container_prefix}_mask.npy",
        "--out", f"{container_prefix}_pose.txt",
    ]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError(
            f"register_once.py failed inside container:\n{result.stderr}"
        )
    print(result.stdout)

    return np.loadtxt(pose_path)


# ---------------------------------------------------------------------------
# Live frame capture — same decode logic as capture_calib_frame.py, inlined
# so it can share the one rclpy context used for the whole script (IMU
# check + TF publish already needed rclpy.init() at the top-level anyway).
# ---------------------------------------------------------------------------

def _decode_rgb(msg: Image) -> np.ndarray:
    enc = msg.encoding.lower()
    if enc in ("rgb8", "bgr8"):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if enc == "rgb8" else img
    if enc in ("rgba8", "bgra8"):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 4)
        return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR if enc == "bgra8" else cv2.COLOR_RGBA2BGR)
    raise RuntimeError(f"Unsupported colour encoding '{msg.encoding}'")


def _decode_depth(msg: Image) -> np.ndarray:
    enc = msg.encoding.lower()
    if enc == "32fc1":
        return np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width).copy()
    if enc == "16uc1":
        mm = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        return mm.astype(np.float32) / 1000.0
    raise RuntimeError(f"Unsupported depth encoding '{msg.encoding}'")


class SyncedFrameCapture(Node):
    """One-shot RGB+depth+intrinsics grab per call to capture_one(), reused
    across --num-frames calls in main()'s loop."""

    def __init__(self, camera_topic_ns: str):
        super().__init__("calib_frame_capture")
        rgb_sub = message_filters.Subscriber(
            self, Image, f"{camera_topic_ns}/rgb/color/rect/image")
        depth_sub = message_filters.Subscriber(
            self, Image, f"{camera_topic_ns}/depth/depth_registered")
        info_sub = message_filters.Subscriber(
            self, CameraInfo, f"{camera_topic_ns}/rgb/color/rect/camera_info")
        self._ts = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub, info_sub], queue_size=10, slop=0.05)
        self._ts.registerCallback(self._cb)
        self._pending = None

    def _cb(self, rgb_msg, depth_msg, info_msg):
        self._pending = (_decode_rgb(rgb_msg), _decode_depth(depth_msg),
                          np.array(info_msg.k).reshape(3, 3))

    def capture_one(self, timeout_sec=15.0):
        self._pending = None
        start = time.time()
        while self._pending is None and (time.time() - start) < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.2)
        if self._pending is None:
            raise RuntimeError(f"No synced frame within {timeout_sec}s — "
                                f"check ROS_DOMAIN_ID and that the ZED node is up.")
        return self._pending


def detect_cube_mask(rgb: np.ndarray, prompt: str, frame_tag: str) -> np.ndarray:
    """Shells out to GroundedSAM's own venv (can't import cross-venv) — same
    detect_object.py already verified working standalone."""
    tmp_dir = Path(tempfile.gettempdir()) / "calib_detect" / frame_tag
    tmp_dir.mkdir(parents=True, exist_ok=True)
    rgb_path = tmp_dir / "rgb.png"
    cv2.imwrite(str(rgb_path), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))

    cmd = [str(GROUNDEDSAM_PYTHON), str(DETECT_OBJECT_SCRIPT),
           "--rgb", str(rgb_path), "--prompt", prompt,
           "--out-dir", str(tmp_dir / "detections")]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError(f"detect_object.py failed:\n{result.stderr}")

    return np.load(tmp_dir / "detections" / "mask.npy")


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
    parser.add_argument("--cube-xyz-in-base", nargs=3, type=float, default=None,
                         metavar=("X", "Y", "Z"),
                         help="Hand-measured cube center in fr3_link0, meters. "
                              "If omitted, only runs the vision capture/register "
                              "step and saves T_cube_in_camera to --out-cube-pose "
                              "for later composition once you have a real "
                              "measurement — does not publish any TF.")
    parser.add_argument("--out-cube-pose", default="T_cube_in_camera.txt",
                         help="Where to save the averaged T_cube_in_camera 4x4 "
                              "when --cube-xyz-in-base is omitted.")
    parser.add_argument("--cube-rpy-in-base", nargs=3, type=float, default=[0, 0, 0],
                         metavar=("R", "P", "Y"))
    parser.add_argument("--num-frames", type=int, default=8)
    parser.add_argument("--camera-topic-ns", default="/zed/zed_node")
    parser.add_argument("--parent-frame", default="fr3_link0")
    parser.add_argument("--child-frame", default="zed_left_camera_frame_optical")
    parser.add_argument("--cube-prompt", default="green cube.",
                         help="GroundedSAM text prompt for the calibration cube")
    args = parser.parse_args()

    have_base_measurement = args.cube_xyz_in_base is not None
    if have_base_measurement:
        T_cube_in_base = pose_from_xyz_rpy(args.cube_xyz_in_base, args.cube_rpy_in_base)

    rclpy.init()

    print(f"Capturing {args.num_frames} independent register() frames — "
          f"keep the cube stationary throughout.")
    capture_node = SyncedFrameCapture(args.camera_topic_ns)
    poses_cube_in_camera = []
    for i in range(args.num_frames):
        print(f"  frame {i+1}/{args.num_frames} ...")
        rgb, depth, K = capture_node.capture_one()
        mask = detect_cube_mask(rgb, args.cube_prompt, frame_tag=f"calib_{i}")
        pose = run_foundationpose_register(args.mesh, rgb, depth, K, mask,
                                            frame_tag=f"calib_{i}")
        poses_cube_in_camera.append(pose)
        print(f"    pose translation: {pose[:3, 3]}")
    capture_node.destroy_node()

    T_cube_in_camera = average_poses(poses_cube_in_camera)

    if not have_base_measurement:
        np.savetxt(args.out_cube_pose, T_cube_in_camera)
        print(f"\nNo --cube-xyz-in-base given — saved averaged T_cube_in_camera "
              f"to {args.out_cube_pose}. Re-run once you have the real "
              f"caliper measurement of the cube's position in fr3_link0 to "
              f"compose and publish the final TF.")
        rclpy.shutdown()
        return

    T_cam_in_base = T_cube_in_base @ np.linalg.inv(T_cube_in_camera)

    # --- IMU sanity check: roll/pitch only ---
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

    print("\nFinal T_cam_in_base:")
    print(T_cam_in_base)
    print("\nStatic TF publisher spinning — leave this process running so "
          "/tf_static reaches subscribers that connect later (RViz, "
          "tf2_echo, ...). Ctrl-C to stop.")
    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == "__main__":
    main()
