"""Grasp-pose math for pick execution.

Loads a camera-frame grasp pose saved by graspgen_client.py's
--save_grasps flag, builds a pre-grasp standoff along GraspGen's own
approach axis, and transforms poses from the ZED optical frame into
fr3_link0 using the live TF tree.

GraspGen's grasp-frame convention (grasp_gen/robot.py,
docs/GRIPPER_DESCRIPTION.md): approach axis is the grasp frame's own
+Z, finger-closing axis is +X, origin is the gripper base/root link
(not the fingertip/TCP).

Frame chain resolved by a single lookup_transform call: fr3_link0 ->
zed_camera_link [easy_handeye2] -> zed_camera_center ->
zed_left_camera_frame -> zed_left_camera_frame_optical [ZED wrapper].
Verify this resolves as expected with
`ros2 run tf2_ros tf2_echo fr3_link0 zed_left_camera_frame_optical`
before trusting any pose it produces.
"""

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, TransformStamped
from scipy.spatial.transform import Rotation


def load_all_grasps(npz_path: str) -> np.ndarray:
    """Load all camera-frame grasp poses saved by graspgen_client.py
    --save_grasps, in the order GraspGen returned them (descending
    confidence). Returns (N, 4, 4).

    graspgen_client.py recenters the point cloud (xyz -= xyz.mean(axis=0))
    before sending it to the server, so the saved grasps are relative to
    that recentered cloud. The subtracted centroid is saved alongside the
    grasps and must be added back here, or every grasp is silently offset
    by the object's own position in the original camera-frame cloud.
    """
    data = np.load(npz_path)
    grasps = data["grasps"].astype(np.float64).copy()
    grasps[:, :3, 3] += data["centroid"]
    return grasps


def load_grasp(npz_path: str, index: int = 0) -> np.ndarray:
    """Load a single camera-frame grasp pose (see load_all_grasps)."""
    return load_all_grasps(npz_path)[index]


def select_topdown_index(grasps_T_base: np.ndarray) -> int:
    """Index of the grasp whose approach axis (GraspGen convention: local
    +Z, pointing from the gripper base toward the object) is most aligned
    with straight down (-Z in `grasps_T_base`'s frame) -- i.e. the most
    top-down approach among the candidates. `grasps_T_base` must already
    be in a gravity-aligned frame (e.g. fr3_link0) -- "top-down" isn't a
    meaningful comparison in camera frame, since the camera's own tilt is
    arbitrary.
    """
    approach_axis = grasps_T_base[:, :3, :3] @ np.array([0.0, 0.0, 1.0])
    return int(np.argmin(approach_axis[:, 2]))


def standoff_pose(grasp_T_cam: np.ndarray, offset_m: float) -> np.ndarray:
    """Pre-grasp pose: same orientation as the grasp, translated back
    `offset_m` along the grasp frame's own +Z (GraspGen's approach axis),
    expressed in the same (camera) frame as the input.
    """
    standoff = grasp_T_cam.copy()
    approach_axis_world = grasp_T_cam[:3, :3] @ np.array([0.0, 0.0, 1.0])
    standoff[:3, 3] = grasp_T_cam[:3, 3] - offset_m * approach_axis_world
    return standoff


def transform_to_matrix(t: TransformStamped) -> np.ndarray:
    """geometry_msgs/TransformStamped -> 4x4 homogeneous matrix."""
    q = t.transform.rotation
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    T[:3, 3] = [
        t.transform.translation.x,
        t.transform.translation.y,
        t.transform.translation.z,
    ]
    return T


def to_robot_frame(
    T_cam: np.ndarray,
    tf_buffer,
    target_frame: str = "fr3_link0",
    source_frame: str = "zed_left_camera_frame_optical",
) -> np.ndarray:
    """Transform a 4x4 pose from `source_frame` into `target_frame`."""
    stamped = tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
    T_target_from_source = transform_to_matrix(stamped)
    return T_target_from_source @ T_cam


def estimate_gripper_width(
    pcd_cam: np.ndarray,
    grasp_T_cam: np.ndarray,
    gripper_depth: float = 0.10527314,
    percentile: float = 5.0,
) -> float:
    """Estimate the object's actual local width where the gripper fingers
    will contact it, directly from the segmented object point cloud --
    NOT a whole-object measurement. A non-uniform object (e.g. a cup) can
    be much narrower/wider at the rim than at the body; the grasp pose
    GraspGen returned determines exactly where along the object the
    fingers land, and a single hand-measured "object diameter" can be
    wrong for whichever spot that actually is.

    `pcd_cam` must be the same (un-centered) camera-frame point cloud
    GraspGen ran on -- e.g. the object_pcd.npy passed as --pcd_file to
    graspgen_client.py, NOT the recentered cloud it sends internally.

    GraspGen's own convention (docs/GRIPPER_DESCRIPTION.md,
    franka_panda.yaml): origin at the gripper base link, approach axis
    +Z, fingertip contact region spans local z in [depth/2, depth]
    (control points). `gripper_depth` default (0.10527314) is
    franka_panda.yaml's own `depth` value for this exact gripper config.

    We transform the cloud into the grasp's local frame, keep points in
    that fingertip z-band, and take a robust (percentile, not min/max)
    spread along local X -- the finger-closing axis -- as the width.
    This relies on the silhouette-width property of a convex object
    viewed near-frontally in a single-view capture (the same single-view
    assumption the rest of this pipeline already makes) -- not guaranteed
    exact, so treat the result as a starting point and keep
    `gripper_epsilon` in execute_pick() generous rather than trusting
    this to the millimeter.
    """
    R = grasp_T_cam[:3, :3]
    t = grasp_T_cam[:3, 3]
    local = (pcd_cam - t) @ R

    z_lo, z_hi = gripper_depth / 2.0, gripper_depth
    band = local[(local[:, 2] >= z_lo) & (local[:, 2] <= z_hi)]
    if len(band) < 10:
        raise ValueError(
            f"Only {len(band)} object points fall in the fingertip contact "
            f"band (local z in [{z_lo:.3f}, {z_hi:.3f}]) -- too few to "
            "estimate width reliably. Check the grasp pose / point cloud "
            "alignment before trusting this."
        )

    lo = np.percentile(band[:, 0], percentile)
    hi = np.percentile(band[:, 0], 100 - percentile)
    return float(hi - lo)


def matrix_to_pose(T: np.ndarray) -> Pose:
    """4x4 homogeneous matrix -> geometry_msgs/Pose."""
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = T[:3, 3]
    qx, qy, qz, qw = Rotation.from_matrix(T[:3, :3]).as_quat()
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose
