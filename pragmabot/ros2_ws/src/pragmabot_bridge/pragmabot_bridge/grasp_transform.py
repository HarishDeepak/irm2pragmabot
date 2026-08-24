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


def select_grasp_index(grasps_T_base: np.ndarray, confidences=None,
                       min_confidence: float = 0.0) -> int:
    """Best grasp by GraspGen confidence AND top-down alignment.

    WHY NOT select_topdown_index. That function takes argmin over the
    approach axis alone, so out of 100 candidates spanning conf 0.66-0.95 it
    returns whichever points straightest down - even if it is the WORST
    grasp GraspGen scored. Geometry was being used to overrule the model's
    own judgement about whether the gripper actually fits the object there.

    Top-down still matters (a side approach on a table risks the fingers
    hitting the surface), so it stays in the score rather than being
    dropped: alignment maps straight-down to 1.0, horizontal to 0.5, and
    upward to 0.0, and multiplies the confidence. A confident near-vertical
    grasp beats a marginal perfectly-vertical one.

    `min_confidence` discards candidates outright; if that would empty the
    set the threshold is ignored rather than failing the pick, since a low
    confidence grasp attempted is more informative than no attempt.
    """
    approach_axis = grasps_T_base[:, :3, :3] @ np.array([0.0, 0.0, 1.0])
    alignment = (1.0 - approach_axis[:, 2]) / 2.0

    if confidences is None:
        return int(np.argmax(alignment))

    conf = np.asarray(confidences, dtype=np.float64).reshape(-1)
    if len(conf) != len(grasps_T_base):
        raise ValueError(
            f"{len(conf)} confidences for {len(grasps_T_base)} grasps"
        )

    score = conf * alignment
    if min_confidence > 0.0:
        keep = conf >= min_confidence
        if keep.any():
            score = np.where(keep, score, -1.0)
    return int(np.argmax(score))


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


# ---------------------------------------------------------------------
# Trajectory safety — pure functions, no ROS calls, so they are testable
# offline against a hand-built JointTrajectory.
# ---------------------------------------------------------------------

def retime_trajectory(joint_traj, factor: float):
    """Slow a JointTrajectory down by `factor` (0.1 = ten times slower).

    WHY THIS EXISTS. moveit_msgs/GetCartesianPath in Humble has NO
    max_velocity_scaling_factor / max_acceleration_scaling_factor fields
    (verified against the installed moveit_msgs 2.2.1; see moveit2 issue
    #1967). The service returns a trajectory timed at full speed and there
    is no request field to ask for less. The approach/lift motions - the
    ones heading toward the table - therefore ran unscaled.

    THE MATH. A trajectory is a geometric path q(s) plus a time map
    s = f(t). Substituting t' = t / factor is a pure reparameterisation of
    the SAME path: positions are unchanged, and by the chain rule
    velocities scale by `factor` and accelerations by `factor**2`. Nothing
    is approximated and no waypoint moves, so a path that was collision-
    free stays collision-free. This is exact, not a heuristic.

    Mutates and returns `joint_traj`.
    """
    if factor <= 0.0 or factor > 1.0:
        raise ValueError(f"factor must be in (0, 1], got {factor}")
    if factor == 1.0:
        return joint_traj

    scale = 1.0 / factor
    for pt in joint_traj.points:
        total_ns = (pt.time_from_start.sec * 1_000_000_000
                    + pt.time_from_start.nanosec) * scale
        pt.time_from_start.sec = int(total_ns // 1_000_000_000)
        pt.time_from_start.nanosec = int(total_ns % 1_000_000_000)
        pt.velocities = [v * factor for v in pt.velocities]
        pt.accelerations = [a * factor * factor for a in pt.accelerations]
    return joint_traj


def max_joint_jump(joint_traj) -> float:
    """Largest absolute joint step between consecutive waypoints, in rad.

    The Cartesian service accepts `revolute_jump_threshold` but does not
    forward it to the interpolator (moveit2 issue #2404), so asking for a
    jump guard does not give you one. A near-singular configuration shows
    up as a large joint step between two waypoints that are only 1 cm
    apart in Cartesian space - which is exactly what this measures. Check
    it client-side instead of trusting a parameter that is dropped.
    """
    pts = joint_traj.points
    if len(pts) < 2:
        return 0.0
    worst = 0.0
    for a, b in zip(pts, pts[1:]):
        for qa, qb in zip(a.positions, b.positions):
            worst = max(worst, abs(qb - qa))
    return worst


def joint_limit_margin(names, positions, limits) -> tuple:
    """Smallest distance to any joint limit, and which joint it was.

    `limits` maps joint name -> (lower, upper). FR3 limits are ASYMMETRIC
    and joints 4 and 6 never contain zero, so a symmetric Panda-era
    assumption is wrong on exactly the two joints that dominate a top-down
    grasp. Returns (margin_rad, joint_name); margin is negative if a joint
    is already past its limit.
    """
    worst, who = float("inf"), ""
    for name, pos in zip(names, positions):
        if name not in limits:
            continue
        lo, hi = limits[name]
        m = min(pos - lo, hi - pos)
        if m < worst:
            worst, who = m, name
    return (worst if worst != float("inf") else 0.0), who


def config_distance(a, b) -> float:
    """L-infinity joint-space distance in rad.

    Used to prefer an IK solution near where the arm already is. The FR3
    is 7-DoF against a 6-DoF pose goal, so a null space of solutions
    reaches the identical gripper pose; without this the planner is free
    to pick one that swings the base 124 degrees, which is what it did.
    """
    return max((abs(x - y) for x, y in zip(a, b)), default=0.0)
