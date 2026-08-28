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
                       min_confidence: float = 0.0,
                       max_tilt_deg: float = 0.0) -> int:
    """Best-confidence grasp among the candidates that are near-perpendicular
    to the table. Returns -1 if none qualify - see below, this is a hard
    filter, not a fallback-able preference.

    WHY A HARD TILT GATE, NOT A SOFT SCORE FACTOR (as this used to be).
    The previous version multiplied confidence by a continuous top-down
    alignment factor (1.0=vertical ... 0.0=upward), so a confident but
    badly tilted grasp could still outscore a well-aligned, merely-decent
    one - the two traded off freely, and nothing stopped a near-horizontal
    approach from winning just because GraspGen rated it highly. On this
    table-mounted setup a steep tilt also means the fingers are more
    likely to clip the table or a neighbouring object before ever reaching
    the target, which a multiplicative score does not represent - it is a
    hazard, not a mild preference.

    `max_tilt_deg` (degrees off straight-down; 0 disables the gate) REMOVES
    any candidate outside that cone before ranking survivors by confidence:
    "vertical enough AND confident", not "vertical enough to make up for
    low confidence, or confident enough to make up for being sideways".

    WHY NOT select_topdown_index. That function takes argmin over tilt
    alone with no confidence involved, so out of 100 candidates spanning
    conf 0.66-0.95 it returns whichever points straightest down - even if
    it is the worst grasp GraspGen scored.

    THE TILT GATE FAILS CLOSED, NOT OPEN: if no candidate is within
    `max_tilt_deg`, this returns -1 rather than silently picking the
    least-tilted (but still unsafe) option - a near-horizontal approach
    attempted anyway is exactly the risk this gate exists to remove.
    Callers must check for -1 and abort the pick with a reason (e.g. "no
    near-perpendicular grasp available - try reorienting the object"),
    not fall through with a negative index. `min_confidence` keeps the
    older best-effort behaviour (ignored rather than emptying the tilt
    survivors) since a merely low-confidence grasp is a judgement call,
    not a collision risk.
    """
    approach_axis = grasps_T_base[:, :3, :3] @ np.array([0.0, 0.0, 1.0])
    # Angle between the approach axis and straight down (base frame -Z).
    # 0 deg = perpendicular to the table, 90 deg = horizontal.
    tilt_deg = np.degrees(np.arccos(np.clip(-approach_axis[:, 2], -1.0, 1.0)))

    if max_tilt_deg > 0.0:
        tilt_keep = tilt_deg <= max_tilt_deg
        if not tilt_keep.any():
            return -1
    else:
        tilt_keep = np.ones(len(grasps_T_base), dtype=bool)

    if confidences is None:
        candidates = np.where(tilt_keep)[0]
        return int(candidates[np.argmin(tilt_deg[candidates])])

    conf = np.asarray(confidences, dtype=np.float64).reshape(-1)
    if len(conf) != len(grasps_T_base):
        raise ValueError(
            f"{len(conf)} confidences for {len(grasps_T_base)} grasps"
        )

    keep = tilt_keep.copy()
    if min_confidence > 0.0:
        conf_keep = conf >= min_confidence
        if (keep & conf_keep).any():
            keep &= conf_keep

    score = np.where(keep, conf, -1.0)
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


def center_grasp_on_object(grasp_T_cam: np.ndarray, pcd_cam: np.ndarray,
                           max_shift: float = 0.08,
                           gripper_depth: float = 0.10527314) -> np.ndarray:
    """Slide a grasp along its finger-closing axis so the fingers straddle
    the object symmetrically. Returns a corrected copy of `grasp_T_cam`.

    WHY (measured 2026-08-26, on a real 4.5 cm cube). A single-view depth
    capture sees only the faces pointing at the camera, so the object
    cloud is a SHELL, not a solid. Its centroid therefore sits toward the
    camera by roughly half the unseen depth -- here +8.7 mm. GraspGen
    recentres the cloud on that centroid before inference, so every grasp
    it returns inherits the same bias.

    On this cube the effect was decisive: the cube's true midpoint along
    the finger-closing axis was 0.5299 while the grasp was placed at
    0.5388. With a half-width of 21 mm, a 9 mm offset lands one finger on
    the top face and the other outside the far side -- the "one gripper
    hit the top surface, one is on the side" failure, reproducing on
    every attempt regardless of tilt, width or calibration.

    THE CORRECTION. Along the finger-closing axis the object's extent
    midpoint is a far better estimate of its centre than the mass
    centroid: min and max come from the two silhouette edges, which are
    both visible even when the interior is not, whereas the centroid is
    pulled by how many points each face contributed. Only this ONE axis
    is corrected -- the approach axis is deliberately left alone (depth
    along it is set by the gripper geometry, not the cloud) and so is the
    third axis, where an offset merely shifts where along the object the
    fingers land rather than whether they close on it at all.

    `max_shift` caps the correction so a bad mask (background leaking in,
    which makes the extent meaningless) cannot fling the grasp across the
    table. It is deliberately LARGER than any plausible graspable object:
    an earlier 0.03 cap silently clipped the correction for grasps GraspGen
    had placed near an object's edge, leaving them 15 mm off centre on a
    44 mm block -- 2.6 mm of finger clearance, which clipped it on contact.
    Guard the mask by checking the cloud's EXTENT (a 90 cm "cube" is a bad
    mask); do not guard it by half-correcting the grasp.
    """
    R = grasp_T_cam[:3, :3]
    t = grasp_T_cam[:3, 3]

    local_x = (pcd_cam - t) @ R[:, 0]
    shift_x = float(np.clip(0.5 * (local_x.min() + local_x.max()), -max_shift, max_shift))

    # DEPTH, along the approach axis. Skipping this was a real error:
    # the single-view centroid bias points TOWARD the camera, and for a
    # near-vertical grasp that is mostly UPWARD, so the grasp sits too
    # high. Measured on a 42 mm cube: the fingertips reached only 5.9 mm
    # below its top face, closing on the top edge while the rest of the
    # cube hung below them. Nothing about the aim was wrong - the grasp
    # was simply too shallow to have the object between the fingers.
    #
    # Target: put the FINGERTIP PLANE (gripper_depth along +Z) at the
    # object's midpoint along the approach axis, so the fingers span the
    # upper half of the object. Aiming deeper (at the contact band's own
    # centre, 0.75 * depth) would put the tips below the object and into
    # the table on a short object, which is the one failure worse than a
    # missed grasp.
    local_z = (pcd_cam - t) @ R[:, 2]
    object_mid_z = float(0.5 * (local_z.min() + local_z.max()))
    shift_z = float(np.clip(object_mid_z - gripper_depth, -max_shift, max_shift))

    corrected = grasp_T_cam.copy()
    corrected[:3, 3] = t + R[:, 0] * shift_x + R[:, 2] * shift_z
    return corrected


def principal_axis_xy(points: np.ndarray):
    """Longest horizontal axis of a point cloud, and how elongated it is.

    Returns (axis_unit_3, elongation) where axis_unit_3 is a unit vector in
    the SAME frame as `points` lying in the x/y (table) plane, and
    elongation is sqrt(lambda0 / lambda1) of the 2D covariance -- 1.0 for a
    round/square footprint, >1 for a banana/carrot/pen. z is ignored on
    purpose: the grasp cross-axis preference only cares which way the
    object is long ACROSS THE TABLE, since the fingers close in a roughly
    horizontal plane for a top-down grasp.
    """
    p = np.asarray(points, dtype=np.float64)[:, :2]
    p = p - p.mean(axis=0)
    if len(p) < 3:
        return np.array([1.0, 0.0, 0.0]), 1.0
    cov = (p.T @ p) / len(p)
    vals, vecs = np.linalg.eigh(cov)          # ascending
    long_2d = vecs[:, -1]
    elong = float(np.sqrt(max(vals[-1], 1e-12) / max(vals[0], 1e-12)))
    return np.array([long_2d[0], long_2d[1], 0.0]), elong


def _crossaxis_factor(grasps_T_base: np.ndarray, long_axis: np.ndarray,
                      weight: float) -> np.ndarray:
    """Per-candidate multiplier in [1-weight, 1] rewarding grasps whose
    fingers close ACROSS the object's long axis, not along it.

    GraspGen's franka_panda convention: local X is the finger-closing axis
    (see estimate_gripper_width). We want that axis perpendicular to
    `long_axis` (grip the banana across its width, not end-to-end). A grasp
    whose fingers close exactly along the length keeps only `1-weight` of
    its confidence; a perfectly crosswise one keeps all of it. This is a
    SOFT re-rank, never a filter -- if every grasp is lengthwise the list
    still comes back, just reordered.
    """
    fx = grasps_T_base[:, :3, 0]                      # finger-closing axis, base frame
    fx_xy = fx[:, :2]
    n = np.linalg.norm(fx_xy, axis=1)
    n[n < 1e-9] = 1e-9
    fx_xy = fx_xy / n[:, None]
    la = long_axis[:2] / max(np.linalg.norm(long_axis[:2]), 1e-9)
    along = np.abs(fx_xy @ la)                        # 1 = along length (bad), 0 = across (good)
    return 1.0 - weight * along


def rank_grasp_indices(grasps_T_base: np.ndarray, confidences=None,
                       min_confidence: float = 0.0,
                       max_tilt_deg: float = 0.0,
                       object_long_axis=None,
                       crossaxis_weight: float = 0.0) -> np.ndarray:
    """Every candidate that passes the tilt gate, best-first.

    Same gate and same ordering as select_grasp_index() -- which is now a
    thin wrapper over this -- but it returns the WHOLE ranked list instead
    of only the winner, so a caller can walk down it and apply a further
    test that needs the grasp pose itself.

    `object_long_axis` (+ `crossaxis_weight` > 0) applies a soft re-rank
    that prefers grasps closing across that axis rather than along it --
    the fix for elongated objects (banana, carrot), where the highest
    confidence grasp is often a physically useless end-to-end pinch. Pass
    None / 0.0 to leave ordering by confidence alone (the cube path).

    WHY THIS EXISTS (measured 2026-08-26). The tilt gate checks the
    approach ANGLE but says nothing about whether the fingers actually
    span the object. On a real cube, the winning candidate closed its
    fingers along the visible surface's normal, and
    estimate_gripper_width() correctly returned 0.0058 m -- the thickness
    of the single-view shell, not the 6 cm cube. Commanding that width
    closes the gripper almost fully and the object is pushed aside
    instead of grasped. A single winner gives the caller nothing to fall
    back to; a ranked list lets it skip that candidate and take the next
    one that both points down AND spans the object.

    Returns an empty array if nothing survives the tilt gate (callers must
    check, exactly as they must check select_grasp_index()'s -1).
    """
    approach_axis = grasps_T_base[:, :3, :3] @ np.array([0.0, 0.0, 1.0])
    tilt_deg = np.degrees(np.arccos(np.clip(-approach_axis[:, 2], -1.0, 1.0)))

    keep = tilt_deg <= max_tilt_deg if max_tilt_deg > 0.0 else np.ones(len(grasps_T_base), bool)
    candidates = np.where(keep)[0]
    if len(candidates) == 0:
        return candidates

    if confidences is None:
        return candidates[np.argsort(tilt_deg[candidates])]

    conf = np.asarray(confidences, dtype=np.float64).reshape(-1)
    if len(conf) != len(grasps_T_base):
        raise ValueError(f"{len(conf)} confidences for {len(grasps_T_base)} grasps")

    # min_confidence stays best-effort (see select_grasp_index): applied
    # only while it leaves something, never emptying the tilt survivors.
    confident = candidates[conf[candidates] >= min_confidence]
    if len(confident):
        candidates = confident

    rank_score = conf[candidates]
    if object_long_axis is not None and crossaxis_weight > 0.0:
        rank_score = rank_score * _crossaxis_factor(
            grasps_T_base[candidates], np.asarray(object_long_axis, float),
            float(np.clip(crossaxis_weight, 0.0, 1.0)))
    return candidates[np.argsort(-rank_score)]


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
