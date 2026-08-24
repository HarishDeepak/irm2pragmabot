"""Live perception + grasp generation for bridge_node.

Replaces the `grasp_file` / `object_pcd_file` parameters, which made the
bridge pick whatever was segmented in the last OFFLINE run regardless of
what the planner asked for. That is not merely a grasping bug: the success
detector may then report True because *something* moved, and a fabricated
success is written into the long-term memory the project is graded on.

Chain, per pick:

    target_object (text from the planner)
        -> PerceptionClient  (ZMQ :5557, groundedsam venv)
             GroundingDINO + SAM 2 -> mask -> back-project -> filter
        -> object cloud, camera frame, ~2000 pts
        -> GraspGenClient    (ZMQ :5556, graspgen venv, ships with GraspGen)
        -> (N, 4, 4) grasp poses + confidences, camera frame

Two processes because the venvs cannot be merged: GraspGen pins
torch==2.1.0 with CUDA extensions compiled against it, Grounded-SAM-2 needs
torch>=2.3.1. See calibration/perception_server.py.

CENTROID CONVENTION — the subtle one. graspgen_client.py recenters the
cloud (`xyz -= xyz.mean(0)`) before sending, so returned grasps are
relative to the recentered cloud, and load_all_grasps() adds the saved
centroid back. This module talks to the GraspGen server directly, so it
must do the same recentre-then-restore itself. Getting it wrong offsets
every grasp by the object's own position in the camera frame — a large,
silent error that still produces plausible-looking poses.

Every failure returns a REASON string rather than raising. The bridge puts
it in the ExecuteSkill result, the planner writes it into STM, and the VLM
reflects on that text — so the wording is part of the graded mechanism.
"""

import logging
import sys
from pathlib import Path
from typing import Optional, Tuple

import numpy as np

logger = logging.getLogger(__name__)

DEFAULT_PERCEPTION_PORT = 5557
DEFAULT_GRASPGEN_PORT = 5556

# GraspGen OOMs on scene-scale clouds; it expects object-scale input.
# perception_server already downsamples to ~2000, this is a backstop.
GRASPGEN_TARGET_POINTS = 2000
MIN_GRASPS = 1


def _calibration_dir() -> Path:
    """Locate calibration/ and put it on sys.path.

    calibration/ is a scripts directory, not a package, and the bridge runs
    from a built ament install space where it is not importable. Everything
    the bridge borrows from it (perception_client, mask_to_pointcloud,
    mask_sampling) is pure numpy/zmq by design, precisely so it can be
    shared across environments that cannot share a torch.
    """
    here = Path(__file__).resolve()
    for parent in here.parents:
        cand = parent / "calibration" / "perception_client.py"
        if cand.is_file():
            if str(cand.parent) not in sys.path:
                sys.path.insert(0, str(cand.parent))
            return cand.parent
    raise ImportError(
        "calibration/perception_client.py not found above "
        f"{here} - is the repo layout intact?"
    )


def _import_perception_client():
    """Import PerceptionClient from calibration/ without installing it."""
    _calibration_dir()
    from perception_client import PerceptionClient  # noqa: PLC0415
    return PerceptionClient


class LivePerception:
    """Resolves an object name to ranked grasp poses in the camera frame."""

    def __init__(self, host: str = "127.0.0.1",
                 perception_port: int = DEFAULT_PERCEPTION_PORT,
                 graspgen_port: int = DEFAULT_GRASPGEN_PORT,
                 timeout_ms: int = 60_000,
                 grasp_topk: int = 0) -> None:
        self.host = host
        self.perception_port = perception_port
        self.graspgen_port = graspgen_port
        self.timeout_ms = timeout_ms
        # Keep only the top-K candidates by GraspGen's own confidence
        # before they ever reach select_grasp_index. 0 = no cap (all
        # candidates GraspGen returns, currently ~100). Client-side, not
        # passed to the server's own topk_num_grasps: that parameter was
        # found NOT to hard-cap the response (a request for 6 returned 36
        # in testing) - it appears to be a per-iteration, not global, cap
        # server-side. Slicing here is the only way to get an exact count.
        self.grasp_topk = grasp_topk

    def grasps_for(self, target_object: str, rgb: np.ndarray, depth: np.ndarray,
                   intrinsics) -> Tuple[Optional[np.ndarray], Optional[np.ndarray],
                                        Optional[np.ndarray], str]:
        """Detect `target_object` and generate grasps for it.

        Returns:
            (grasps_T_cam, confidences, object_pcd_cam, reason). On success
            reason is a short note; on failure the first three are None and
            reason explains why, in language the VLM can reflect on.
        """
        if not target_object or not target_object.strip():
            return None, None, None, "no target object was named in the action"

        cloud, reason = self._detect(target_object, rgb, depth, intrinsics)
        if cloud is None:
            return None, None, None, reason

        grasps, confs, greason = self._generate(cloud)
        if grasps is None:
            return None, None, cloud, greason

        return grasps, confs, cloud, (
            f"{len(grasps)} grasps for {target_object!r} "
            f"(conf {confs.min():.2f}-{confs.max():.2f})"
        )

    # -- placement ----------------------------------------------------

    def surface_point_for(self, placement_object: str, rgb, depth, intrinsics,
                          n_candidates: int = 16, min_interior_px: int = 8,
                          patch_px: int = 5, max_range: float = 3.0,
                          max_mask_frac: float = 0.85):
        """Resolve a placement surface name to ONE 3D point on it, camera frame.

        Deliberately NOT the grasp path. GraspGen answers "how do I close a
        gripper around this object"; a table, plate or tray is not being
        grasped, and a grasp pose on it would be meaningless. What place
        needs is a single point on the surface, far enough from its rim that
        the released object stays on it.

        Same detector, same guards as picking — this reuses
        PerceptionClient.detect(), so the ambiguity margin, the mask-area
        bounds and the minimum-surviving-points check all apply unchanged.
        The only addition is `return_mask`: the choice of WHERE on the
        surface is made in 2D (calibration/mask_sampling.py) and then
        back-projected.

        Returns:
            (point_cam, reason, info). `point_cam` is (3,) float64 in the
            camera optical frame, or None on failure; `reason` is always
            populated and is written verbatim into the ExecuteSkill result,
            so it must read as an explanation, not a log line.
        """
        if not placement_object or not placement_object.strip():
            return None, "no placement object was named in the action", None

        try:
            PerceptionClient = _import_perception_client()
        except ImportError as exc:
            return None, f"perception client unavailable: {exc}", None

        prompt = placement_object.strip()
        if not prompt.endswith("."):
            prompt += "."

        try:
            with PerceptionClient(self.host, self.perception_port,
                                  timeout_ms=self.timeout_ms) as client:
                # A surface legitimately covers far more of the frame than
                # a graspable object; the object-sized cap would refuse
                # every real table. Every other guard is unchanged.
                res = client.detect(rgb, depth, intrinsics, prompt,
                                    return_mask=True,
                                    max_mask_frac=max_mask_frac)
        except Exception as exc:  # noqa: BLE001
            return None, (f"perception request for the placement surface "
                          f"failed: {type(exc).__name__}: {exc}"), None

        if not res.ok:
            return None, res.reason, None
        if res.mask is None:
            return None, ("the perception server returned no mask for "
                          f"{placement_object!r}; it is running a build without "
                          "return_mask support, so the placement point cannot "
                          "be chosen"), None

        _calibration_dir()
        import mask_sampling  # noqa: PLC0415
        import mask_to_pointcloud as m2p  # noqa: PLC0415
        from perception_client import _as_intrinsics_dict  # noqa: PLC0415

        # raw_cloud() indexes K as a dict; the caller may hand us a 3x3.
        # PerceptionClient normalises it on the way to the server, but the
        # back-projection below happens HERE, so it must normalise too.
        K = _as_intrinsics_dict(intrinsics)

        depth = np.asarray(depth)
        # Only pixels the stereo matcher actually resolved are candidates.
        # Choosing first and discovering the depth is a hole second wastes
        # the whole selection.
        valid = np.isfinite(depth) & (depth > 0) & (depth <= max_range)

        try:
            u, v, info = mask_sampling.select_placement_pixel(
                res.mask, valid=valid, n_candidates=n_candidates,
                min_interior_px=min_interior_px)
        except ValueError as exc:
            return None, (f"{res.label!r} was detected but no point on it has "
                          f"usable depth ({exc})"), None

        # Back-project a small patch rather than the single chosen pixel, and
        # take its median: one pixel's depth is one stereo match, and a
        # single bad match would move the release point by tens of cm. The
        # patch runs through m2p.raw_cloud() so the pinhole math is literally
        # the same code the object cloud is built with.
        half = max(int(patch_px) // 2, 0)
        patch = np.zeros(depth.shape, dtype=bool)
        patch[max(v - half, 0):v + half + 1, max(u - half, 0):u + half + 1] = True
        patch &= res.mask & valid

        try:
            patch_xyz = m2p.raw_cloud(depth, patch, K, max_range, 0)
        except Exception as exc:  # noqa: BLE001
            return None, (f"back-projecting the chosen point on {res.label!r} "
                          f"failed: {type(exc).__name__}: {exc}"), None

        point_cam = np.median(patch_xyz.astype(np.float64), axis=0)

        # Sanity check: the point must lie inside the surface cloud's own
        # robust extent. Same percentile/pad thresholds mask_to_pointcloud
        # already uses for its AABB crop - not new numbers. This catches a
        # depth hole filled with a background match, which would otherwise
        # place the object metres behind the table.
        lo, hi = m2p.aabb_bounds(res.points.astype(np.float64),
                                 percentile=2.0, pad_frac=0.05, pad_abs=0.0)
        if not np.all((point_cam >= lo) & (point_cam <= hi)):
            return None, (
                f"the chosen point on {res.label!r} back-projected to "
                f"{np.round(point_cam, 3).tolist()} m, outside that surface's "
                f"own visible extent {np.round(lo, 3).tolist()}..."
                f"{np.round(hi, 3).tolist()} - the depth at that pixel is not "
                "on the surface"), None

        info = dict(info)
        info.pop("candidates", None)
        info.update({"u": u, "v": v, "label": res.label,
                     "confidence": float(res.confidence),
                     "n_surface_points": int(res.n_points),
                     "patch_points": int(len(patch_xyz))})

        reason = (
            f"placement point perceived on {res.label!r} (conf "
            f"{res.confidence:.2f}): pixel ({u}, {v}), {info['interior_px']} px "
            f"from the surface edge, chosen as the most interior of "
            f"{info['n_candidates']} farthest-point candidates"
            + (f" (edge margin relaxed to {info['margin_px']} px - the surface "
               "is narrow)" if info["relaxed"] else "")
        )
        return point_cam, reason, info

    # -- stages -------------------------------------------------------

    def _detect(self, target_object: str, rgb, depth, intrinsics):
        try:
            PerceptionClient = _import_perception_client()
        except ImportError as exc:
            return None, f"perception client unavailable: {exc}"

        # GroundingDINO expects phrases terminated by a period.
        prompt = target_object.strip()
        if not prompt.endswith("."):
            prompt += "."

        try:
            with PerceptionClient(self.host, self.perception_port,
                                  timeout_ms=self.timeout_ms) as client:
                res = client.detect(rgb, depth, intrinsics, prompt)
        except Exception as exc:  # noqa: BLE001
            return None, f"perception request failed: {type(exc).__name__}: {exc}"

        if not res.ok:
            # res.reason already names the object, the confidences and the
            # guard that fired - pass it through unchanged.
            return None, res.reason

        logger.info("detected %r conf=%.3f, %d points",
                    res.label, res.confidence, res.n_points)
        return res.points.astype(np.float64), ""

    def _generate(self, cloud: np.ndarray):
        try:
            from grasp_gen.serving.zmq_client import GraspGenClient  # noqa: PLC0415
        except ImportError as exc:
            return None, None, (
                "GraspGen client not importable in this environment "
                f"({exc}). bridge_node must run where grasp_gen is on the "
                "path, or GraspGen must be reached over ZMQ from a process "
                "that has it."
            )

        pcd = cloud[:, :3].astype(np.float64)
        if len(pcd) > GRASPGEN_TARGET_POINTS:
            idx = np.random.default_rng(0).choice(
                len(pcd), GRASPGEN_TARGET_POINTS, replace=False)
            pcd = pcd[idx]

        # Recentre exactly as graspgen_client.py does, and keep the centroid
        # so it can be added back below. Skipping this offsets every grasp
        # by the object's position in the camera frame.
        centroid = pcd.mean(axis=0)
        pcd_centered = pcd - centroid

        try:
            with GraspGenClient(host=self.host, port=self.graspgen_port) as gg:
                grasps, confidences = gg.infer(pcd_centered)
        except Exception as exc:  # noqa: BLE001
            return None, None, (
                f"grasp generation failed: {type(exc).__name__}: {exc} "
                f"(is graspgen_server.py running on port {self.graspgen_port}?)"
            )

        grasps = np.asarray(grasps, dtype=np.float64)
        confidences = np.asarray(confidences, dtype=np.float64)

        if grasps.ndim != 3 or grasps.shape[1:] != (4, 4):
            return None, None, (
                f"grasp generator returned an unexpected shape {grasps.shape}; "
                "expected (N, 4, 4)")
        if len(grasps) < MIN_GRASPS:
            return None, None, (
                "no feasible grasp was generated for this object - it may be "
                "too large for the gripper, or the visible surface too small")

        # Undo the recentring: grasps come back relative to the centered
        # cloud (see load_all_grasps() in grasp_transform.py).
        grasps = grasps.copy()
        grasps[:, :3, 3] += centroid

        if self.grasp_topk > 0 and len(grasps) > self.grasp_topk:
            order = np.argsort(-confidences)[:self.grasp_topk]
            grasps = grasps[order]
            confidences = confidences[order]

        return grasps, confidences, ""
