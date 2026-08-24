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


def _import_perception_client():
    """Import PerceptionClient from calibration/ without installing it.

    calibration/ is a scripts directory, not a package, and the bridge runs
    from a built ament install space where it is not on the path.
    """
    here = Path(__file__).resolve()
    for parent in here.parents:
        cand = parent / "calibration" / "perception_client.py"
        if cand.is_file():
            sys.path.insert(0, str(cand.parent))
            from perception_client import PerceptionClient  # noqa: PLC0415
            return PerceptionClient
    raise ImportError(
        "calibration/perception_client.py not found above "
        f"{here} - is the repo layout intact?"
    )


class LivePerception:
    """Resolves an object name to ranked grasp poses in the camera frame."""

    def __init__(self, host: str = "127.0.0.1",
                 perception_port: int = DEFAULT_PERCEPTION_PORT,
                 graspgen_port: int = DEFAULT_GRASPGEN_PORT,
                 timeout_ms: int = 60_000) -> None:
        self.host = host
        self.perception_port = perception_port
        self.graspgen_port = graspgen_port
        self.timeout_ms = timeout_ms

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

        return grasps, confidences, ""
