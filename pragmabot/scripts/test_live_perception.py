#!/usr/bin/env python3
"""Offline checks for bridge_node's live perception chain.

No robot, no GPU, no servers: both ZMQ clients are stubbed. What this
guards is the wiring and the CENTROID CONVENTION - GraspGen returns poses
relative to a recentered cloud, so the centroid must be added back. Get it
wrong and every grasp is offset by the object's position in the camera
frame, silently, while still looking like a plausible pose.

    python scripts/test_live_perception.py
"""

import sys
import types
from pathlib import Path
from unittest import mock

import numpy as np

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "ros2_ws" / "src" / "pragmabot_bridge"))

from pragmabot_bridge.live_perception import LivePerception  # noqa: E402


class _Res:
    def __init__(self, ok, points=None, reason="", label="cup", conf=0.93):
        self.ok, self.points, self.reason = ok, points, reason
        self.label, self.confidence = label, conf
        self.n_points = 0 if points is None else len(points)


def _stub_perception(result):
    """Patch the PerceptionClient factory to return a canned result."""
    class _Client:
        def __init__(self, *a, **k): pass
        def __enter__(self): return self
        def __exit__(self, *a): return False
        def detect(self, *a, **k): return result
    return mock.patch(
        "pragmabot_bridge.live_perception._import_perception_client",
        return_value=_Client)


def _stub_graspgen(grasps, confidences, capture=None):
    """Install a fake grasp_gen.serving.zmq_client module."""
    class _GG:
        def __init__(self, *a, **k): pass
        def __enter__(self): return self
        def __exit__(self, *a): return False
        def infer(self, pcd, **k):
            if capture is not None:
                capture["pcd"] = np.asarray(pcd).copy()
            return grasps, confidences

    mod = types.ModuleType("grasp_gen.serving.zmq_client")
    mod.GraspGenClient = _GG
    pkg = types.ModuleType("grasp_gen")
    serving = types.ModuleType("grasp_gen.serving")
    return mock.patch.dict(sys.modules, {
        "grasp_gen": pkg, "grasp_gen.serving": serving,
        "grasp_gen.serving.zmq_client": mod})


def _cloud(offset=(0.5, -0.1, 0.7), n=800):
    rng = np.random.default_rng(0)
    return (rng.normal(scale=0.02, size=(n, 3)) + np.asarray(offset))


def _identity_grasps(k=3):
    return np.stack([np.eye(4) for _ in range(k)]), np.array([0.9, 0.8, 0.7])


# --- the important one ------------------------------------------------

def test_centroid_is_added_back_to_grasps():
    """GraspGen sees a CENTERED cloud; returned grasps must be un-centered."""
    cloud = _cloud(offset=(0.5, -0.1, 0.7))
    centroid = cloud.mean(axis=0)
    seen = {}
    with _stub_perception(_Res(True, cloud)), _stub_graspgen(*_identity_grasps(), capture=seen):
        grasps, confs, pcd, reason = LivePerception().grasps_for(
            "red cup", np.zeros((4, 4, 3), np.uint8), np.zeros((4, 4), np.float32),
            {"fx": 1, "fy": 1, "cx": 0, "cy": 0})

    assert grasps is not None, reason
    # GraspGen must have been handed a cloud centred on the origin.
    assert np.allclose(seen["pcd"].mean(axis=0), 0, atol=1e-9), \
        "cloud sent to GraspGen was not recentered"
    # And identity grasps must come back translated BY that centroid.
    assert np.allclose(grasps[0][:3, 3], centroid, atol=1e-9), \
        "centroid was not added back - every grasp would be silently offset"


def test_detection_failure_reason_passes_through_unchanged():
    """The VLM reflects on this text; the bridge must not flatten it."""
    detailed = ("ambiguous detection for 'cube.': 'cube' (0.57) vs "
                "'e-stop button' (0.48), margin 0.09 < 0.15")
    with _stub_perception(_Res(False, reason=detailed)):
        g, c, p, reason = LivePerception().grasps_for(
            "cube", np.zeros((4, 4, 3), np.uint8), np.zeros((4, 4), np.float32), {})
    assert g is None and reason == detailed


def test_empty_target_object_refused():
    g, c, p, reason = LivePerception().grasps_for(
        "  ", np.zeros((4, 4, 3), np.uint8), np.zeros((4, 4), np.float32), {})
    assert g is None and "no target object" in reason


def test_zero_grasps_gives_an_actionable_reason():
    with _stub_perception(_Res(True, _cloud())), \
         _stub_graspgen(np.zeros((0, 4, 4)), np.zeros((0,))):
        g, c, p, reason = LivePerception().grasps_for(
            "cup", np.zeros((4, 4, 3), np.uint8), np.zeros((4, 4), np.float32), {})
    assert g is None and "no feasible grasp" in reason
    # The cloud is still returned, so the caller can log what was seen.
    assert p is not None


def test_graspgen_missing_is_a_reason_not_a_crash():
    with _stub_perception(_Res(True, _cloud())), \
         mock.patch.dict(sys.modules, {"grasp_gen.serving.zmq_client": None}):
        g, c, p, reason = LivePerception().grasps_for(
            "cup", np.zeros((4, 4, 3), np.uint8), np.zeros((4, 4), np.float32), {})
    assert g is None and "GraspGen client not importable" in reason


def test_bad_grasp_shape_refused():
    with _stub_perception(_Res(True, _cloud())), \
         _stub_graspgen(np.zeros((5, 3, 3)), np.zeros((5,))):
        g, c, p, reason = LivePerception().grasps_for(
            "cup", np.zeros((4, 4, 3), np.uint8), np.zeros((4, 4), np.float32), {})
    assert g is None and "unexpected shape" in reason


def test_success_reports_confidence_range():
    with _stub_perception(_Res(True, _cloud())), _stub_graspgen(*_identity_grasps()):
        g, c, p, reason = LivePerception().grasps_for(
            "red cup", np.zeros((4, 4, 3), np.uint8), np.zeros((4, 4), np.float32), {})
    assert g is not None and len(g) == 3
    assert "red cup" in reason and "0.70-0.90" in reason


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
        print(f"  ok  {t.__name__}")
    print(f"\n{len(tests)} passed")
