#!/usr/bin/env python3
"""Offline check for perception_server's detection guards.

Runs with NO torch, NO GPU, NO server, NO robot: it stubs out the detector and
exercises only the guard branches in PerceptionServer.handle().

Those guards are the thing standing between "pick the red cup" and the arm
grasping the emergency-stop button, so they need a check that fails if someone
loosens them. The e-stop case is real measured data: cube 0.57 vs e-stop 0.48,
a 0.09 margin.

    python scripts/test_perception_guards.py
"""

import io
import sys
from pathlib import Path
from unittest import mock

import numpy as np

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "calibration"))


def _load_server_module():
    """Import perception_server without importing torch/SAM2/GroundingDINO."""
    fake = mock.MagicMock()
    with mock.patch.dict(sys.modules, {
        "detect_object": fake,
        "mask_to_pointcloud": fake,
        "torch": fake,
        "torchvision": fake,
        "torchvision.ops": fake,
    }):
        import importlib
        return importlib.import_module("perception_server")


ps = _load_server_module()


def _npy(arr):
    buf = io.BytesIO()
    np.save(buf, np.asarray(arr), allow_pickle=False)
    return buf.getvalue()


def _req(prompt="cube."):
    return {"rgb": _npy(np.zeros((720, 1280, 3), np.uint8)),
            "depth": _npy(np.full((720, 1280), 0.7, np.float32)),
            "intrinsics": {"K": [528.604, 0, 635.405, 0, 528.604, 363.729, 0, 0, 1]},
            "prompt": prompt}


def _server(detect_return, cloud=None):
    """A PerceptionServer with __init__ skipped and detect() stubbed."""
    srv = object.__new__(ps.PerceptionServer)
    srv.device, srv.box_threshold, srv.text_threshold = "cpu", 0.35, 0.25
    srv.detect = lambda rgb, prompt: detect_return
    if cloud is not None:
        ps.m2p.backproject = lambda **kw: cloud
    return srv


def _mask(frac, shape=(720, 1280)):
    m = np.zeros(shape, bool)
    m.ravel()[: int(m.size * frac)] = True
    return m


def test_empty_prompt_refused():
    r = _server((None, None, None)).handle({**_req(), "prompt": "  "})
    assert not r["ok"] and "empty prompt" in r["reason"], r


def test_no_detection_names_the_prompt():
    r = _server((None, None, None)).handle(_req("red cup."))
    assert not r["ok"] and "red cup" in r["reason"], r


def test_estop_ambiguity_refused():
    """The real failure: cube 0.57 vs e-stop 0.48. Margin 0.09 < 0.15."""
    masks = np.stack([_mask(0.02), _mask(0.02)])
    r = _server((masks, [0.57, 0.48], ["cube", "e-stop button"])).handle(_req())
    assert not r["ok"], "a 0.09 margin must be refused, not resolved by argmax"
    assert "ambiguous" in r["reason"]
    # Both candidates must be named, so the VLM can disambiguate.
    assert "cube" in r["reason"] and "e-stop" in r["reason"], r["reason"]


def test_clear_winner_accepted():
    masks = np.stack([_mask(0.02), _mask(0.02)])
    r = _server((masks, [0.91, 0.30], ["cube", "shadow"]),
                cloud=np.random.rand(1500, 3).astype(np.float32)).handle(_req())
    assert r["ok"], r
    assert r["label"] == "cube" and r["n_points"] == 1500


def test_mask_too_large_refused():
    r = _server((np.stack([_mask(0.85)]), [0.9], ["table"])).handle(_req())
    assert not r["ok"] and "implausible mask" in r["reason"], r


def test_mask_too_small_refused():
    r = _server((np.stack([_mask(0.0002)]), [0.9], ["speck"])).handle(_req())
    assert not r["ok"] and "implausible mask" in r["reason"], r


def test_too_few_points_refused():
    r = _server((np.stack([_mask(0.02)]), [0.9], ["cup"]),
                cloud=np.random.rand(42, 3).astype(np.float32)).handle(_req())
    assert not r["ok"] and "depth failed" in r["reason"], r
    assert "42" in r["reason"], "the reason must carry the real number"


def test_exception_becomes_a_reason_not_a_crash():
    """The server must never die on one bad request."""
    def boom(**kw):
        raise RuntimeError("AABB crop removed every point")
    srv = _server((np.stack([_mask(0.02)]), [0.9], ["cup"]))
    ps.m2p.backproject = boom
    r = srv.handle(_req())
    assert not r["ok"] and "AABB crop removed every point" in r["reason"], r


def test_every_failure_has_a_nonempty_reason():
    """Reasons feed STM. An empty one gives the VLM nothing to reflect on."""
    for r in [_server((None, None, None)).handle(_req()),
              _server((np.stack([_mask(0.85)]), [0.9], ["table"])).handle(_req())]:
        assert not r["ok"] and len(r.get("reason", "")) > 20, r


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
        print(f"  ok  {t.__name__}")
    print(f"\n{len(tests)} passed")
