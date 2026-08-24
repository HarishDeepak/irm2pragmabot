#!/usr/bin/env python3
"""perception_client.py — client for perception_server.py.

Pure stdlib + numpy + zmq/msgpack: NO torch, NO cv2. That is the point. This
runs in the ROS 2 environment (or any other), while the models it drives live
in the Grounded-SAM-2 venv that the ROS 2 env cannot import.

Usage from bridge_node.py:

    from perception_client import PerceptionClient

    with PerceptionClient() as perception:
        result = perception.detect(rgb, depth, K, target_object)
    if not result.ok:
        return _result(False, result.reason)   # reason -> STM
    cloud = result.points                      # -> GraspGenClient.infer(cloud)

Standalone smoke test against the extracted scene:

    python calibration/perception_client.py \\
        --rgb extracted/red_cup/rgb.png \\
        --depth extracted/red_cup/depth.npy \\
        --intrinsics extracted/red_cup/intrinsics.json \\
        --prompt "red cup."
"""

import io
from dataclasses import dataclass
from typing import Optional

import numpy as np

DEFAULT_PORT = 5557
DEFAULT_TIMEOUT_MS = 30_000


@dataclass
class DetectionResult:
    """Either a cloud, or a reason the VLM can reflect on. Never both."""
    ok: bool
    points: Optional[np.ndarray] = None
    n_points: int = 0
    confidence: float = 0.0
    label: str = ""
    reason: str = ""


class PerceptionClient:
    """REQ client for the perception server.

    Mirrors GraspGen's GraspGenClient usage so bridge_node speaks one pattern
    to both servers.
    """

    def __init__(self, host: str = "127.0.0.1", port: int = DEFAULT_PORT,
                 timeout_ms: int = DEFAULT_TIMEOUT_MS) -> None:
        self.host, self.port, self.timeout_ms = host, port, timeout_ms
        self._ctx = None
        self._sock = None

    def __enter__(self) -> "PerceptionClient":
        self.connect()
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def connect(self) -> None:
        import zmq
        self._ctx = zmq.Context()
        self._sock = self._ctx.socket(zmq.REQ)
        # Without RCVTIMEO a dead server hangs the bridge forever, and a
        # blocking action callback that never returns is exactly the
        # single-threaded-executor deadlock we are trying to avoid.
        self._sock.setsockopt(zmq.RCVTIMEO, self.timeout_ms)
        self._sock.setsockopt(zmq.SNDTIMEO, self.timeout_ms)
        self._sock.setsockopt(zmq.LINGER, 0)
        self._sock.connect(f"tcp://{self.host}:{self.port}")

    def close(self) -> None:
        if self._sock is not None:
            self._sock.close()
        if self._ctx is not None:
            self._ctx.term()
        self._sock = self._ctx = None

    def detect(self, rgb: np.ndarray, depth: np.ndarray, K, prompt: str,
               **kwargs) -> DetectionResult:
        """Ask for the object named by `prompt`. Never raises.

        `K` may be the flat dict loaded straight from intrinsics.json
        ({"fx","fy","cx","cy"}) or a 3x3 matrix; both are normalised to the
        dict form that mask_to_pointcloud.backproject() indexes.
        """
        import msgpack
        import zmq

        if self._sock is None:
            self.connect()

        req = {"rgb": _npy_dump(rgb), "depth": _npy_dump(depth),
               "intrinsics": _as_intrinsics_dict(K), "prompt": prompt}
        req.update(kwargs)

        try:
            self._sock.send(msgpack.packb(req, use_bin_type=True))
            rep = msgpack.unpackb(self._sock.recv(), raw=False)
        except zmq.Again:
            # REQ sockets cannot recover from a timeout mid-exchange; the
            # socket is stuck in the wrong state, so rebuild it.
            self.close()
            return DetectionResult(
                ok=False,
                reason=f"perception server did not answer within "
                       f"{self.timeout_ms / 1000:.0f}s (is it running on port {self.port}?)")
        except Exception as exc:  # noqa: BLE001
            self.close()
            return DetectionResult(ok=False, reason=f"perception transport error: {exc}")

        if not rep.get("ok"):
            return DetectionResult(ok=False, reason=rep.get("reason", "unknown failure"))

        return DetectionResult(ok=True, points=_npy_load(rep["points"]),
                               n_points=rep["n_points"],
                               confidence=rep["confidence"], label=rep["label"])


def _as_intrinsics_dict(K) -> dict:
    """Normalise intrinsics to the flat dict backproject() expects.

    Accepts intrinsics.json's own form (fx/fy/cx/cy, plus extra keys like
    width/height/frame_id, which are passed through harmlessly) or a 3x3
    matrix. Mixing the two silently is how a 3x3 ends up being indexed as
    K["fx"] and raising IndexError deep inside back-projection.
    """
    if isinstance(K, dict):
        return {k: (float(v) if isinstance(v, (int, float)) else v)
                for k, v in K.items()}
    m = np.asarray(K, dtype=float).reshape(3, 3)
    return {"fx": float(m[0, 0]), "fy": float(m[1, 1]),
            "cx": float(m[0, 2]), "cy": float(m[1, 2])}


def _npy_dump(arr: np.ndarray) -> bytes:
    buf = io.BytesIO()
    np.save(buf, np.asarray(arr), allow_pickle=False)
    return buf.getvalue()


def _npy_load(raw: bytes) -> np.ndarray:
    return np.load(io.BytesIO(raw), allow_pickle=False)


def main() -> None:
    import argparse
    import json

    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--rgb", required=True)
    p.add_argument("--depth", required=True)
    p.add_argument("--intrinsics", required=True)
    p.add_argument("--prompt", required=True)
    p.add_argument("--out", default=None, help="optional .npy to write the cloud to")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=DEFAULT_PORT)
    args = p.parse_args()

    rgb = np.load(args.rgb) if args.rgb.endswith(".npy") else _imread(args.rgb)
    depth = np.load(args.depth)
    with open(args.intrinsics) as fh:
        intr = json.load(fh)

    with PerceptionClient(args.host, args.port) as client:
        res = client.detect(rgb, depth, intr, args.prompt)

    if not res.ok:
        print(f"REFUSED: {res.reason}")
        raise SystemExit(1)

    print(f"OK: {res.label!r} conf={res.confidence:.3f}, {res.n_points} points")
    lo, hi = res.points.min(axis=0), res.points.max(axis=0)
    print(f"extent (cm): {(hi - lo) * 100}")
    if args.out:
        np.save(args.out, res.points)
        print(f"wrote {args.out}")


def _imread(path: str) -> np.ndarray:
    from PIL import Image
    return np.array(Image.open(path).convert("RGB"))


if __name__ == "__main__":
    main()
