#!/usr/bin/env python3
"""perception_server.py — ZMQ REP server: text prompt + RGB-D -> object point cloud.

WHY THIS EXISTS
---------------
`detect_object.py` and `mask_to_pointcloud.py` both work, but they are CLIs
that pass data through files. Wiring them into the running system by shelling
out would reload SAM2 + GroundingDINO on every pick (~10s each time), because
`detect_object.run_detection()` builds both models per call.

This server hoists that load to startup and then answers requests forever, so
the per-pick cost is inference only. It is the perception counterpart to
GraspGen's own ZMQ server (`GraspGen/client-server/graspgen_server.py`), and
deliberately mirrors its REQ/REP shape so the bridge speaks one pattern to both.

MUST run in the Grounded-SAM-2 venv (see CLAUDE.md — the venvs cannot be
merged; GraspGen pins torch==2.1.0 with CUDA extensions built against it,
Grounded-SAM-2 needs torch>=2.3.1):

    ~/groundedsam/.venv/bin/python calibration/perception_server.py

PROTOCOL (msgpack over ZMQ REQ/REP, one dict in, one dict out)

  request  {"rgb": <png bytes>, "depth": <npy bytes>, "intrinsics": {...},
            "prompt": "red cup.", "workspace": {...}|None,
            "return_mask": bool}
  reply    {"ok": true,  "points": <npy bytes>, "n_points": int,
            "confidence": float, "label": str,
            "mask": <npy bytes>}          # only when return_mask is set
           {"ok": false, "reason": "<why>"}

`return_mask` exists for PLACEMENT. A placement surface is not a graspable
object: the bridge does not want a cloud to hand to GraspGen, it wants one
point on the surface, chosen away from the rim. That choice is made on the
2D mask (see calibration/mask_sampling.py), so the mask has to come back.
It is off by default — the pick path has no use for it and a 1280x720 bool
array is ~0.9 MB per reply.

`reason` is not a log line. On failure the bridge puts that string into the
ExecuteSkill result message, which becomes an STM entry that the VLM reflects
on. "no object matched 'red cup' above 0.35" gives the planner something to
act on; "detection failed" does not.

GUARDS — a wrong object is worse than no object. The planner will happily
grasp whatever it is handed, and the success detector may then report True,
writing a fabricated success into the graded memory. Every guard below fails
loudly rather than falling through to a stale result.
"""

import argparse
import io
import logging
import os
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

import detect_object  # noqa: E402  (adds Grounded-SAM-2 to sys.path on import)
import mask_to_pointcloud as m2p  # noqa: E402

logger = logging.getLogger("perception_server")

# Guard thresholds. AMBIGUITY_MARGIN is the one that stops the e-stop button
# (measured conf 0.48) being grasped instead of the cube (0.57): that pair is
# 0.09 apart, so it is refused as ambiguous rather than silently resolved by
# argmax. See CLAUDE.md and the workspace-crop discussion.
AMBIGUITY_MARGIN = 0.15
MIN_MASK_FRAC = 0.001   # 0.1% of the image
MAX_MASK_FRAC = 0.40    # 40% of the image
MIN_POINTS = 200        # below this, depth failed on the object

# A PLACEMENT SURFACE is not a graspable object, and the 40% cap above is
# wrong for it. That cap exists because a mask covering half the frame
# means the detector latched onto the background instead of the cup. But a
# table you place ON legitimately fills half the view - measured 51% on
# this rig - so the object bound would refuse every real placement target.
# Requests may raise the cap to this value instead; it still rejects a
# degenerate mask that has swallowed the whole image.
MAX_SURFACE_MASK_FRAC = 0.85


# Spatial selectors for picking one of several same-looking detections.
# All are image-frame: +col is right, +row is DOWN. On a table viewed from
# the front the nearer object sits LOWER in the frame, so "near" == max row.
_SPATIAL_SELECTORS = {
    "left", "right", "near", "far", "top", "bottom", "largest", "smallest",
}
# Words a caller might use, mapped onto the canonical selector above.
_SPATIAL_ALIASES = {
    "leftmost": "left", "rightmost": "right",
    "front": "near", "frontmost": "near", "nearest": "near", "closest": "near",
    "back": "far", "rear": "far", "backmost": "far",
    "farthest": "far", "furthest": "far",
    "topmost": "top", "bottommost": "bottom",
    "biggest": "largest", "closest-to-camera": "near",
}


def _select_spatial(selector: str, masks):
    """Index of the mask that best matches `selector`, or None if unknown.

    `masks` is (N, H, W) bool. Uses each mask's pixel centroid (and area
    for largest/smallest). Ties are broken by numpy argmin/argmax order,
    which is deterministic.
    """
    sel = _SPATIAL_ALIASES.get(selector, selector)
    if sel not in _SPATIAL_SELECTORS:
        return None

    areas = np.array([int(m.sum()) for m in masks], dtype=np.float64)
    if sel == "largest":
        return int(np.argmax(areas))
    if sel == "smallest":
        # Ignore empty masks (area 0) - they are not real detections.
        areas_safe = np.where(areas > 0, areas, np.inf)
        return int(np.argmin(areas_safe))

    rows = np.zeros(len(masks))
    cols = np.zeros(len(masks))
    for i, m in enumerate(masks):
        ys, xs = np.nonzero(m)
        if len(ys):
            rows[i], cols[i] = ys.mean(), xs.mean()
    if sel == "left":
        return int(np.argmin(cols))
    if sel == "right":
        return int(np.argmax(cols))
    if sel in ("near", "bottom"):
        return int(np.argmax(rows))
    if sel in ("far", "top"):
        return int(np.argmin(rows))
    return None


class PerceptionServer:
    """Loads SAM2 + GroundingDINO once, then serves detections."""

    def __init__(self, device: str = "cuda", box_threshold: float = 0.35,
                 text_threshold: float = 0.25) -> None:
        self.device = device
        self.box_threshold = box_threshold
        self.text_threshold = text_threshold

        logger.info("Loading SAM2 + GroundingDINO (this is the ~10s we pay once)...")
        t0 = time.time()
        from sam2.build_sam import build_sam2
        from sam2.sam2_image_predictor import SAM2ImagePredictor
        from groundingdino.util.inference import load_model

        sam2_model = build_sam2(
            detect_object.SAM2_MODEL_CONFIG,
            str(detect_object.SAM2_CHECKPOINT),
            device=device,
        )
        self.sam2 = SAM2ImagePredictor(sam2_model)
        self.dino = load_model(
            model_config_path=str(detect_object.GROUNDING_DINO_CONFIG),
            model_checkpoint_path=str(detect_object.GROUNDING_DINO_CHECKPOINT),
            device=device,
        )
        logger.info("Models loaded in %.1fs — ready.", time.time() - t0)

    def detect(self, rgb: np.ndarray, prompt: str):
        """Run DINO + SAM2 on an in-memory image.

        Mirrors detect_object.run_detection(), but against the already-loaded
        models and an array rather than a path. Kept structurally identical so
        the two cannot drift apart silently.

        Returns:
            (masks, confidences, labels) or (None, None, None) if nothing
            scored above box_threshold.
        """
        import torch
        from torchvision.ops import box_convert
        from groundingdino.util.inference import predict
        import groundingdino.datasets.transforms as T
        from PIL import Image

        # load_image() takes a path; replicate its transform on an array so the
        # preprocessing matches what the CLI produces exactly.
        transform = T.Compose([
            T.RandomResize([800], max_size=1333),
            T.ToTensor(),
            T.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
        ])
        image_tensor, _ = transform(Image.fromarray(rgb), None)

        self.sam2.set_image(rgb)

        boxes, confidences, labels = predict(
            model=self.dino,
            image=image_tensor,
            caption=prompt,
            box_threshold=self.box_threshold,
            text_threshold=self.text_threshold,
            device=self.device,
        )
        if len(boxes) == 0:
            return None, None, None

        h, w, _ = rgb.shape
        boxes = boxes * torch.Tensor([w, h, w, h])
        input_boxes = box_convert(boxes=boxes, in_fmt="cxcywh", out_fmt="xyxy").numpy()

        if self.device == "cuda" and torch.cuda.get_device_properties(0).major >= 8:
            torch.backends.cuda.matmul.allow_tf32 = True
            torch.backends.cudnn.allow_tf32 = True

        with torch.autocast(device_type=self.device, dtype=torch.bfloat16):
            masks, _scores, _logits = self.sam2.predict(
                point_coords=None, point_labels=None,
                box=input_boxes, multimask_output=False,
            )
        if masks.ndim == 4:
            masks = masks.squeeze(1)

        return masks.astype(bool), confidences.numpy().tolist(), list(labels)

    def handle(self, req: dict) -> dict:
        """One request -> one reply. Never raises; every path returns a reason."""
        try:
            prompt = req.get("prompt", "").strip()
            if not prompt:
                return {"ok": False, "reason": "empty prompt: no target object named"}
            # GroundingDINO expects phrases terminated by a period.
            if not prompt.endswith("."):
                prompt += "."

            rgb = _npy_load(req["rgb"])
            depth = _npy_load(req["depth"])
            # mask_to_pointcloud.backproject() indexes K as a DICT
            # (K["fx"], K["cx"], ...), not a 3x3 matrix - see raw_cloud().
            # Passing a matrix raises IndexError deep inside back-projection.
            # intrinsics.json on disk is already in this flat form.
            K = req["intrinsics"]
            missing = {"fx", "fy", "cx", "cy"} - set(K)
            if missing:
                return {"ok": False, "reason":
                        f"intrinsics missing {sorted(missing)}; need flat "
                        f"fx/fy/cx/cy as in extracted/*/intrinsics.json"}

            masks, confs, labels = self.detect(rgb, prompt)
            if masks is None:
                return {"ok": False, "reason":
                        f"no object matched {prompt!r} above "
                        f"{self.box_threshold} confidence"}

            order = np.argsort(confs)[::-1]
            best = int(order[0])

            # Spatial disambiguation. When the scene holds several instances
            # of the same object (three wooden cubes), the detector scores
            # them near-equally and the ambiguity guard below would refuse
            # every one. If the caller passed a `disambiguate` selector
            # ("left"/"right"/"near"/"far"/"top"/"bottom"/"largest"/
            # "smallest"), resolve BY GEOMETRY instead of by score - this is
            # the planner saying which of the identical objects it means.
            disambiguate = str(req.get("disambiguate", "")).strip().lower()
            if disambiguate and len(confs) > 1:
                picked = _select_spatial(disambiguate, masks)
                if picked is None:
                    return {"ok": False, "reason":
                            f"disambiguate={disambiguate!r} is not one of "
                            f"{sorted(_SPATIAL_SELECTORS)}"}
                best = picked
                logger.info("disambiguate=%r picked box %d of %d for %r",
                            disambiguate, best, len(confs), prompt)

            # Ambiguity guard. Two similar scores mean the detector cannot tell
            # the objects apart; picking argmax here is how the e-stop gets
            # grasped. Name BOTH candidates so the VLM can disambiguate.
            elif len(order) > 1:
                margin = confs[best] - confs[int(order[1])]
                if margin < AMBIGUITY_MARGIN:
                    return {"ok": False, "reason":
                            f"ambiguous detection for {prompt!r}: "
                            f"{labels[best]!r} ({confs[best]:.2f}) vs "
                            f"{labels[int(order[1])]!r} ({confs[int(order[1])]:.2f}), "
                            f"margin {margin:.2f} < {AMBIGUITY_MARGIN}. If the scene "
                            f"has several of the same object, pass a disambiguate "
                            f"selector ({sorted(_SPATIAL_SELECTORS)})"}

            mask = masks[best]

            # DEBUG SAVE (opt-in via PRAGMABOT_DEBUG_SAVE=1). Off by default -
            # this is purely to let a human inspect whether the mask is
            # symmetric around the real object or biased to one side (e.g.
            # a shadow/highlight on a glossy object eating into one face's
            # pixels), which would bias the computed centroid/extent used
            # for grasp centering independent of any robot/camera extrinsic
            # calibration. Saved under /tmp/pragmabot_debug so it survives
            # across runs without polluting the repo.
            if os.environ.get("PRAGMABOT_DEBUG_SAVE"):
                try:
                    from PIL import Image
                    dbg_dir = Path("/tmp/pragmabot_debug")
                    dbg_dir.mkdir(exist_ok=True)
                    ts = time.strftime("%Y%m%d_%H%M%S")
                    Image.fromarray(rgb).save(dbg_dir / f"{ts}_rgb.png")
                    np.save(dbg_dir / f"{ts}_mask.npy", mask)
                    logger.info(f"debug: saved rgb+mask to {dbg_dir}/{ts}_*")
                except Exception as exc:  # noqa: BLE001 - diagnostics only
                    logger.warning(f"debug save failed: {exc}")

            frac = float(mask.sum()) / mask.size
            # Callers resolving a placement SURFACE pass the looser bound;
            # everything else keeps the object bound. Same guard, bound
            # chosen by what is being looked for.
            max_frac = float(req.get("max_mask_frac", MAX_MASK_FRAC))
            min_frac = float(req.get("min_mask_frac", MIN_MASK_FRAC))
            if not (min_frac <= frac <= max_frac):
                return {"ok": False, "reason":
                        f"implausible mask for {labels[best]!r}: covers "
                        f"{frac * 100:.2f}% of the image (expected "
                        f"{min_frac * 100:.1f}-{max_frac * 100:.0f}%)"}

            # Same filtering the CLI applies: 3px erosion drops the flying
            # pixels at the silhouette, MAD drops one-sided depth outliers.
            # Measured effect on our cube: z-spread 16.7cm -> 4.5cm.
            xyz = m2p.backproject(
                depth=depth, mask=mask, K=K,
                max_range=req.get("max_range", 3.0),
                erode_px=req.get("erode_px", 3),
                z_outlier_mad=req.get("z_outlier_mad", 3.5),
                aabb_percentile=req.get("aabb_percentile", 2.0),
                aabb_pad_frac=req.get("aabb_pad_frac", 0.05),
                aabb_pad_abs=req.get("aabb_pad_abs", 0.0),
                target_points=req.get("target_points", 2000),
            )

            if len(xyz) < MIN_POINTS:
                return {"ok": False, "reason":
                        f"depth failed on {labels[best]!r}: only {len(xyz)} "
                        f"points survived filtering (need {MIN_POINTS}). The "
                        f"object may be reflective, transparent, or out of range"}

            reply = {"ok": True, "points": _npy_dump(xyz.astype(np.float32)),
                     "n_points": int(len(xyz)),
                     "confidence": float(confs[best]), "label": str(labels[best])}
            if req.get("return_mask"):
                # The UNERODED mask: the caller applies its own interior
                # margin, which for a placement surface is much larger than
                # the 3px flying-pixel erosion used for the cloud above.
                reply["mask"] = _npy_dump(mask)
            return reply

        except Exception as exc:  # noqa: BLE001 - must never kill the server
            logger.exception("request failed")
            return {"ok": False, "reason": f"perception error: {type(exc).__name__}: {exc}"}


def _npy_dump(arr: np.ndarray) -> bytes:
    buf = io.BytesIO()
    np.save(buf, arr, allow_pickle=False)
    return buf.getvalue()


def _npy_load(raw: bytes) -> np.ndarray:
    return np.load(io.BytesIO(raw), allow_pickle=False)


def serve_forever(server: PerceptionServer, host: str, port: int) -> None:
    import msgpack
    import zmq

    ctx = zmq.Context()
    sock = ctx.socket(zmq.REP)
    sock.bind(f"tcp://{host}:{port}")
    logger.info("Listening on tcp://%s:%d", host, port)

    while True:
        req = msgpack.unpackb(sock.recv(), raw=False)
        t0 = time.time()
        reply = server.handle(req)
        logger.info("%s in %.2fs%s",
                    "OK" if reply["ok"] else "REFUSED",
                    time.time() - t0,
                    "" if reply["ok"] else f" - {reply['reason']}")
        sock.send(msgpack.packb(reply, use_bin_type=True))


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", default="127.0.0.1",
                   help="bind address (default: localhost only)")
    p.add_argument("--port", type=int, default=5557,
                   help="default 5557; GraspGen's server uses 5556")
    p.add_argument("--device", default="cuda")
    p.add_argument("--box-threshold", type=float, default=0.35)
    p.add_argument("--text-threshold", type=float, default=0.25)
    args = p.parse_args()

    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")

    server = PerceptionServer(device=args.device,
                              box_threshold=args.box_threshold,
                              text_threshold=args.text_threshold)
    serve_forever(server, args.host, args.port)


if __name__ == "__main__":
    main()
