"""
detect_object.py — text-prompted 2D segmentation via Grounded-SAM-2.

Wraps the standalone Grounded-SAM-2 install at ~/groundedsam/Grounded-SAM-2
(GroundingDINO for open-vocabulary box detection + SAM2 for box-prompted
segmentation) to turn an RGB image + text prompt into a binary mask. This
is the `mask` input that calibrate_extrinsic.py's run_foundationpose_register()
expects — see that file's docstring.

MUST run with Grounded-SAM-2's own venv (separate from every other tool's
venv in this project — see CLAUDE.md):

    ~/groundedsam/.venv/bin/python calibration/detect_object.py --rgb ...

USAGE (against the extracted scene at extracted/scene_single_cup/):

    ~/groundedsam/.venv/bin/python calibration/detect_object.py \\
        --rgb ~/pragmabot/extracted/scene_single_cup/rgb.png \\
        --prompt "cube." \\
        --out-dir ~/pragmabot/extracted/scene_single_cup/detections

Outputs (written to --out-dir, default: alongside --rgb in a "detections/"
subfolder):
  - mask.npy              bool array, shape (H, W) — the best-scoring match
  - annotated.jpg         box + mask overlay, for a quick sanity check
  - detections.json       every match above threshold: class name, bbox
                           (xyxy pixels), confidence score

Text prompts follow GroundingDINO convention: lowercase, each phrase
ending in a period (e.g. "cube." or "yellow cube. red button."). Multiple
phrases can be detected in one call; --select picks what goes into mask.npy.
"""

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np
import pycocotools.mask as mask_util
import supervision as sv
import torch
from torchvision.ops import box_convert

GROUNDED_SAM2_ROOT = Path.home() / "groundedsam" / "Grounded-SAM-2"

# grounding_dino's own inference.py does `import grounding_dino.groundingdino...`
# (a self-reference through the repo-root directory name, not the installed
# package name) — that only resolves if the repo root itself is on sys.path.
sys.path.insert(0, str(GROUNDED_SAM2_ROOT))

from sam2.build_sam import build_sam2  # noqa: E402
from sam2.sam2_image_predictor import SAM2ImagePredictor  # noqa: E402
from groundingdino.util.inference import load_model, load_image, predict  # noqa: E402

SAM2_CHECKPOINT = GROUNDED_SAM2_ROOT / "checkpoints" / "sam2.1_hiera_large.pt"
SAM2_MODEL_CONFIG = "configs/sam2.1/sam2.1_hiera_l.yaml"  # resolved via sam2's hydra module search path, not a filesystem path
GROUNDING_DINO_CONFIG = GROUNDED_SAM2_ROOT / "grounding_dino" / "groundingdino" / "config" / "GroundingDINO_SwinT_OGC.py"
GROUNDING_DINO_CHECKPOINT = GROUNDED_SAM2_ROOT / "gdino_checkpoints" / "groundingdino_swint_ogc.pth"


def run_detection(rgb_path: Path, prompt: str, box_threshold: float,
                   text_threshold: float, device: str):
    sam2_model = build_sam2(SAM2_MODEL_CONFIG, str(SAM2_CHECKPOINT), device=device)
    sam2_predictor = SAM2ImagePredictor(sam2_model)

    grounding_model = load_model(
        model_config_path=str(GROUNDING_DINO_CONFIG),
        model_checkpoint_path=str(GROUNDING_DINO_CHECKPOINT),
        device=device,
    )

    image_source, image = load_image(str(rgb_path))
    sam2_predictor.set_image(image_source)

    boxes, confidences, labels = predict(
        model=grounding_model,
        image=image,
        caption=prompt,
        box_threshold=box_threshold,
        text_threshold=text_threshold,
        device=device,
    )

    if len(boxes) == 0:
        return None, None, None, None

    h, w, _ = image_source.shape
    boxes = boxes * torch.Tensor([w, h, w, h])
    input_boxes = box_convert(boxes=boxes, in_fmt="cxcywh", out_fmt="xyxy").numpy()

    if device == "cuda" and torch.cuda.get_device_properties(0).major >= 8:
        torch.backends.cuda.matmul.allow_tf32 = True
        torch.backends.cudnn.allow_tf32 = True

    with torch.autocast(device_type=device, dtype=torch.bfloat16):
        masks, scores, _logits = sam2_predictor.predict(
            point_coords=None,
            point_labels=None,
            box=input_boxes,
            multimask_output=False,
        )

    if masks.ndim == 4:
        masks = masks.squeeze(1)
    masks = masks.astype(bool)
    confidences = confidences.numpy().tolist()

    return input_boxes, masks, confidences, labels


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--rgb", required=True, help="path to rgb.png (or any RGB image)")
    parser.add_argument("--prompt", required=True,
                         help='GroundingDINO text prompt, e.g. "cube." or "yellow cube. red button."')
    parser.add_argument("--out-dir", default=None,
                         help="default: <rgb's parent dir>/detections")
    parser.add_argument("--select", choices=["best", "all"], default="best",
                         help="best (default): mask.npy = highest-confidence match. "
                              "all: mask.npy = union of every match above threshold.")
    parser.add_argument("--box-threshold", type=float, default=0.35)
    parser.add_argument("--text-threshold", type=float, default=0.25)
    args = parser.parse_args()

    rgb_path = Path(args.rgb).expanduser().resolve()
    if not rgb_path.is_file():
        parser.error(f"--rgb not found: {rgb_path}")

    out_dir = Path(args.out_dir).expanduser().resolve() if args.out_dir else rgb_path.parent / "detections"
    out_dir.mkdir(parents=True, exist_ok=True)

    device = "cuda" if torch.cuda.is_available() else "cpu"

    boxes, masks, confidences, labels = run_detection(
        rgb_path, args.prompt, args.box_threshold, args.text_threshold, device)

    if boxes is None:
        print(f"No matches for prompt {args.prompt!r} above box_threshold={args.box_threshold}. "
              f"Try a lower --box-threshold or a different phrasing.")
        sys.exit(1)

    for label, conf, box in zip(labels, confidences, boxes):
        print(f"  {label:20s} conf={conf:.2f}  bbox(xyxy)={box.round(1).tolist()}")

    if args.select == "best":
        best = int(np.argmax(confidences))
        final_mask = masks[best]
        print(f"-> mask.npy = best match: {labels[best]!r} (conf={confidences[best]:.2f})")
    else:
        final_mask = np.any(masks, axis=0)
        print(f"-> mask.npy = union of all {len(labels)} matches")

    np.save(out_dir / "mask.npy", final_mask)

    class_ids = np.arange(len(labels))
    detections = sv.Detections(xyxy=boxes, mask=masks, class_id=class_ids)
    annot_labels = [f"{name} {conf:.2f}" for name, conf in zip(labels, confidences)]

    img = cv2.imread(str(rgb_path))
    annotated = sv.BoxAnnotator().annotate(scene=img.copy(), detections=detections)
    annotated = sv.LabelAnnotator().annotate(scene=annotated, detections=detections, labels=annot_labels)
    annotated = sv.MaskAnnotator().annotate(scene=annotated, detections=detections)
    cv2.imwrite(str(out_dir / "annotated.jpg"), annotated)

    def mask_to_rle(mask):
        rle = mask_util.encode(np.asfortranarray(mask[:, :, None].astype("uint8")))[0]
        rle["counts"] = rle["counts"].decode("utf-8")
        return rle

    results = {
        "image_path": str(rgb_path),
        "prompt": args.prompt,
        "select": args.select,
        "detections": [
            {"class_name": name, "bbox_xyxy": box.tolist(), "score": conf,
             "segmentation_rle": mask_to_rle(mask)}
            for name, box, conf, mask in zip(labels, boxes, confidences, masks)
        ],
    }
    with open(out_dir / "detections.json", "w") as f:
        json.dump(results, f, indent=2)

    print(f"\nWrote: {out_dir / 'mask.npy'}")
    print(f"       {out_dir / 'annotated.jpg'}")
    print(f"       {out_dir / 'detections.json'}")


if __name__ == "__main__":
    main()
