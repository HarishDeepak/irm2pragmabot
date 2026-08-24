#!/usr/bin/env python3
"""
test_fps.py — standalone test of Farthest-Point Sampling on a mask.

Loads an RGB image and a binary mask (from your existing Grounded-SAM
detection), picks N spread-out candidate points inside the mask, and
draws them as numbered dots on the image. Nothing else — no depth, no
VLM, no robot.

The sampler itself now lives in `mask_sampling.py` so the bridge can call
it too; this file is its CLI and its visual check, unchanged.

USAGE
-----
    python3 test_fps.py \
        --rgb extracted/<scene>/rgb.png \
        --mask extracted/<scene>/detections/mask.npy \
        --n 8 \
        --out fps_check.png

    # also mark the pixel the bridge would actually place on (green X):
    python3 test_fps.py --rgb ... --mask ... --show-placement
"""

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from mask_sampling import farthest_point_sampling, select_placement_pixel


def draw_numbered_candidates(rgb, mask, candidate_pixels, radius=8):
    annotated = rgb.copy()

    contours, _ = cv2.findContours(
        (mask.astype(np.uint8) * 255), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    cv2.drawContours(annotated, contours, -1, (0, 255, 0), 2)

    for i, (u, v) in enumerate(candidate_pixels):
        u, v = int(u), int(v)
        cv2.circle(annotated, (u, v), radius, (0, 0, 255), -1)
        label = str(i + 1)
        cv2.putText(annotated, label, (u + radius + 3, v - radius - 3),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(annotated, label, (u + radius + 3, v - radius - 3),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)

    return annotated


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rgb", required=True, help="path to rgb.png")
    ap.add_argument("--mask", required=True,
                    help="path to a mask file — .npy (bool array) or .png (0/255 image)")
    ap.add_argument("--n", type=int, default=8, help="number of candidate points")
    ap.add_argument("--out", default="fps_check.png", help="output image path")
    ap.add_argument("--show-placement", action="store_true",
                    help="also mark the pixel select_placement_pixel() would "
                         "choose (green X) — the one the bridge actually places on")
    args = ap.parse_args()

    rgb = cv2.imread(args.rgb)
    if rgb is None:
        raise SystemExit(f"Could not read {args.rgb}")

    if args.mask.endswith(".npy"):
        mask = np.load(args.mask).astype(bool)
    else:
        mask = cv2.imread(args.mask, cv2.IMREAD_GRAYSCALE) > 127

    if mask.shape != rgb.shape[:2]:
        print(f"Resizing mask {mask.shape} -> rgb {rgb.shape[:2]}")
        mask = cv2.resize(mask.astype(np.uint8), (rgb.shape[1], rgb.shape[0]),
                          interpolation=cv2.INTER_NEAREST).astype(bool)

    candidates = farthest_point_sampling(mask, n_points=args.n)
    print(f"Selected {len(candidates)} candidate points:")
    for i, (u, v) in enumerate(candidates):
        print(f"  [{i+1}] pixel (u={u}, v={v})")

    annotated = draw_numbered_candidates(rgb, mask, candidates)

    if args.show_placement:
        u, v, info = select_placement_pixel(mask)
        print(f"\nplacement pixel: (u={u}, v={v}) "
              f"{info['interior_px']} px from the mask edge "
              f"(margin enforced {info['margin_px']} px"
              f"{', RELAXED' if info['relaxed'] else ''})")
        cv2.drawMarker(annotated, (u, v), (0, 255, 0), cv2.MARKER_TILTED_CROSS,
                       28, 3, cv2.LINE_AA)

    cv2.imwrite(args.out, annotated)
    print(f"\nWrote {args.out}")


if __name__ == "__main__":
    main()