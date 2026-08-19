#!/usr/bin/env python3
"""
Offline test: run GroundedSAM on a saved image.
Usage: python3 test_grounded_sam.py --image /path/to/image.png --text "red cup"
"""
import argparse
import cv2
import numpy as np
import sys
sys.path.insert(0, "/catkin_ws/src/pragmabot/pragmabot/src")

from pragmabot.grounded_sam import GroundedSAM

parser = argparse.ArgumentParser()
parser.add_argument("--image", required=True)
parser.add_argument("--text",  required=True)
args = parser.parse_args()

img = cv2.imread(args.image)
assert img is not None, f"Could not load image: {args.image}"

print(f"Image shape: {img.shape}")
print("Loading GroundedSAM (may take ~30s on CPU)...")
gsam = GroundedSAM()
print(f"Running on: {gsam.device}")

mask, conf = gsam.segment(img, args.text)

print(f"Confidence: {conf:.3f}")
print(f"Mask pixels: {mask.sum()} / {mask.size} ({100*mask.mean():.1f}%)")

vis = img.copy()
vis[mask] = (vis[mask] * 0.4 + np.array([0, 200, 0]) * 0.6).astype(np.uint8)
out_path = "/tmp/gsam_result.png"
cv2.imwrite(out_path, vis)
print(f"Saved visualisation to {out_path}")