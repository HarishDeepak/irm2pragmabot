#!/usr/bin/env bash
# setup.sh -- fetch model checkpoints from their official sources (~2.5 GB).
# Run once after cloning. Safe to re-run: finished files are skipped and
# partial downloads resume.
#
# Uses curl only. No python, no pip, no wget -- so it works in Git Bash on
# Windows, where /usr/bin/python3 has no pip and wget is not installed.
#
# NOT handled here (no public source):
#   - the recorded rosbag  -> ask Harish
#   - the hand-eye calibration result -> from the lab machine

set -eu
cd "$(dirname "$0")"

if ! command -v curl >/dev/null 2>&1; then
  echo "ERROR: curl not found. Install curl, or download the four files listed"
  echo "       at the bottom of this script by hand."
  exit 1
fi

# fetch <url> <destination-path> <human-readable-size>
fetch() {
  url="$1"; dest="$2"; size="$3"
  name="$(basename "$dest")"
  if [ -f "$dest" ]; then
    echo "    $name ... already present, skipping"
    return 0
  fi
  mkdir -p "$(dirname "$dest")"
  echo "    $name ($size)"
  # -L follow redirects (HuggingFace 302s to a CDN), -C - resume, -f fail on HTTP error
  if ! curl -fL -C - --retry 3 --retry-delay 2 -o "$dest" "$url"; then
    echo "    FAILED: $url"
    rm -f "$dest"
    return 1
  fi
}

echo "=============================================="
echo " IRM2 / PragmaBot -- checkpoint download"
echo "=============================================="
echo

HF="https://huggingface.co/adithyamurali/GraspGenModels/resolve/main"
SAM2="https://dl.fbaipublicfiles.com/segment_anything_2/092824"
GDINO="https://github.com/IDEA-Research/GroundingDINO/releases/download/v0.1.0-alpha"

echo "[1/3] GraspGen checkpoints (~1 GB)"
fetch "$HF/checkpoints/graspgen_franka_panda_gen.pth" \
      "GraspGen/GraspGenModels/checkpoints/graspgen_franka_panda_gen.pth" "866 MB"
fetch "$HF/checkpoints/graspgen_franka_panda_dis.pth" \
      "GraspGen/GraspGenModels/checkpoints/graspgen_franka_panda_dis.pth" "159 MB"
echo

echo "[2/3] SAM 2.1 checkpoint"
fetch "$SAM2/sam2.1_hiera_large.pt" \
      "groundedsam/Grounded-SAM-2/checkpoints/sam2.1_hiera_large.pt" "857 MB"
echo

echo "[3/3] GroundingDINO checkpoint"
fetch "$GDINO/groundingdino_swint_ogc.pth" \
      "groundedsam/Grounded-SAM-2/gdino_checkpoints/groundingdino_swint_ogc.pth" "662 MB"
echo

echo "=============================================="
echo " Verifying"
echo "=============================================="
ok=1
for f in \
  GraspGen/GraspGenModels/checkpoints/graspgen_franka_panda_gen.pth \
  GraspGen/GraspGenModels/checkpoints/graspgen_franka_panda_dis.pth \
  groundedsam/Grounded-SAM-2/checkpoints/sam2.1_hiera_large.pt \
  groundedsam/Grounded-SAM-2/gdino_checkpoints/groundingdino_swint_ogc.pth ; do
  if [ -f "$f" ]; then
    sz=$(du -m "$f" 2>/dev/null | cut -f1)
    if [ "${sz:-0}" -lt 50 ]; then
      echo "  SUSPICIOUS (${sz}MB, too small): $f"; ok=0
    else
      echo "  OK  ${sz}MB  $f"
    fi
  else
    echo "  MISSING: $f"; ok=0
  fi
done
echo

if [ "$ok" -ne 1 ]; then
  echo "Some checkpoints are missing or truncated. Re-run this script -- it resumes."
  exit 1
fi

cat <<'NEXT'
==============================================
 Checkpoints complete. Next steps:
==============================================

  1. Find your GPU's compute capability:

         nvidia-smi --query-gpu=name,compute_cap --format=csv

     SETUP.md uses 8.9 (the lab machine's RTX 4080). Replace it with yours.

  2. Build the TWO virtual environments -- see SETUP.md sections 2 and 3.
     They CANNOT be merged: GraspGen pins torch==2.1.0, Grounded-SAM-2
     needs torch>=2.3.1.

  3. Sanity check (SETUP.md section 7): expect ~6 grasps at confidence
     0.92-0.96 on pragmabot/extracted/red_cup/.

  4. Rosbag (832 MB) has no public source. Ask Harish, then place at:
         pragmabot/bags/red_cup/red_cup_0.db3
     You do NOT need it to start -- pragmabot/extracted/ already has two
     fully processed scenes.

NEXT
