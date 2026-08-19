#!/usr/bin/env bash
# setup.sh -- fetch model checkpoints from their official sources (~2.5 GB).
# Run once after cloning. Safe to re-run; downloads resume/skip.
#
# What this does NOT do: build the Python environments (see SETUP.md) or
# fetch the recorded rosbag (no public source -- ask Harish).

set -euo pipefail
cd "$(dirname "$0")"

echo "=============================================="
echo " IRM2 / PragmaBot -- checkpoint download"
echo "=============================================="
echo

# ---------------------------------------------------------------- GraspGen
if [ -f GraspGen/GraspGenModels/checkpoints/graspgen_franka_panda_gen.pth ]; then
  echo "[1/3] GraspGen checkpoints .......... already present, skipping"
else
  echo "[1/3] GraspGen checkpoints (~1 GB)"
  python3 -m pip install -q --upgrade huggingface_hub
  huggingface-cli download adithyamurali/GraspGenModels \
      --local-dir GraspGen/GraspGenModels
fi
echo

# ------------------------------------------------------------------- SAM 2.1
if [ -f groundedsam/Grounded-SAM-2/checkpoints/sam2.1_hiera_large.pt ]; then
  echo "[2/3] SAM 2.1 checkpoint ............ already present, skipping"
else
  echo "[2/3] SAM 2.1 checkpoint (~857 MB)"
  ( cd groundedsam/Grounded-SAM-2/checkpoints && bash download_ckpts.sh )
fi
echo

# -------------------------------------------------------------- GroundingDINO
if [ -f groundedsam/Grounded-SAM-2/gdino_checkpoints/groundingdino_swint_ogc.pth ]; then
  echo "[3/3] GroundingDINO checkpoint ...... already present, skipping"
else
  echo "[3/3] GroundingDINO checkpoint (~662 MB)"
  ( cd groundedsam/Grounded-SAM-2/gdino_checkpoints && bash download_ckpts.sh )
fi
echo

echo "=============================================="
echo " Checkpoints done."
echo "=============================================="
cat <<'NEXT'

Next steps:

  1. Find your GPU's compute capability -- you need it before building anything:

         nvidia-smi --query-gpu=name,compute_cap --format=csv

     Every TORCH_CUDA_ARCH_LIST="8.9" in SETUP.md is the lab machine's value.
     Replace it with yours.

  2. Build the TWO virtual environments per SETUP.md.
     They CANNOT be merged -- GraspGen pins torch==2.1.0, Grounded-SAM-2
     needs torch>=2.3.1.

  3. Recorded rosbag (832 MB) has no public source. Ask Harish for it, then:

         pragmabot/bags/red_cup/red_cup_0.db3
         pragmabot/bags/red_cup/metadata.yaml

     You do NOT need it to get started -- pragmabot/extracted/ is committed
     and already contains two processed scenes.

  4. Sanity check once the GraspGen venv is built:

         source ~/GraspGen/.venv/bin/activate
         python3 GraspGen/client-server/graspgen_server.py \
             --gripper_config GraspGen/GraspGenModels/checkpoints/graspgen_franka_panda.yml &
         python3 GraspGen/client-server/graspgen_client.py \
             --pcd_file pragmabot/extracted/red_cup/detections/object_pcd.npy

     Expect ~6 grasps at confidence 0.9+.

NEXT
