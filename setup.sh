#!/usr/bin/env bash
# setup.sh -- fetch model checkpoints from their official sources (~2.5 GB).
#
# Safe to re-run. Each file is checked against its exact expected byte size:
#   - correct size  -> skipped
#   - partial       -> resumed (not skipped, not restarted)
#   - missing       -> downloaded
#
# Uses curl only. No python, no pip, no wget -- so it works in Git Bash on
# Windows, where /usr/bin/python3 has no pip and wget is not installed.
#
# NOT handled here (no public source):
#   - the recorded rosbag             -> ask Harish
#   - the hand-eye calibration result -> from the lab machine

set -eu
cd "$(dirname "$0")"

if ! command -v curl >/dev/null 2>&1; then
  echo "ERROR: curl not found. Install curl, or download the four files listed"
  echo "       at the bottom of this script by hand."
  exit 1
fi

HF="https://huggingface.co/adithyamurali/GraspGenModels/resolve/main"
SAM2="https://dl.fbaipublicfiles.com/segment_anything_2/092824"
GDINO="https://github.com/IDEA-Research/GroundingDINO/releases/download/v0.1.0-alpha"

# url | destination | exact expected size in bytes
FILES="
$HF/checkpoints/graspgen_franka_panda_gen.pth|GraspGen/GraspGenModels/checkpoints/graspgen_franka_panda_gen.pth|907408223
$HF/checkpoints/graspgen_franka_panda_dis.pth|GraspGen/GraspGenModels/checkpoints/graspgen_franka_panda_dis.pth|165853892
$SAM2/sam2.1_hiera_large.pt|groundedsam/Grounded-SAM-2/checkpoints/sam2.1_hiera_large.pt|898083611
$GDINO/groundingdino_swint_ogc.pth|groundedsam/Grounded-SAM-2/gdino_checkpoints/groundingdino_swint_ogc.pth|693997677
"

filesize() { wc -c < "$1" 2>/dev/null | tr -d ' ' || echo 0; }
mb() { echo $(( $1 / 1048576 )); }

echo "=============================================="
echo " IRM2 / PragmaBot -- checkpoint download"
echo "=============================================="
echo

n=0
echo "$FILES" | while IFS='|' read -r url dest want; do
  [ -z "${url:-}" ] && continue
  n=$((n+1))
  name="$(basename "$dest")"
  mkdir -p "$(dirname "$dest")"

  have=0
  [ -f "$dest" ] && have=$(filesize "$dest")

  if [ "$have" = "$want" ]; then
    echo "[$n/4] $name ... complete ($(mb "$want") MB), skipping"
    continue
  fi

  if [ "$have" -gt 0 ] 2>/dev/null; then
    echo "[$n/4] $name ... partial ($(mb "$have")/$(mb "$want") MB), resuming"
  else
    echo "[$n/4] $name ... downloading ($(mb "$want") MB)"
  fi

  # -C - resume, -L follow redirects, -f fail on HTTP error, --retry transient
  curl -fL -C - --retry 3 --retry-delay 2 --progress-bar -o "$dest" "$url" || {
    echo "      download error -- re-run this script to resume"
  }
done

echo
echo "=============================================="
echo " Verifying exact sizes"
echo "=============================================="
fail=0
echo "$FILES" | while IFS='|' read -r url dest want; do
  [ -z "${url:-}" ] && continue
  name="$(basename "$dest")"
  have=0; [ -f "$dest" ] && have=$(filesize "$dest")
  if [ "$have" = "$want" ]; then
    printf "  OK        %-42s %s MB\n" "$name" "$(mb "$want")"
  elif [ "$have" -gt 0 ] 2>/dev/null; then
    printf "  PARTIAL   %-42s %s/%s MB\n" "$name" "$(mb "$have")" "$(mb "$want")"
    echo "$name" >> .setup_failed
  else
    printf "  MISSING   %-42s\n" "$name"
    echo "$name" >> .setup_failed
  fi
done

if [ -f .setup_failed ]; then
  rm -f .setup_failed
  echo
  echo "Some checkpoints are incomplete. Re-run this script -- it resumes"
  echo "from where it stopped, it does not start over."
  exit 1
fi

cat <<'NEXT'

==============================================
 All checkpoints verified. Next steps:
==============================================

  1. Find your GPU's compute capability:

         nvidia-smi --query-gpu=name,compute_cap --format=csv

     SETUP.md uses 8.9 (the lab machine's RTX 4080). Replace it with yours.

  2. Build the TWO virtual environments -- SETUP.md sections 2 and 3.
     They CANNOT be merged: GraspGen pins torch==2.1.0, Grounded-SAM-2
     needs torch>=2.3.1.

  3. Sanity check (SETUP.md section 7): expect ~6 grasps at confidence
     0.92-0.96 on pragmabot/extracted/red_cup/.

  4. Rosbag (832 MB) has no public source. Ask Harish, then place at:
         pragmabot/bags/red_cup/red_cup_0.db3
     You do NOT need it to start -- pragmabot/extracted/ already has two
     fully processed scenes.

NEXT
