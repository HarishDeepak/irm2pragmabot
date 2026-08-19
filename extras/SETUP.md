# IRM2 / pragmabot — laptop setup from these archives

Built on Alonnisos (lab machine, RTX 4080, compute capability 8.9) on
2026-08-14. These archives contain code, data, and model checkpoints —
**not** the Python venvs themselves, since those have CUDA extensions
compiled against Alonnisos's specific GPU and won't run on a different
one. You rebuild the venvs fresh here, against your own GPU.

## 0. Before anything else: find your GPU's compute capability

```bash
nvidia-smi --query-gpu=name,compute_cap --format=csv
```

Every `TORCH_CUDA_ARCH_LIST="8.9"` below is Alonnisos's value — replace
`8.9` with whatever your laptop's command prints. Get this right before
building anything; a wrong value either fails to compile or silently
builds for the wrong architecture.

## 1. Extract the archives

```bash
mkdir -p ~/pragmabot ~/GraspGen ~/groundedsam ~/ros2_ws
tar xzf pragmabot_full.tar.gz -C ~/                          # -> ~/pragmabot
tar xzf graspgen_models.tar.gz -C ~/GraspGen/                # -> ~/GraspGen/GraspGenModels
tar xzf groundedsam_checkpoints.tar.gz -C ~/groundedsam/     # needs Grounded-SAM-2/ cloned first, see step 3
tar xzf franka_ros2_src.tar.gz -C ~/ros2_ws/                 # -> ~/ros2_ws/franka_ros2
```

`pragmabot_full.tar.gz` includes the raw rosbag recordings (`bags/`) —
that's most of its 629M.

## 2. GraspGen

The code itself wasn't archived (only `GraspGenModels/`, already
extracted above) — clone it fresh:

```bash
git clone https://github.com/NVlabs/GraspGen.git ~/GraspGen_repo
# merge: put GraspGenModels/ (already extracted) inside the cloned repo,
# or clone directly into ~/GraspGen if that directory is otherwise empty
# after step 1 — check for conflicts before overwriting anything.

cd ~/GraspGen
uv venv --python 3.10 .venv
source .venv/bin/activate
uv pip install torch==2.1.0 torchvision==0.16.0 --index-url https://download.pytorch.org/whl/cu121
uv pip install -e .   # editable install, grasp_gen package
export TORCH_CUDA_ARCH_LIST="<your compute_cap>"   # only if any custom CUDA op needs building
```

`GraspGenModels/` in this archive only has `franka_panda` gripper
checkpoints (Robotiq/suction removed as unneeded) and
`sample_data/{meshes,real_object_pc}` — matches what's actually used in
this project.

## 3. GroundedSAM (Grounded-SAM-2)

Also code-not-archived — clone fresh, then drop the checkpoint archive in:

```bash
git clone https://github.com/IDEA-Research/Grounded-SAM-2.git ~/groundedsam/Grounded-SAM-2
tar xzf groundedsam_checkpoints.tar.gz -C ~/groundedsam/   # -> Grounded-SAM-2/{checkpoints,gdino_checkpoints}

cd ~/groundedsam
uv venv --python 3.10 .venv
source .venv/bin/activate

export CUDA_HOME=/usr/local/cuda
export PATH=/usr/local/cuda/bin:$PATH
export TORCH_CUDA_ARCH_LIST="<your compute_cap>"

uv pip install torch>=2.3.1 torchvision>=0.18.1 --index-url https://download.pytorch.org/whl/cu121
uv pip install --no-build-isolation -e ".[notebooks]"
uv pip install --no-build-isolation -e grounding_dino

# must-pin, not auto-installed correctly otherwise:
uv pip install "transformers<5"    # transformers 5.x removed BertModel.get_head_mask,
                                    # which GroundingDINO's BertModelWarper still calls
uv pip install addict yapf timm supervision pycocotools   # runtime deps, not in either
                                                            # package's own setup.py
```

**`--no-build-isolation` is required** on both `-e` installs — without
it, uv's build isolation pulls a fresh unrelated torch into a temp env
and the CUDA-version-mismatch check fails against nvcc.

Verify: `python grounded_sam2_local_demo.py` on the repo's own
`notebooks/images/truck.jpg` should produce correct boxes/masks in
`outputs/`.

## 4. pragmabot repo

Already extracted in step 1 (`~/pragmabot`). Its own scripts
(`calibration/*.py`) don't need a dedicated venv — `mask_to_pointcloud.py`
etc. only need numpy, runs fine under system python or either of the
venvs above. `detect_object.py` specifically needs the GroundedSAM venv
(`~/groundedsam/.venv/bin/python`).

## 5. franka_ros2 (robot control container) — only matters near the robot

```bash
cd ~/ros2_ws/franka_ros2
docker compose build   # or docker compose up, per its own docker-compose.yml
```

**Flagging clearly: this container's whole job is talking to the real
FR3 over the lab network** (`robot_ip:=10.10.10.10`, FCI). From home,
with no VPN into the lab network, it can build and run, but will never
successfully connect to the robot. This step is worth doing only once
you're either physically on the lab network again or have a VPN set up
(not something this session looked into).

No GPU/CUDA needed for this one — it's pure robot control, not vision.

## 6. FoundationPose

Not included — per CLAUDE.md, weights were never downloaded on Alonnisos
either (env existed with no packages installed). Nothing to transfer;
set up fresh if/when actually needed, following the same
`TORCH_CUDA_ARCH_LIST` pattern as above (PyTorch3D + NVDiffRast built
from source, python 3.9).

## Sanity check once everything's built

```bash
source ~/GraspGen/.venv/bin/activate
python3 ~/GraspGen/client-server/graspgen_server.py \
    --gripper_config ~/GraspGen/GraspGenModels/checkpoints/graspgen_franka_panda.yml &
python3 ~/GraspGen/client-server/graspgen_client.py \
    --pcd_file ~/pragmabot/extracted/red_cup/detections/object_pcd.npy \
    --topk_num_grasps 1
```
Should print 6 grasps at confidence ~0.9+, same as it did on Alonnisos —
confirms the rebuilt venv actually works before relying on it for
anything else.
