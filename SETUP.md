# SETUP — building the environments

For the **monorepo layout**. All paths are relative to the repo root, so run
everything from inside your clone.

> The old lab-transfer guide (tar.gz archives, separate `~/pragmabot`,
> `~/GraspGen`, `~/groundedsam` home directories) is preserved at
> `extras/SETUP.lab-original.md`. **Do not follow it** — GraspGen and
> Grounded-SAM-2 are already vendored here, so cloning them again produces a
> duplicate, broken tree.

---

## 0. Before anything: your GPU's compute capability

```bash
nvidia-smi --query-gpu=name,compute_cap --format=csv
```

Every `TORCH_CUDA_ARCH_LIST="8.9"` below is **Alonnisos's** value (RTX 4080).
Replace `8.9` with whatever your card prints. Getting this wrong either fails to
compile or silently builds for the wrong architecture.

## 1. Clone and fetch checkpoints

```bash
git clone https://github.com/HarishDeepak/irm2pragmabot.git
cd irm2pragmabot
bash setup.sh          # ~2.5 GB, ~20 min
```

`setup.sh` downloads the four model checkpoints from their official sources
(HuggingFace, Meta, IDEA-Research). Nothing else is needed for them.

## 2. GraspGen venv

```bash
cd GraspGen
uv venv --python 3.10 .venv
source .venv/bin/activate
uv pip install torch==2.1.0 torchvision==0.16.0 --index-url https://download.pytorch.org/whl/cu121
uv pip install -e .
uv pip install "numpy<2"                # REQUIRED: numpy 2.x breaks torch 2.1.0
export TORCH_CUDA_ARCH_LIST="8.9"      # your value; only needed if a CUDA op builds
cd ..
```

## 3. Grounded-SAM-2 venv — **separate, cannot be merged with GraspGen's**

GraspGen pins `torch==2.1.0`; Grounded-SAM-2 needs `>=2.3.1`. One environment
cannot satisfy both. This is why the project uses per-tool venvs rather than
containers.

```bash
cd groundedsam
uv venv --python 3.10 .venv
source .venv/bin/activate

export CUDA_HOME=/usr/local/cuda
export PATH=/usr/local/cuda/bin:$PATH      # nvcc is not on PATH by default
export TORCH_CUDA_ARCH_LIST="8.9"          # your value

uv pip install torch>=2.3.1 torchvision>=0.18.1 --index-url https://download.pytorch.org/whl/cu121

cd Grounded-SAM-2
uv pip install --no-build-isolation -e ".[notebooks]"
uv pip install --no-build-isolation -e grounding_dino

uv pip install "transformers<5"
uv pip install addict yapf timm supervision pycocotools
cd ../..
```

### Three things that will bite you

1. **`--no-build-isolation` is mandatory** on both editable installs. Without it,
   uv pulls a fresh unrelated torch into a temp env and the CUDA-version check
   fails against nvcc.
2. **Pin `transformers<5`.** v5 removed `BertModel.get_head_mask`, which
   GroundingDINO's `BertModelWarper` still calls — you get
   `AttributeError: 'BertModel' object has no attribute 'get_head_mask'`.
3. **`addict yapf timm supervision pycocotools`** are runtime deps that neither
   package's `setup.py` installs.

Verify:
```bash
cd groundedsam/Grounded-SAM-2
python grounded_sam2_local_demo.py     # correct boxes/masks in outputs/
cd ../..
```

## 4. Calibration scripts — no dedicated venv

`pragmabot/calibration/mask_to_pointcloud.py` needs only numpy and runs under
system python or either venv.

`detect_object.py` is the exception — it needs the Grounded-SAM-2 venv:

```bash
groundedsam/.venv/bin/python pragmabot/calibration/detect_object.py \
    --rgb pragmabot/extracted/red_cup/rgb.png --prompt "red cup."
```

## 5. Control container — only matters near the robot

```bash
cd ros2_ws/franka_ros2
cp .env.example .env
sed -i "s/^USER_UID=.*/USER_UID=$(id -u)/; s/^USER_GID=.*/USER_GID=$(id -g)/" .env
docker compose build
docker compose up -d
cd ../..
```

No GPU needed — this is pure robot control, not vision.

> Its whole job is talking to the FR3 over the lab network (`robot_ip:=10.10.10.10`,
> FCI). From home without a VPN it builds and runs but will never connect.

## 6. ZED workspace — host-side, needs ROS 2 Humble

```bash
export ROS_DOMAIN_ID=7                 # required on EVERY terminal
source /opt/ros/humble/setup.bash
cd zed_ros2_ws && colcon build --symlink-install && source install/setup.bash && cd ..
```

Builds `zed_wrapper`, `aruco_detector` (needed for hand-eye calibration) and
`easy_handeye2`.

## 7. Sanity check — do this before trusting anything

```bash
source GraspGen/.venv/bin/activate
python3 GraspGen/client-server/graspgen_server.py \
    --gripper_config GraspGen/GraspGenModels/checkpoints/graspgen_franka_panda.yml &
sleep 8
python3 GraspGen/client-server/graspgen_client.py \
    --pcd_file pragmabot/extracted/red_cup/detections/object_pcd.npy
```

**Expect ~6 grasps at confidence 0.92–0.96.** That matches the committed
`pragmabot/extracted/red_cup/grasps.npz`, so if you get something very different
the environment is wrong, not the data.

## What is not in the repo

| Item | Size | How to get it |
|---|---|---|
| Model checkpoints | ~2.5 GB | `bash setup.sh` |
| `red_cup_0.db3` rosbag | 832 MB | ask Harish → `pragmabot/bags/red_cup/` |
| Hand-eye calibration result | few KB | from the lab machine — **not yet copied off it** |
| Python venvs | ~12 GB | built above; CUDA extensions are GPU-specific, never copy them |

**You do not need the rosbag to start.** `pragmabot/extracted/` holds two fully
processed scenes (RGB, depth, intrinsics, masks, point clouds, grasps).

## Open the workspace

```bash
code irm2.code-workspace
```

Six folders, one window, five terminal profiles (Control-in-docker, ZED-on-host,
GraspGen venv, GroundedSAM venv, Bridge/Planner).

## Where to read next

`extras/analysis/05_HANDOFF.md` — project state, verified findings, next action.

---

## Honest status of this guide

The **paths and layout** here are verified against a fresh clone. The **venv
build steps** are carried over from the lab machine and have *not* been re-run
end to end on a second machine — if a step fails, that is the likely place.
Report what breaks and it gets corrected here.
