# SETUP_LAPTOP.md — getting this running on a laptop

Written from an actual clone-and-run on a Windows 11 laptop (RTX 4050) with
WSL2 Ubuntu 24.04. Everything marked **VERIFIED** was executed and observed, not
assumed. Everything marked **NOT VERIFIED** says so plainly.

---

## TL;DR

```bash
git clone https://github.com/HarishDeepak/irm2pragmabot.git
cd irm2pragmabot
bash setup.sh                 # VERIFIED: 2.5 GB, resumable, checks exact sizes
```

Then build the two venvs (§4). **Do this in Linux or WSL, not Git Bash.**

---

## 1. Where to run this

| Environment | Verdict |
|---|---|
| **Linux (Ubuntu 22.04)** | best — what the lab uses, ROS 2 Humble targets it |
| **WSL2 Ubuntu** | works. GPU passes through — **VERIFIED**: `torch.cuda.is_available() == True` |
| **Windows Git Bash** | `setup.sh` only. Cannot build the venvs. |

**Why not Git Bash:** the CUDA extensions (`pointnet2_ops`, `ms_deform_attn`)
need `nvcc` and a Linux toolchain. Git Bash has neither.

> **WSL caveat:** the default WSL image is Ubuntu **24.04**, but ROS 2 Humble
> targets **22.04**. The Python side (GraspGen, Grounded-SAM-2) is fine on 24.04
> — **VERIFIED**. The ROS side is not: install Humble on 22.04, or use the
> Docker container for anything ROS.

## 2. Prerequisites

```bash
nvidia-smi --query-gpu=name,compute_cap --format=csv
```

Note your `compute_cap`. Every `TORCH_CUDA_ARCH_LIST="8.9"` below assumes 8.9.

> **VERIFIED convenience:** an RTX 4050 Laptop reports **8.9** — the same as the
> lab's RTX 4080. If your GPU is 40-series, `8.9` is already correct.

You need `curl` and `git`. You do **not** need Python 3.10 preinstalled —
**VERIFIED**: `uv` downloads CPython 3.10.21 itself.

Install `uv` if missing:
```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
export PATH="$HOME/.local/bin:$PATH"
```

## 3. Checkpoints — **VERIFIED end to end**

```bash
bash setup.sh
```

Observed output:
```
OK  graspgen_franka_panda_gen.pth   865 MB
OK  graspgen_franka_panda_dis.pth   158 MB
OK  sam2.1_hiera_large.pt           856 MB
OK  groundingdino_swint_ogc.pth     661 MB
All checkpoints verified.
```

The script checks each file's **exact byte size**, resumes partials, and skips
completed files. Safe to re-run; a dropped connection does not restart 866 MB.

## 4. The two virtual environments

They **cannot** be merged: GraspGen pins `torch==2.1.0`, Grounded-SAM-2 needs
`torch>=2.3.1`. This is why the project uses per-tool venvs, not containers.

### 4a. GraspGen — **VERIFIED through the torch step**

```bash
cd GraspGen
uv venv --python 3.10 .venv
uv pip install --python .venv/bin/python torch==2.1.0 torchvision==0.16.0 \
    --index-url https://download.pytorch.org/whl/cu121
uv pip install --python .venv/bin/python -e .
export TORCH_CUDA_ARCH_LIST="8.9"     # your value
cd ..
```

Observed after the torch step:
```
torch 2.1.0+cu121 | cuda build 12.1 | cuda available True
```

> **Expect this warning after the torch step, and ignore it:**
> `UserWarning: Failed to initialize NumPy: _ARRAY_API not found`
>
> torch pulls `numpy 2.x`, but torch 2.1.0 was built against numpy 1.x.
> GraspGen's `pyproject.toml:43` pins `numpy==1.26.4`, so the **next** command
> (`uv pip install -e .`) downgrades numpy and the warning disappears. It is
> only alarming if you stop halfway.

**NOT VERIFIED:** `uv pip install -e .` builds `pointnet2_ops`, which needs
`nvcc`. If you hit `nvcc: command not found`:
```bash
sudo apt install nvidia-cuda-toolkit     # or the CUDA 12.1 toolkit from NVIDIA
export CUDA_HOME=/usr/local/cuda
export PATH=$CUDA_HOME/bin:$PATH
```

### 4b. Grounded-SAM-2 — **NOT VERIFIED on this laptop**

Carried over from the lab machine, where it works.

```bash
cd groundedsam
uv venv --python 3.10 .venv
source .venv/bin/activate
export CUDA_HOME=/usr/local/cuda
export PATH=/usr/local/cuda/bin:$PATH
export TORCH_CUDA_ARCH_LIST="8.9"
uv pip install torch>=2.3.1 torchvision>=0.18.1 --index-url https://download.pytorch.org/whl/cu121
cd Grounded-SAM-2
uv pip install --no-build-isolation -e ".[notebooks]"
uv pip install --no-build-isolation -e grounding_dino
uv pip install "transformers<5"
uv pip install addict yapf timm supervision pycocotools
cd ../..
```

Three things that will bite you:

1. **`--no-build-isolation` is mandatory.** Without it uv pulls an unrelated
   torch into a temp env and the CUDA version check fails against nvcc.
2. **Pin `transformers<5`.** v5 removed `BertModel.get_head_mask`, which
   GroundingDINO's `BertModelWarper` still calls.
3. **`addict yapf timm supervision pycocotools`** are runtime deps neither
   package's `setup.py` installs.

## 5. Sanity check

```bash
source GraspGen/.venv/bin/activate
python3 GraspGen/client-server/graspgen_server.py \
    --gripper_config GraspGen/GraspGenModels/checkpoints/graspgen_franka_panda.yml &
sleep 8
python3 GraspGen/client-server/graspgen_client.py \
    --pcd_file pragmabot/extracted/red_cup/detections/object_pcd.npy
```

**Expect ~6 grasps at confidence 0.92–0.96** — matching the committed
`pragmabot/extracted/red_cup/grasps.npz`. Very different numbers mean the
environment is wrong, not the data.

---

## 6. Problems already hit and fixed — do not re-debug these

| Symptom | Cause | Status |
|---|---|---|
| `setup.sh: line 7: $'\r': command not found` | Windows git converted `.sh` to CRLF; repo had no `.gitattributes` | **fixed** — root `.gitattributes` forces `eol=lf` on all 46 shell scripts |
| `: invalid option name: pipefail` | same CRLF issue — bash read `pipefail\r` | **fixed** with the above |
| `/usr/bin/python3: No module named pip` | Git Bash ships a stripped Python without pip | **fixed** — `setup.sh` is curl-only, no python/pip |
| `wget: command not found` (would have hit next) | `gdino_checkpoints/download_ckpts.sh` calls wget unconditionally | **fixed** — `setup.sh` downloads directly, bypassing that script |
| `Encountered 13 files that should have been pointers` | GraspGen's `.gitattributes` declared `*.json filter=lfs`, but real content is vendored | **fixed** — LFS rule disabled |
| Truncated checkpoint silently skipped forever | skip-check tested existence, not size | **fixed** — exact byte-size verification + resume |
| `Failed to initialize NumPy: _ARRAY_API not found` | numpy 2.x vs torch 2.1.0 ABI | **not a bug** — `uv pip install -e .` downgrades to the pinned 1.26.4 |

## 7. Known environment limits

- **No `nvcc` on a stock Windows or WSL install.** Needed to compile
  `pointnet2_ops` (GraspGen) and `ms_deform_attn` (GroundingDINO).
- **Building on `/mnt/d` (NTFS through WSL) is slow.** For real work, clone into
  the WSL native filesystem (`~/`) instead.
- **Disk:** ~2.5 GB checkpoints + ~4 GB per venv. **VERIFIED**: the GraspGen
  venv alone is **4.0 GB**. Budget ~15 GB.
- **ROS 2 Humble needs Ubuntu 22.04.** WSL's default 24.04 will not install it
  cleanly. Use the Docker container for ROS, or a 22.04 distro.

## 8. What is not in the repo

| Item | Size | How to get it |
|---|---|---|
| Model checkpoints | ~2.5 GB | `bash setup.sh` |
| `red_cup_0.db3` rosbag | 832 MB | ask Harish → `pragmabot/bags/red_cup/` |
| Hand-eye calibration result | few KB | from the lab machine — **not yet copied off it** |
| Python venvs | ~8–12 GB | built above; CUDA extensions are GPU-specific, never copy them |

**You do not need the rosbag to start.** `pragmabot/extracted/` holds two fully
processed scenes (RGB, depth, intrinsics, masks, point clouds, grasps).

## 9. Open the workspace

```bash
code irm2.code-workspace
```

**VERIFIED**: 6 folders resolve, 5 terminal profiles load.

## 10. Read next

`extras/analysis/05_HANDOFF.md` — project state, verified findings, next action.

---

## Verification status, honestly

**Executed and observed on this laptop:**
- full `setup.sh` run, all four checkpoints, exact sizes, resume-after-partial
- `uv venv --python 3.10` (uv fetched CPython 3.10.21 itself)
- torch 2.1.0+cu121 install, `torch.cuda.is_available() == True` under WSL2
- fresh clone with no LFS warnings, no CRLF in 46 shell scripts
- `irm2.code-workspace` folder + profile resolution
- the offline pipeline reproducing `9.5 × 10.7 × 7.3 cm` from committed data

**Not executed:**
- `uv pip install -e .` for GraspGen (needs `nvcc`)
- the entire Grounded-SAM-2 venv
- the GraspGen server/client sanity check in §5
- anything involving the robot

If a step in §4 fails, that is the untested region. Report the error and this
file gets corrected against real output.
