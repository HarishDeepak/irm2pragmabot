# IRM2 — PragmaBot on a Franka FR3

Reproducing **PragmaBot** (Qu et al., *A Pragmatist Robot: Learning to Plan Tasks by Experiencing the Real World*, IEEE RAL 2026, [arXiv:2507.16713](https://arxiv.org/abs/2507.16713)) on a real **Franka FR3** with a **ZED2** camera.

TU Darmstadt · PEARL Lab · *Praktikum zur intelligenten Robotermanipulation (Part II)* · Project 3, *Memory representations for Robotic Task Planning*.

---

## What this is

PragmaBot lets a robot improve its task planning **without any model fine-tuning**: a vision-language model plans a skill, looks at before/after images to judge whether it worked, writes a natural-language critique of its own failures into a short-term memory, and distils completed episodes into a long-term memory retrieved by RAG for future tasks.

**The published code deliberately leaves action execution as `NotImplementedError`.** This repository is that missing half — open-vocabulary segmentation, point-cloud reconstruction, 6-DoF grasp synthesis, hand-eye calibration and ROS 2 motion execution on the FR3 — plus the reproduction of the paper's memory claims on that platform.

Based on [leggedrobotics/pragmabot](https://github.com/leggedrobotics/pragmabot), BSD-3-Clause. This repository's history begins at the official release commit `ee68710`.

---

## Layout

```
irm2pragmabot/
├── pragmabot/                        planner, memory, calibration pipeline
│   ├── pragmabot/                    the ROS package (VLM + STM/LTM/RAG)
│   ├── ros2_ws/src/pragmabot_bridge/ *** the FR3 execution layer -- THE active copy ***
│   ├── calibration/                  detect_object, mask_to_pointcloud, extrinsics
│   ├── extracted/                    2 captured scenes -- TRACKED, work offline now
│   ├── ARMIN.md                      running lab journal, most current day-to-day record
│   └── bags/                         rosbag goes here (not in git, see below)
├── ros2_ws/franka_ros2/              STALE mirror, do not edit -- see caveat below
├── GraspGen/                         6-DoF grasp synthesis (NVIDIA, non-commercial)
├── groundedsam/Grounded-SAM-2/       open-vocabulary segmentation
├── zed_ros2_ws/src/zed-ros2-wrapper/ ZED2 driver
├── setup.sh                          downloads model checkpoints
└── SETUP.md                          environment build guide
```

> **Caveat, confirmed 2026-09-23 — three copies of `bridge_node.py` exist on a
> lab machine, only one is live.** `pragmabot/ros2_ws/src/pragmabot_bridge/`
> (git-tracked, this repo) is the one actually built and run by the Control
> container — confirmed by log lines added there during a live debugging
> session showing up in the real robot's console output. Two other copies are
> stale leftovers from an earlier layout: this repo's own
> `ros2_ws/franka_ros2/pragmabot_bridge/` (last touched 2026-08-24, far
> smaller) and a completely separate, **not git-tracked** directory at
> `~/ros2_ws/franka_ros2/pragmabot_bridge/` on the lab machine (also stale,
> 2026-08-24). Edit only `pragmabot/ros2_ws/src/pragmabot_bridge/`; the other
> two should probably be deleted once someone confirms nothing on the lab
> machine still points at them, but that hasn't been verified/done yet.

## Where things run

| Inside the `franka_ros2_humble` container | On the host |
|---|---|
| franka_ros2 / MoveIt 2 / FR3 control | ZED wrapper (`zed_ros2_ws`) |
| `pragmabot_bridge` | GraspGen ZMQ server (own venv) |
| | Grounded-SAM-2 (own venv) |
| | `pragmabot/calibration/` scripts |

The container mounts `ros2_ws/franka_ros2/` at `/ros2_ws/src` — that is its colcon build path. ROS 2 Humble is installed on the host as well. Everything shares `ROS_DOMAIN_ID=7`.

---

## Getting started

```bash
git clone https://github.com/HarishDeepak/irm2pragmabot.git
cd irm2pragmabot
bash setup.sh          # ~2.5 GB of checkpoints, ~20 min
```

Then follow **[`SETUP_LAPTOP.md`](SETUP_LAPTOP.md)** — the step-by-step laptop
setup guide. It was written from an actual clone-and-run, marks every step as
VERIFIED or NOT VERIFIED, and lists the seven problems already hit and fixed so
you do not re-debug them.

(`SETUP.md` is the shorter reference version of the same steps.)

> **Two venvs, and they cannot be merged.** GraspGen pins `torch==2.1.0`; Grounded-SAM-2 needs `torch>=2.3.1`. Isolation is at the venv level, not the container level — this is deliberate.

> **Replace `TORCH_CUDA_ARCH_LIST="8.9"`** everywhere in `SETUP.md` with your own GPU's value from `nvidia-smi --query-gpu=name,compute_cap --format=csv`. A wrong value either fails to compile or silently builds for the wrong architecture.

### What is not in git

| Item | Size | How to get it |
|---|---|---|
| Model checkpoints | ~2.5 GB | `bash setup.sh` |
| Raw rosbag `red_cup_0.db3` | 832 MB | ask Harish → `pragmabot/bags/red_cup/` |
| Hand-eye calibration result | few KB | from the lab machine; **not yet copied off it** |
| Python venvs | ~12 GB | rebuilt locally — CUDA extensions are GPU-specific |

**You do not need the rosbag to start.** `pragmabot/extracted/` is committed and holds two fully processed scenes (RGB, depth, intrinsics, masks, point clouds, grasps).

### Open the whole workspace in VS Code

```bash
code irm2.code-workspace
```

Six folders in one window, each with its own terminal profile (Control-in-docker,
ZED-on-host, GraspGen venv, GroundedSAM venv, pragmabot). Paths are relative, so
it works wherever you clone.

The original lab version, with absolute `/home/harish/...` paths, is preserved at
`extras/irm2.code-workspace.lab-original` for reference.


### Verify your setup

```bash
source ~/GraspGen/.venv/bin/activate
python3 GraspGen/client-server/graspgen_server.py \
    --gripper_config GraspGen/GraspGenModels/checkpoints/graspgen_franka_panda.yml &
python3 GraspGen/client-server/graspgen_client.py \
    --pcd_file pragmabot/extracted/red_cup/detections/object_pcd.npy
```

Expect ~6 grasps at confidence 0.9+.

---

## Running it

### 1. Control container (FR3 + MoveIt 2)

```bash
cd ros2_ws/franka_ros2
cp .env.example .env
sed -i "s/^USER_UID=.*/USER_UID=$(id -u)/; s/^USER_GID=.*/USER_GID=$(id -g)/" .env

docker compose build          # first time only
docker compose up -d
docker exec -it -e DISPLAY=$DISPLAY franka_ros2_humble bash
```

Inside the container:

```bash
source /ros2_ws/install/setup.bash        # NOT sourced in a fresh shell
colcon build --symlink-install            # only if install/ is missing
ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=10.10.10.10
```

> **Unlock the robot and activate FCI in Desk** (`https://10.10.10.10/desk/`) first, or you get `libfranka: Connection to FCI refused`.

> If `install/` is missing entirely, the container was **recreated** rather than restarted — rebuild.

### 2. ZED camera (host, not a container)

```bash
export ROS_DOMAIN_ID=7                    # required on EVERY terminal
source /opt/ros/humble/setup.bash
cd zed_ros2_ws && colcon build --symlink-install && source install/setup.bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2
```

### 3. Perception + grasping (host, separate venvs)

```bash
# segment
~/groundedsam/.venv/bin/python pragmabot/calibration/detect_object.py \
    --rgb pragmabot/extracted/red_cup/rgb.png --prompt "red cup."

# mask + depth -> object point cloud (numpy only, any venv)
python3 pragmabot/calibration/mask_to_pointcloud.py \
    --depth pragmabot/extracted/red_cup/depth.npy \
    --intrinsics pragmabot/extracted/red_cup/intrinsics.json \
    --mask pragmabot/extracted/red_cup/detections/mask.npy \
    --out /tmp/object_pcd.npy

# grasps (GraspGen venv)
source ~/GraspGen/.venv/bin/activate
python3 GraspGen/client-server/graspgen_server.py \
    --gripper_config GraspGen/GraspGenModels/checkpoints/graspgen_franka_panda.yml &
python3 GraspGen/client-server/graspgen_client.py --pcd_file /tmp/object_pcd.npy
```

### 4. Hand-eye calibration (needs the robot)

```bash
# in the container: gravity compensation so you can jog the arm by hand
ros2 launch franka_bringup example.launch.py \
    controller_names:=gravity_compensation_example_controller

# host: marker detection
ros2 run aruco_detector aruco_detector --ros-args \
    --remap image:=/zed/zed_node/rgb/color/rect/image \
    --remap camera_info:=/zed/zed_node/rgb/color/rect/image/camera_info \
    -p marker_size:='0.05' -p image_is_rectified:=true

# host: calibration GUI
ros2 launch easy_handeye2 calibrate.launch.py name:=fr3_zed_right \
    calibration_type:='eye_on_base' \
    tracking_base_frame:='zed_camera_link' tracking_marker_frame:='marker_0' \
    robot_base_frame:='fr3_link0' robot_effector_frame:='fr3_hand'

# publish + verify
ros2 launch easy_handeye2 publish.launch.py name:=fr3_zed_right
ros2 run tf2_ros tf2_echo fr3_link0 zed_camera_link
```

More detail and every gotcha: `docs/00_MASTER_REFERENCE.md`.

---

## Documentation

`docs/` (top-level) is empty — the file names below don't exist there; use
these instead, which do:

| File | What it covers |
|---|---|
| **`pragmabot/CLAUDE.md`** | **Read first.** Verified facts, confirmed paths, hard rules, behavioral rules. Kept current, don't re-derive what's already here. |
| `pragmabot/ARMIN.md` | Running lab journal — chronological, day-by-day findings, fixes, and open issues. The most current record of what actually happened on hardware. |
| `pragmabot/RUNBOOK.md` | Full pipeline bring-up, in order, with a check after each step. |
| `pragmabot/HANDOFF_2026-08-24_EVENING.md` | Point-in-time project-state snapshot. |
| `pragmabot/LAB_README.md` | Raw scratch log of commands actually typed at the lab. |
| `pragmabot/LAB_README_ARCHIVE.md` | Earlier, longer version of the lab notes, kept for calibration history. |

---

## Status

_(Last refreshed 2026-09-23, from `pragmabot/ARMIN.md` and a live debugging session — verify against it if this drifts again.)_

**Working, verified on real hardware:** the full perception → grasp → execution chain. `pick` + `place` (e.g. green cube onto a tray/bowl) ran reliably (~8+ clean runs, ~1mm arrival error) as of 2026-08-28. `push` (obstacle clearing) is also implemented and has run on hardware. The planner ⇄ executor connection exists and is live — `/pragmabot/execute_skill` dispatches VLM-chosen skills to the bridge. STM and LTM are both implemented, re-enabled, and exercised in real multi-step tasks (e.g. unstacking before picking).

**Known open issues (2026-09-23):** a live debugging session traced repeated `Cartesian approach incomplete` failures to a stale `calib_correction_z` launch override (from before a same-day hand-eye recalibration) pushing the grasp-depth target too close to the table for `avoid_collisions` to clear — root-caused, fix identified, not yet confirmed on hardware. Also open: the wrist/backhand-yaw grasp gate, and multi-view point-cloud capture for non-boxy objects (cups, bowls) — see `ARMIN.md`.

**Not started:** the two course extensions (local VLM acceleration, ontology-based memory).

Most of the paper's claims are reproducible **without the robot** — `rosbag_replay: true` skips execution entirely, and the retrieval ablation executes nothing at all.

---

## Licences

Vendored third-party code keeps its own licence in place. See **`THIRD_PARTY_LICENSES.md`**.

⚠️ **GraspGen is under the NVIDIA License: research/evaluation use only.** Anyone reusing this repository inherits that restriction for the `GraspGen/` component.
