# IRM2 — PragmaBot on a Franka FR3



TU Darmstadt · PEARL Lab · *Praktikum zur intelligenten Robotermanipulation (Part II)* · Project 3, *Memory representations for Robotic Task Planning*.

---

## What this is

[PragmaBot](https://arxiv.org/abs/2507.16713) (Qu et al., IEEE RAL 2026) lets a robot improve its task planning **without any model fine-tuning**: a vision-language model plans a skill, looks at before/after images to judge whether it worked, writes a natural-language critique of its own failures into a short-term memory (STM), and distils completed episodes into a long-term memory (LTM) retrieved via RAG for future tasks.

**The published code deliberately leaves action execution as `NotImplementedError`.** This repository is that missing half, built for this course project: open-vocabulary segmentation, point-cloud reconstruction, 6-DoF grasp synthesis, hand-eye calibration, and ROS 2 motion execution on a real **Franka FR3** with a **ZED2** camera — plus reproduction of the paper's STM/LTM claims on that platform.

Based on [leggedrobotics/pragmabot](https://github.com/leggedrobotics/pragmabot), BSD-3-Clause. The vendored copy at [`pragmabot/`](pragmabot/) keeps upstream's own `README.md`, `CHANGELOG.md`, `CONTRIBUTING.md`, and `LICENSE`; this top-level README covers what was added on top for the FR3.

---

## Pipeline

Each task executes the PragmaBot loop, with steps 4 and parts of step 1 now backed by real perception and a real robot instead of stubs:

1. **Perception** — Grounded-SAM-2 segments the target object from a ZED2 RGB frame; the depth image and hand-eye calibration turn the mask into an object point cloud in the robot's base frame.
2. **Scene description + planning** — `VLMSceneDescriber` and `VLMTaskPlanner` (unchanged from upstream) choose the next skill (`pick`, `place`, `push`) using the current observation, retrieved LTM entries, and the accumulated STM.
3. **Grasp synthesis** — GraspGen proposes 6-DoF grasp candidates over the object point cloud; candidates are filtered by finger-width, joint-limit proximity, and approach-tilt gates before one is sent to the arm.
4. **Execution** — `pragmabot_bridge` (ROS 2, this repo) drives the FR3 through MoveIt 2: hover, approach, grasp/push, retreat.
5. **Success detection + memory** — `VLMSuccessDetector` compares before/after images (unchanged from upstream); on task completion, `VLMExperienceSummarizer` distills the episode into the LTM.

## Layout

```
irm2pragmabot/
├── pragmabot/                        planner, memory, calibration pipeline (vendored PragmaBot + FR3 additions)
│   ├── pragmabot/                    the ROS package (VLM + STM/LTM/RAG) — upstream, unmodified logic
│   ├── ros2_ws/src/pragmabot_bridge/ *** the FR3 execution layer — THE active copy ***
│   ├── calibration/                  detect_object, mask_to_pointcloud, extrinsics
│   ├── extracted/                    2 captured scenes — TRACKED, work offline now
│   ├── ARMIN.md                      running lab journal, most current day-to-day record
│   ├── CLAUDE.md                     verified facts, confirmed paths, project rules
│   ├── RUNBOOK.md                    full pipeline bring-up, in order, with a check after each step
│   └── bags/                         rosbag goes here (not in git, see below)
├── ros2_ws/franka_ros2/              STALE mirror, do not edit — see caveat below
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
> 2026-08-24). Edit only `pragmabot/ros2_ws/src/pragmabot_bridge/`.

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
VERIFIED or NOT VERIFIED, and lists the problems already hit and fixed so
you do not re-debug them.

(`SETUP.md` is the shorter reference version of the same steps.)

> **Two venvs, and they cannot be merged.** GraspGen pins `torch==2.1.0`; Grounded-SAM-2 needs `torch>=2.3.1`. Isolation is at the venv level, not the container level — this is deliberate.

> **Replace `TORCH_CUDA_ARCH_LIST="8.9"`** everywhere in `SETUP.md` with your own GPU's value from `nvidia-smi --query-gpu=name,compute_cap --format=csv`. A wrong value either fails to compile or silently builds for the wrong architecture.

### What is not in git

| Item | Size | How to get it |
|---|---|---|
| Model checkpoints | ~2.5 GB | `bash setup.sh` |
| Raw rosbag `red_cup_0.db3` | 832 MB | request from project owner → `pragmabot/bags/red_cup/` |
| Hand-eye calibration result | few KB | from the lab machine; not yet copied off it |
| Python venvs | ~12 GB | rebuilt locally — CUDA extensions are GPU-specific |

**You do not need the rosbag to start.** `pragmabot/extracted/` is committed and holds two fully processed scenes (RGB, depth, intrinsics, masks, point clouds, grasps).

### Open the whole workspace in VS Code

```bash
code irm2.code-workspace
```

Six folders in one window, each with its own terminal profile (Control-in-docker,
ZED-on-host, GraspGen venv, GroundedSAM venv, pragmabot). Paths are relative, so
it works wherever you clone.

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

More detail and every gotcha: [`extras/analysis/00_MASTER_REFERENCE.md`](extras/analysis/00_MASTER_REFERENCE.md).

---

## Documentation

| File | What it covers |
|---|---|
| **[`pragmabot/CLAUDE.md`](pragmabot/CLAUDE.md)** | **Read first.** Verified facts, confirmed paths, hard rules. |
| [`pragmabot/ARMIN.md`](pragmabot/ARMIN.md) | Running lab journal — chronological, day-by-day findings, fixes, and open issues. The most current record of what actually happened on hardware. |
| [`pragmabot/RUNBOOK.md`](pragmabot/RUNBOOK.md) | Full pipeline bring-up, in order, with a check after each step. |
| [`extras/analysis/`](extras/analysis/) | Paper analysis, reproduction plan, ROS 2 migration notes, and a from-scratch walkthrough of the paper's memory-retrieval mechanism. |

---

## Status

_(Last refreshed 2026-09-27, from `pragmabot/ARMIN.md`.)_

**Working, verified on real hardware:** the full perception → grasp → execution chain. `pick` + `place` (e.g. green cube onto a tray/bowl) ran reliably (~8+ clean runs, ~1mm arrival error) as of 2026-08-28. `push` (obstacle clearing) is also implemented and has run on hardware. The planner ⇄ executor connection exists and is live — `/pragmabot/execute_skill` dispatches VLM-chosen skills to the bridge. STM and LTM are both implemented, re-enabled, and exercised in real multi-step tasks (e.g. unstacking before picking), with retrieval and saving confirmed on live runs.

**Not started:** the two course extensions (local VLM acceleration, ontology-based memory).

Most of the paper's claims are reproducible **without the robot** — `rosbag_replay: true` skips execution entirely, and the retrieval ablation executes nothing at all.

## Results (upstream, reference)

PragmaBot's own evaluation, on a legged manipulator with a 6-DoF arm across 12 real-world object-manipulation scenarios (see the [paper](https://arxiv.org/abs/2507.16713) for the full table): STM self-reflection raises task success from **35% → 84%**; LTM with RAG raises single-trial success from **22% → 80%**, generalizing to previously unseen scenarios. This repository's own FR3 trial results are tracked in `pragmabot/pragmabot/data/ltm/` and `pragmabot/pragmabot/data/logs/`.

---

## Citation

If you build on the underlying method, please cite the original paper:

```bibtex
@article{qu2026pragmatist,
  title={A Pragmatist Robot: Learning to Plan Tasks by Experiencing the Real World},
  author={Qu, Kaixian and Lan, Guowei and Zurbrügg, René and Chen, Changan and Mower, Christopher E and Bou-Ammar, Haitham and Hutter, Marco},
  journal={IEEE Robotics and Automation Letters},
  year={2026},
  publisher={IEEE}
}
```

## Licences

Vendored third-party code keeps its own licence in place. See **[`THIRD_PARTY_LICENSES.md`](THIRD_PARTY_LICENSES.md)**.

⚠️ **GraspGen is under the NVIDIA License: research/evaluation use only.** Anyone reusing this repository inherits that restriction for the `GraspGen/` component.

## Acknowledgements

- [Qu, Lan, Zurbrügg, Chen, Mower, Bou-Ammar, Hutter — *A Pragmatist Robot*, IEEE RAL 2026](https://arxiv.org/abs/2507.16713) and [leggedrobotics/pragmabot](https://github.com/leggedrobotics/pragmabot) — the planning, memory, and evaluation pipeline this project builds action execution for.
- [NVIDIA GraspGen](https://github.com/NVlabs/GraspGen) — 6-DoF grasp synthesis.
- [IDEA-Research Grounded-SAM-2](https://github.com/IDEA-Research/Grounded-SAM-2) — open-vocabulary segmentation.
- PEARL Lab, TU Darmstadt — *Praktikum zur intelligenten Robotermanipulation (Part II)*, course supervision and FR3/lab access.
