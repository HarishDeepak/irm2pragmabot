# PragmaBot × Panda — Setup Guide

**PEARL Lab, TU Darmstadt | IRM2 Project**  
Robot: Franka Panda + ZED2 camera  
Branch: `feature/panda-adaptation`

---

## What This Is

PragmaBot is a VLM-driven robotic task learning system (originally from ETH Zurich RSL, IEEE RAL 2026), adapted here to run on a Franka Panda robot. It uses GPT-4o to observe a scene, plan pick/place/push actions, execute them on the Panda, and learn from experience via short-term and long-term memory.

**Architecture overview:**

```
Camera (ZED2)
     │
     ▼
VLMSceneDescriber  →  GPT-4o describes the scene
     │
     ▼
VLMTaskPlanner     →  GPT-4o chooses: PICK / PLACE / PUSH + target
     │
     ▼
PandaSkillExecutor →  sends goal to action server
     │
     ├── PandaPickServer:  GroundedSAM → GraspGen → MoveIt
     ├── PandaPlaceServer: GroundedSAM → centroid → MoveIt
     └── PandaPushServer:  GroundedSAM × 2 → Cartesian push
     │
     ▼
VLMSuccessDetector →  GPT-4o compares before/after images
     │
     ▼
STM / LTM update   →  short-term memory per episode, long-term CSV
```

---

## Prerequisites

- Docker installed on the host machine
- NVIDIA GPU on the host (for GraspGen — CPU fallback possible but slow)
- `nvidia-docker2` or Docker with `--gpus` support
- Git access to `https://github.com/HarishDeepak/IRM2.git`
- An OpenAI API key with GPT-4o access

---

## Step 1 — Start the ROS Container

```bash
# Pull and run the Franka container (first time only)
docker pull 3liyounes/pearl_robots:franka

docker run -it --name pragmabot_panda \
  --network host \
  --privileged \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  3liyounes/pearl_robots:franka bash

# On re-entry (container already exists):
docker start -ai pragmabot_panda
```

> **Note:** `--network host` is important — it lets the container reach the GraspGen server running on the host at `172.17.0.1:5556`.

---

## Step 2 — Clone the Repo

Inside the container:

```bash
cd /catkin_ws/src
git clone https://github.com/HarishDeepak/IRM2.git pragmabot
cd pragmabot
git checkout feature/panda-adaptation
```

---

## Step 3 — Install Python Dependencies

```bash
pip install -r /catkin_ws/src/pragmabot/requirements.txt

# Additional dependencies for Panda adaptation
pip install pyzmq msgpack msgpack-numpy ros-numpy
```

---

## Step 4 — Install GroundingDINO

```bash
cd /tmp
git clone https://github.com/IDEA-Research/GroundingDINO.git
cd GroundingDINO
pip install -e .
```

> This step is slow (~5–15 min) due to CUDA extension compilation. You will see:
> `UserWarning: Failed to load custom C++ ops. Running on CPU mode Only!`
> This is fine — GroundingDINO still works on CPU.

Download weights:

```bash
mkdir -p /catkin_ws/src/pragmabot/weights
cd /catkin_ws/src/pragmabot/weights
curl -L -o groundingdino_swint_ogc.pth \
  https://github.com/IDEA-Research/GroundingDINO/releases/download/v0.1.0-alpha/groundingdino_swint_ogc.pth
```

---

## Step 5 — Install SAM

```bash
pip install segment-anything

cd /catkin_ws/src/pragmabot/weights
curl -L -o sam_vit_h_4b8939.pth \
  https://dl.fbaipublicfiles.com/segment_anything/sam_vit_h_4b8939.pth
```

> SAM ViT-H weights are ~2.4 GB — takes a few minutes to download.

Verify both work:

```bash
python3 -c "from groundingdino.util.inference import load_model; print('gdino ok')"
python3 -c "from segment_anything import sam_model_registry; print('sam ok')"
```

---

## Step 6 — Build the ROS Package

```bash
cd /catkin_ws
catkin_make --pkg pragmabot
source devel/setup.bash
```

Verify action messages generated:

```bash
python3 -c "from pragmabot.msg import PandaPickAction, PandaPlaceAction, PandaPushAction; print('msgs ok')"
```

---

## Step 7 — Set API Key

PragmaBot now supports three VLM backends. Set the key for whichever you use:

**OpenAI (GPT-4o):**
```bash
export OPENAI_API_KEY="sk-..."
```

**Anthropic (Claude):** *(recommended — supervisor-provided)*
```bash
export ANTHROPIC_API_KEY="sk-ant-..."
pip install anthropic sentence-transformers
```

> Claude has no embeddings API, so embeddings fall back to `sentence-transformers` running locally (`all-MiniLM-L6-v2`). The first call downloads ~90 MB of model weights automatically.

**Google Gemini:**
```bash
export GOOGLE_API_KEY="..."
pip install google-generativeai
```

Select the backend in `config.yaml` under `vlm.vlm_model`:
- OpenAI: `gpt-4o-2024-08-06`
- Claude: `claude-opus-4-8`
- Gemini: `gemini-2.5-pro`

Add the relevant key to `~/.bashrc` to persist across sessions:
```bash
echo 'export ANTHROPIC_API_KEY="sk-ant-..."' >> ~/.bashrc
```

---

## Step 8 — Start the GraspGen Server (on the HOST machine, not in container)

GraspGen requires Python 3.10 + CUDA. It runs on the host machine and the container connects to it via ZMQ.

On your **host machine** (outside Docker):

```bash
# Install Python 3.10 if needed (via pyenv or conda)
# Then clone GraspGen:
git clone https://github.com/NVlabs/GraspGen.git /tmp/GraspGen
cd /tmp/GraspGen

# Install (uv recommended):
pip install uv
uv venv --python 3.10 .venv
source .venv/bin/activate
uv pip install -e .
bash install_pointnet.sh

# Download Franka Panda checkpoint:
git clone https://huggingface.co/adithyamurali/GraspGenModels /tmp/GraspGenModels

# Start the server:
python client-server/graspgen_server.py \
  --gripper_config /tmp/GraspGenModels/checkpoints/graspgen_franka_panda.yml \
  --port 5556
```

The container reaches this server at `172.17.0.1:5556` (Docker bridge gateway). No extra configuration needed.

> **Without GPU / not in lab:** The system runs with stub responses — the VLM planning loop still works, execution just won't move the robot. Useful for testing memory and planning offline.

---

## Step 9 — Configure ROS Topics

Edit `pragmabot/config/config.yaml` to match your ZED2 topic names:

```yaml
topics:
  color_image: /zed2/zed_node/rgb/image_rect_color
  depth_image: /zed2/zed_node/depth/depth_registered
  camera_info: /zed2/zed_node/rgb/camera_info
```

Also set:

```yaml
rosbag_replay: false   # true = skip real execution (for offline testing)
```

---

## Step 10 — Launch

### Option A: With real robot

```bash
# Terminal 1 — MoveIt + Franka control
roslaunch panda_moveit_config franka_control.launch robot_ip:=<ROBOT_IP>

# Terminal 2 — ZED2 camera
roslaunch zed_wrapper zed2.launch

# Terminal 3 — Panda action servers
source /catkin_ws/devel/setup.bash
roslaunch pragmabot launch_panda_servers.launch

# Terminal 4 — Main PragmaBot node
export OPENAI_API_KEY="sk-..."
source /catkin_ws/devel/setup.bash
roslaunch pragmabot launch_pragmabot.launch
```

Open the Gradio UI at `http://localhost:7860`, type an instruction like `"put the apple on the plate"`.

### Option B: Rosbag replay (offline / no robot)

```bash
# Terminal 1 — Play rosbag
rosbag play -l /path/to/your.bag

# Terminal 2 — PragmaBot (rosbag_replay: true in config.yaml)
export OPENAI_API_KEY="sk-..."
roslaunch pragmabot launch_pragmabot.launch
```

---

## One-Time Lab Setup (do once per robot)

### Measure observation pose

Jog the arm to a configuration where:
- The entire arm is out of the ZED2 field of view
- The workspace is fully visible

Then read joint values:

```bash
rostopic echo /joint_states -n 1
```

Replace `OBSERVATION_JOINT_CONFIG` in all three server files:
- `pragmabot/nodes/panda_pick_server.py` line ~17
- `pragmabot/nodes/panda_place_server.py` line ~17
- `pragmabot/nodes/panda_push_server.py` line ~17

### ZED2 → panda_link0 TF calibration

Create `pragmabot/launch/static_zed2_tf.launch`:

```xml
<launch>
  <node pkg="tf2_ros" type="static_transform_publisher" name="zed2_to_panda"
        args="TX TY TZ QX QY QZ QW zed2_left_camera_frame panda_link0" />
</launch>
```

Replace `TX TY TZ QX QY QZ QW` with calibrated values. Verify:

```bash
rosrun tf tf_echo panda_link0 zed2_left_camera_frame
```

### Replace placeholder camera intrinsics

The pick/place/push servers currently use `fx=fy=700` as a placeholder. Replace with real values from:

```bash
rostopic echo /zed2/zed_node/rgb/camera_info -n 1
```

Look for `K[0]` (fx) and `K[4]` (fy), `K[2]` (cx), `K[5]` (cy). Update `_mask_to_pointcloud` / `_mask_centroid_3d` in each server.

---

## Testing GroundedSAM Offline

```bash
python3 /catkin_ws/src/pragmabot/scripts/test_grounded_sam.py \
    --image /path/to/image.png \
    --text "apple"
# Result saved to /tmp/gsam_result.png
```

---

## Key Files

| File | Purpose |
|------|---------|
| `nodes/pragmabot_node.py` | Main orchestrator, Gradio UI, VLM loop |
| `nodes/panda_pick_server.py` | Pick action server (SAM → GraspGen → MoveIt) |
| `nodes/panda_place_server.py` | Place action server (SAM → centroid → MoveIt) |
| `nodes/panda_push_server.py` | Push action server (SAM × 2 → Cartesian push) |
| `src/pragmabot/panda_skill_executor.py` | Bridges VLM planner to action servers |
| `src/pragmabot/grounded_sam.py` | GroundingDINO + SAM wrapper |
| `src/pragmabot/graspgen_client.py` | ZMQ client for GraspGen server on host |
| `config/config.yaml` | Runtime configuration |
| `action/PandaPick.action` | Pick action message definition |
| `action/PandaPlace.action` | Place action message definition |
| `action/PandaPush.action` | Push action message definition |

**Do NOT modify these upstream files:**  
`vlm_client.py`, `vlm_task_planner.py`, `vlm_success_detector.py`, `vlm_scene_describer.py`, `vlm_exp_summarizer.py`, `memory_manager.py`, `scene_observer.py`

---

## Common Errors

| Error | Fix |
|-------|-----|
| `Failed to load custom C++ ops` | Normal warning — GroundingDINO runs on CPU. Ignore. |
| `GraspGen server timeout` | Start the GraspGen server on the host machine (Step 8) |
| `Timed out waiting for RGBD image pair` | Check ZED2 is running and topic names match `config.yaml` |
| `No IK solution for ...` | Observation pose not calibrated — do One-Time Lab Setup |
| `OPENAI_API_KEY not set` | `export OPENAI_API_KEY="sk-..."` |
| `catkin build: build space previously built by catkin_make` | Use `catkin_make --pkg pragmabot` instead of `catkin build` |
| `ModuleNotFoundError: No module named '_curses'` | Harmless warning from pyenv Python build. Ignore. |

---

## Phase Completion Status

| Phase | Description | Status |
|-------|-------------|--------|
| 1 | Environment verification | ✅ Done |
| 2 | Action messages + stub files | ✅ Done |
| 3 | pragmabot_node.py wiring | ✅ Done |
| 4 | GroundedSAM installation + implementation | ✅ Done |
| 5 | GraspGen ZMQ client | ✅ Done |
| 6 | Full action server implementations | ✅ Done |
| 7 | Physical robot validation | ⏳ Needs lab |

---

## Reproducing the ZED2/FR3 extrinsic calibration environment (2026-07-27)

Everything needed to bring this exact setup back up on the lab machine (`Alonnisos`) after time away. All commands assume this host, not a fresh machine — paths below are absolute for that reason.

**Required running pieces, in order:**

1. **`export ROS_DOMAIN_ID=7`** in every terminal that touches ZED or FR3 topics — the ZED wrapper and franka containers both publish/subscribe on domain 7, not the ROS2 default (domain 0). Forgetting this is the single most common "nothing shows up" symptom.

2. **ZED2 camera** (host, not a container):
   ```
   source /opt/ros/humble/setup.bash
   source ~/zed_ros2_ws/install/setup.bash
   ros2 launch zed_wrapper zed_camera.launch.py \
       camera_model:=zed2 \
       param_overrides:="general.grab_resolution:=HD1080"
   ```
   `depth_mode` and `pos_tracking_enabled` are NOT top-level launch args in this wrapper version — they're `param_overrides` dotted keys (`depth.depth_mode`, `pos_tracking.pos_tracking_enabled`) and both already default correctly on the `local-fr3-setup` branch (`NEURAL_PLUS`, `false`) — no override needed for those two. Confirmed topic namespace is `/zed/zed_node/...` (not `/zedxm` — that only appears in this repo's stale `config.yaml`).

3. **FR3 robot + MoveIt** (inside the `franka_ros2_humble` docker container, host networking, already has `ROS_DOMAIN_ID=7` baked in):
   ```
   docker start franka_ros2_humble   # if not already running
   docker exec -it -e DISPLAY=$DISPLAY franka_ros2_humble bash
   source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash
   ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=10.10.10.10
   ```
   **Check first whether this is already running** (`ps aux | grep move_group` inside the container) before launching again — a second `moveit.launch.py` instance starts its own `robot_state_publisher`/`move_group`/`ros2_control_node` that conflicts with an existing one (its controller spawners fail since the real hardware connection is already held), and having two `robot_state_publisher`s publish the same `fr3_link*` TF frames makes the robot model render with stale/wrong joint values in RViz ("floating" robot, disconnected from the real point cloud). If one's already up, just attach a plain `rviz2 -d /ros2_ws/install/franka_fr3_moveit_config/share/franka_fr3_moveit_config/rviz/moveit.rviz` instead of relaunching the whole thing.

4. **FoundationPose container** (GPU-bound, RTX 4080 needs CUDA 11.8+/sm_89 — the other two pulled images are stuck on CUDA 11.3 and won't compile):
   ```
   docker start foundationpose_calib
   ```
   Sanity check mesh units before any real run:
   ```
   docker exec foundationpose_calib /opt/conda/envs/my/bin/python -c \
       "import trimesh; print(trimesh.load('/workspace/FoundationPose/calib_io/cube.obj').bounding_box.extents)"
   ```
   should print `[0.061 0.061 0.061]` (meters, not mm).

5. **Run the calibration** (host, `~/pragmabot/calibration/`):
   ```
   python3 calibrate_extrinsic.py --mesh ~/foundationpose/FoundationPose/calib_io/cube.obj \
       --num-frames 8 --camera-topic-ns /zed/zed_node \
       --cube-xyz-in-base X Y Z --cube-rpy-in-base R P Y
   ```
   Omit `--cube-xyz-in-base`/`--cube-rpy-in-base` entirely to run just the vision capture (saves `T_cube_in_camera` to `--out-cube-pose` without publishing TF) if the hand measurement isn't ready yet — avoids re-running the expensive GPU step later.

**Gotchas already hit once, don't re-debug them:**
- `register_once.py` OOMs a 16GB GPU at full 1920x1080 input — it now downscales to `--max-side 960` by default (rescales K to match) before calling `register()`.
- Don't publish the external calibration TF directly to `zed_left_camera_frame_optical` — that frame already has a parent inside the ZED wrapper's own internal static TF tree (`zed_camera_link -> ... -> zed_left_camera_frame_optical`). Attach to `zed_camera_link` instead (the actual root of the ZED's tree), composing out the fixed offset via `tf2_echo zed_camera_link zed_left_camera_frame_optical`.
- If re-taping the calibration cube, match `mesh_gen.py`'s `--stripe-frac` exactly (currently `0.3` → ~1.8cm on the 6.1cm cube) — a mismatch between the physical tape and the modeled mesh stripe biases FoundationPose's rendered-vs-real pose scoring.
- System `franka_description` (`/opt/ros/humble/share/franka_description`) is missing its `.dae`/`.stl` mesh files on the host (only the container's own install has them) — RobotModel display in a host-launched RViz will show load errors; use the container's own `rviz2` (has working meshes) instead of a host one for anything showing the robot mesh.
