# RUNBOOK — bringing up the full VLM pick pipeline

Lab machine `Alonnisos`. Everything below is the **host** unless it says
container. `ROS_DOMAIN_ID=7` is already in `~/.bashrc`; so is the ROS + ZED
overlay, so a fresh terminal is ready to go.

Bring the pieces up **in this order**. Each has a check — do not skip them,
every failure this project has hit was a dead process upstream.

---

## 0. Robot, at the Desk

Before any terminal:

- **User Stop fully twisted out.** A half-release still reads as pressed.
- Desk: joints unlocked, brakes released, **FCI active**, **end effector
  connected**.
- No other Desk session holding control.

If the User Stop fires later, `franka_gripper_node` dies and does not come back.
That is the single most common failure here.

---

## 1. ZED camera (host)

```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2
```

Check: `ros2 node list | grep zed_node`

---

## 2. Robot + MoveIt + RViz + gripper (container)

```bash
docker start franka_ros2_humble          # if not already running
docker exec -it -e DISPLAY=$DISPLAY franka_ros2_humble bash
ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=10.10.10.10
```

`-e DISPLAY=$DISPLAY` is not optional — without it RViz silently fails to open
(the image carries a stale `DISPLAY=:2`; the real socket is `:1`).

**Do not launch this twice.** A second instance fights the first for the
hardware and makes the robot render with stale joint values in RViz. If it is
already up, attach a plain `rviz2` instead.

### THE GATE — check this before every run

```bash
ros2 action info /franka_gripper/homing
```

Must read **`Action servers: 1`**. If it reads `0`, the gripper node has died
and every pick will fail at homing. Wait a full minute after launch and check
again — it sometimes comes up and then dies.

If it died but the rest of the stack is alive, restart just the gripper
(the standalone launch file requires a `namespace` argument that cannot be
passed empty from a shell, so run the node directly):

```bash
ros2 run franka_gripper franka_gripper_node --ros-args \
  -r __node:=franka_gripper \
  -p robot_ip:=10.10.10.10 \
  -p joint_names:="[fr3_finger_joint1, fr3_finger_joint2]" \
  --params-file /ros2_ws/install/franka_gripper/share/franka_gripper/config/franka_gripper_node.yaml
```

Also check controllers: `ros2 control list_controllers` → `fr3_arm_controller`
active.

---

## 3. Perception server — GroundedSAM (host)

**Give it its own terminal and leave it alone.** Reusing this terminal kills the
server, and the failure looks like a 60 s timeout in the bridge.

```bash
source ~/groundedsam/.venv/bin/activate
cd ~/irm2pragmabot/pragmabot
python3 calibration/perception_server.py
```

Ready when it prints `Listening on tcp://127.0.0.1:5557` (~5 s).

---

## 4. GraspGen server (host)

```bash
source ~/GraspGen/.venv/bin/activate
python3 ~/GraspGen/client-server/graspgen_server.py \
    --gripper_config ~/GraspGen/GraspGenModels/checkpoints/graspgen_franka_panda.yml
```

Ready when it prints `GraspGen ZMQ server listening on tcp://0.0.0.0:5556`
(~45 s the first time).

### Check both, without the robot

```bash
cd ~/irm2pragmabot
pragmabot/calibration/test_zmq_pipeline.sh "cup" pragmabot/extracted/red_cup
```

Expect `conf 0.778, 2000 points, extent 9.5 x 10.7 x 7.3 cm` and ~100 grasps.
This exercises both servers on the GPU and touches no robot.

---

## 5. TERMINAL A — bridge (host)

```bash
export ROS_DOMAIN_ID=7
source /opt/ros/humble/setup.bash
source ~/pragmabot_bridge_ws/install/setup.bash
export PYTHONPATH=$PYTHONPATH:$HOME/GraspGen
ros2 run pragmabot_bridge bridge_node
```

Expect two lines then idle:
```
pragmabot_bridge started - waiting for /move_action, ...
Serving /pragmabot/execute_skill - waiting for planner goals
```

---

## 6. TERMINAL B — VLM planner + Gradio (host)

```bash
export ROS_DOMAIN_ID=7
source /opt/ros/humble/setup.bash
source ~/pragmabot_bridge_ws/install/setup.bash
export PYTHONPATH=$PYTHONPATH:$HOME/irm2pragmabot/pragmabot/pragmabot/src:$HOME/GraspGen
cd ~/irm2pragmabot/pragmabot
python3 pragmabot/nodes/pragmabot_node.py
```

Expect `Using claude-opus-4-8 with Anthropic`, then `Running on local URL:
http://0.0.0.0:7860`. **Do not run this from inside a venv** — gradio and
omegaconf are installed for the system python.

Open `http://127.0.0.1:7860` and give it one instruction: **`pick up the cup`**.

Start a **fresh task** rather than continuing an old one — a failed task leaves
failures in short-term memory that steer the next plan.

---

## What should happen

```
Terminal B  VLM task planner running...          ~6-13 s
Terminal B  Sending pick goal: target='red cup'
Terminal A  ExecuteSkill goal: skill='pick' ...
Terminal B  [skill] perceiving                   ~1.5 s
Terminal A  perception: N grasps for 'red cup' (conf 0.66-0.95)
Terminal A  Auto-selected grasp i/N (confidence 0.94, best available 0.95)
Terminal B  [skill] picking
Terminal A  standoff pose (fr3_link0):  <4x4>
Terminal A  grasp pose (fr3_link0):     <4x4>
            → gripper homes (fingers move), then ARM MOVES:
              standoff → Cartesian approach → close → 12 cm lift → stop
```

`place_after_s = 0.0`, so a pick ends after the lift. No place.

### Anchors — stop it if

- grasp translation far from `[0.602, -0.010, 0.061]`; specifically `x > 0.86`
  or `|z| > 0.5`
- the standoff move starts swinging the base — there is **no collision scene in
  MoveIt**, nothing prevents a path through the table

### Lines worth watching

| Line | Meaning |
|---|---|
| `slowing the trajectory to N% of its planned speed` | FR3 velocity gate re-timed a trajectory. Path unchanged, timing only. Normal. |
| `Refusing to execute:` | Gate declined on purpose. Names the joint and the numbers. |
| `Homing action server did not appear within 10s` | Gripper node is dead. Go back to §2's gate. |
| `Cartesian path incomplete (fraction=...) - retrying with a finer step` | Normal; it halves `eef_step` down to 1 mm. |

---

## Reading the three surfaces

- **Terminal A (bridge)** — robot ground truth. `ERROR` lines here are the real
  cause of any failure.
- **Terminal B (VLM node)** — what Claude decided and what the executor sent.
  It only knows the reason the bridge put in the result message.
- **Gradio** — the conversation, with Claude's reasoning and its
  success/failure judgement. If the bridge reports a vague reason, Claude will
  invent a plausible physical one; trust Terminal A over the reasoning text.
- **`pragmabot/data/logs/*.jsonl`** — one file per task, the structured record
  for the report.

---

## After editing bridge code

Edits are **not** live — rebuild:

```bash
source /opt/ros/humble/setup.bash && source ~/pragmabot_bridge_ws/install/setup.bash
cd ~/pragmabot_bridge_ws && colcon build --packages-select pragmabot_bridge --symlink-install
```

Then restart Terminal A.

Robot-free tests (no ROS, no GPU, no servers):
```bash
cd ~/irm2pragmabot/pragmabot
for t in scripts/test_*.py; do python3 "$t"; done     # expect 8 x pass
```

---

## Quick triage

| Symptom | First thing to check |
|---|---|
| `ros2: command not found` | new terminal not sourced — should be automatic via `~/.bashrc` |
| RViz does not open | `docker exec` missing `-e DISPLAY=$DISPLAY` |
| `perception server did not answer within 60s` | the server process is dead — did its terminal get reused? |
| pick hangs with no output | should no longer happen; if it does, timeouts are not in the built package — rebuild |
| `Homing action server did not appear` | `ros2 action info /franka_gripper/homing` → `Action servers: 0` |
| gripper node keeps dying | User Stop not fully released; check Desk |
| VLM keeps choosing `push` | prompt fix not picked up — restart Terminal B |
| arm reaches but misses the object | grab the `grasp pose (fr3_link0)` matrix before doing anything else |
