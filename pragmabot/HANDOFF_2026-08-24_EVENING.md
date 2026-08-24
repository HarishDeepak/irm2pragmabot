# HANDOFF — 2026-08-24, evening session (~19:30–21:30)

Continues `extras/HANDOFF_2026-08-24.md` (written earlier the same day, from the
laptop). **Read that one first for the verified calibration facts and the
protected-module rule.** This file covers only the evening lab session.

Report due **2026-09-14**.

---

## 0. State in six lines

- The **full VLM pipeline ran end to end for the first time**: Claude planned →
  executor → action server → GroundedSAM → GraspGen → MoveIt. The arm moved.
- The pick was **not accurate** and did not grasp. Root causes found, two fixed.
- `franka_gripper_node` **crashes repeatedly** — the current blocker. Not caused
  by anything in this repo.
- **8 robot-free test scripts, all passing.** Two are new.
- **Nothing is committed.** Branch `ros2-port`, previously in sync with origin.
- Everything runs on the **host**, not in the container. See §2 — this was
  misdiagnosed for an hour and is the single most important thing to know.

---

## 1. Where the code actually lives and runs

This cost an hour. There are **three** copies of `bridge_node.py`:

| Path | What it is |
|---|---|
| `~/irm2pragmabot/pragmabot/ros2_ws/src/pragmabot_bridge/` | **the source of truth — edit here** |
| `~/pragmabot_bridge_ws/src/pragmabot_bridge` | **symlink** to the above |
| `~/ros2_ws/franka_ros2/pragmabot_bridge/` | stale duplicate, bind-mounted into the container, **not used to run the bridge** |

`ros2 run pragmabot_bridge bridge_node` runs from **`~/pragmabot_bridge_ws`**,
whose `src/` symlinks straight into the repo, built with `--symlink-install`:

```
pragmabot_bridge     -> ~/irm2pragmabot/pragmabot/ros2_ws/src/pragmabot_bridge
pragmabot_interfaces -> ~/irm2pragmabot/pragmabot/ros2_ws/src/pragmabot_interfaces
franka_msgs          -> ~/ros2_ws/franka_ros2/franka_msgs
```

**After editing bridge code you must rebuild** (ament_python copies sources into
`build/`, so edits are not live):

```bash
source /opt/ros/humble/setup.bash && source ~/pragmabot_bridge_ws/install/setup.bash
cd ~/pragmabot_bridge_ws && colcon build --packages-select pragmabot_bridge --symlink-install
```

`pragmabot/scripts/deploy_to_container.sh` was written this session for the
CONTAINER path. It is only relevant if the bridge is ever run inside the
container. **It is not part of the normal workflow.** It does report drift
(`--check`), which is how the stale duplicate was found: the container had been
building an Aug-14 `bridge_node.py` with no action server and a
`pragmabot_interfaces` whose `ExecuteSkill` was never generated.

---

## 2. Environment fixes applied (persistent)

- **Host `~/.bashrc`** — sources `/opt/ros/humble`, `~/zed_ros2_ws/install`, sets
  `ROS_DOMAIN_ID=7`. Backup `~/.bashrc.bak-20260824`.
  *Note: `~/irm2pragmabot/zed_ros2_ws` is a source-only copy with no `install/`.
  The real built one is `~/zed_ros2_ws`. `ros2 launch` resolves by package, not
  cwd, so the directory you stand in never mattered.*
- **Container `~/.bashrc`** — sources `/ros2_ws/install/setup.bash`, sets
  `ROS_DOMAIN_ID=7` and `DISPLAY=:1`. (RViz failed to open because a
  `docker exec` without `-e DISPLAY=$DISPLAY` inherited a stale `DISPLAY=:2`
  while the real socket is `X1`.)
- **Container restart policy** `no` → `unless-stopped`. It was not coming back
  after host reboots while other users' containers were, which read as "the
  workspace isn't built".
- **`NOT_THE_BUILT_COPY.md`** placed in the four vendored source dirs
  (`zed_ros2_ws/`, `ros2_ws/franka_ros2/`, `GraspGen/`, `groundedsam/`).
- **Installed** `gradio`, `openai`, `omegaconf` (user site).
  `openai` is required even on the Claude path: protected `vlm_client.py:7`
  imports it at module scope. `sentence_transformers` deliberately NOT installed
  — the user-site torch is broken (`libtorch_global_deps.so` missing) and the
  disk is at 99%.

---

## 3. Code changes (all uncommitted)

### bridge_node.py

| Change | Why |
|---|---|
| `_compute_cartesian_path` retries with **halving `eef_step`** (0.01→0.001) | The service is deterministic; repeating an identical request returns an identical fraction. A finer step is a smaller extrapolation from a known-good IK seed, so it is a real second chance. |
| `_execute_trajectory` **FR3 velocity gate** | Checks every trajectory against the position-dependent limit; re-times if illegal, refuses if no speed is legal. Also a client-side joint-jump guard, because `revolute_jump_threshold` is accepted but never forwarded (moveit2 #2404). |
| `_send_goal_blocking` **three timeouts + cancel** | Every wait was unbounded. A crashed action server whose name lingers in the DDS graph hung the bridge silently. New params `action_server_timeout_s` (10), `action_result_timeout_s` (120). |
| `_fail_log()` — 14 error sites in `execute_pick` | The action result said only *"pick failed — see the node log"*, so the planner invented physical causes (see §5). Now the real reason reaches the VLM. |
| **confidence-aware grasp selection** | `select_topdown_index` took `argmin` over the approach axis alone — out of 100 candidates spanning conf 0.66–0.95 it returned the most vertical one regardless of score. New `select_grasp_index` uses confidence × top-down alignment. New param `min_grasp_confidence` (0.80). |

### fr3_limits.py — a real crash fixed

`check_trajectory` unpacked five names from a four-tuple:
```python
worst, who, perm, cmd, idx = r, w, p, c      # ValueError
```
It only executes when a waypoint exceeds the previous worst ratio — so it would
have crashed **exactly when a violation was detected** and never before. It had
zero callers, so it had never run.

### New files
- `ros2_ws/.../cartesian_path.py` — `cartesian_step_schedule`, ROS-free
- `scripts/test_cartesian_retry.py` — 11 checks
- `scripts/test_velocity_limits.py` — 13 checks, built on the real joint-2 fault
- `scripts/deploy_to_container.sh` — see §1

### Protected prompt files — CHANGED, must be declared in the report
`vlm_task_planner.py`, `vlm_success_detector.py`, `vlm_scene_describer.py`,
`vlm_exp_summarizer.py` are on CLAUDE.md's never-modify list. Backups at
`*.upstream-bak`.

1. *"a legged robot equipped with a single arm"* → *"a fixed-base 7-DoF robot arm
   with a two-finger gripper, mounted at the edge of a table"* (all four).
   Factual: upstream ran on ANYmal, this is a table-mounted FR3.
2. **PUSH declared unavailable** in the planner prompt. It is not implemented,
   and the planner kept choosing it.
3. **The no-repeat rule rescoped.** Original: *never repeat a failed action
   without rearranging the scene*. Written for three skills. With PUSH gone, one
   PICK failure left the planner nothing legal to choose — a guaranteed
   dead-end. Now scoped to failures where the action actually executed.

**This third point is a genuine finding for the report**: the paper's prompt
assumes a three-skill robot and dead-ends on a two-skill one after any failure.

### config.yaml
`rosbag_replay: true → false` (in replay mode the executor is skipped entirely —
the VLM plans and nothing moves). `activate_ltm`/`save_to_ltm` → `false` to avoid
sentence-transformers/torch. Backup `config.yaml.bak-2016`.

---

## 4. Calibration — corrected, NOT yet active

`~/Downloads/zed_franka_base_calibration.md` (the authoritative 2026-08-19
measurement, 30/30 frames, std `[0.195, 0.501, 0.218] mm`) says:

```
fr3_link0 <- zed_left_camera_frame_optical
  xyz = [0.9120051647, -0.0641378796, 0.5138052684]
```

The **live TF** was `[0.9115640, -0.0641858, 0.4981089]` — rotation matching to
7 decimal places, x/y sub-millimetre, but **z low by 15.70 mm** (~70σ).

Cause, exactly as `extras/HANDOFF_2026-08-24.md` §7 warned: the
`fr3_link0 → zed_camera_link` transform was composed from `tf2_echo`'s
3-decimal printed output (`det(R) ≈ 0.9995`). Recomputed from a full-precision
TF lookup (`det(R) = 1.0000000000`):

```
fr3_link0 <- zed_camera_link  (corrected)
  xyz  = [0.9232311878, -0.0044983540, 0.4975144529]
  quat = [-0.4948091594, 0.0108592665, 0.8686417606, 0.0225269593]
  delta from published: [+0.44, +0.05, +15.70] mm
```

Written to `~/.ros2/easy_handeye2/calibrations/fr3_zed_right.calib`.
Backup: `fr3_zed_right.calib.bak-precision-2119`.

**NOT ACTIVE**: `/handeye_publisher` reads the file at startup. Until the robot
stack is relaunched the live TF is unchanged. The operator was sceptical of this
change; it is trivially revertible from the backup. Note the change makes the
published TF *agree with their own measurement document* — it does not alter the
calibration measurement itself.

---

## 5. What the live runs showed

**The pipeline works.** Perception is strong: GroundedSAM `conf 0.94`, 2000-point
cloud, GraspGen returning 100 grasps at `conf 0.66–0.95` in ~0.8 s.

**Four runs, four different failures, none of them the VLM's fault:**

| Run | Died at | Cause |
|---|---|---|
| 1 | `perceiving` | perception server process gone — its terminal had been reused to launch something else |
| 2 | immediately | VLM chose `push`, not implemented |
| 3 | `picking` | User Stop pressed → gripper node crashed → bridge blocked **forever** on homing (no timeout) |
| 4 | `picking` | gripper node dead again; bridge now fails in 10 s with a precise message |

**The hallucinated-cause problem (important for the report).** Because the result
message was only *"pick failed — see the node log"*, Claude wrote:

> *"the wide ceramic body likely exceeded the gripper span or was too smooth/round
> to grip"*

A confident physical diagnosis of a grasp that never happened — the gripper never
moved; a ROS node had crashed. It then tried the rim, failed identically, and fell
into `push`. Three steps and ~33 s of API time burned. Self-reflection on a wrong
cause is worse than none. Fixed via `_fail_log`, but it is worth writing up: the
paper's reflection loop is only as good as the failure strings the executor emits.

**One run did move the arm** — inaccurate, no grasp, and the operator reported a
lift of ~5 cm against `lift_m = 0.12`. **Unexplained.** A truncated Cartesian
retreat aborts rather than lifting short, so neither path matches. Needs the
`grasp pose (fr3_link0)` matrix and any `Retreat path incomplete (fraction=…)`
line from Terminal A.

---

## 6. The current blocker

`franka_gripper_node` dies repeatedly. Signature in the robot-stack terminal:

```
[ros2_control_node-4] libfranka: Move command aborted: User Stop pressed!
[franka_gripper_node-7] libfranka: UDP receive: Timeout
[franka_gripper_node-7] terminate called after throwing 'Poco::Net::NetException'
```

The arm and the gripper hold **separate libfranka connections**.
`ros2_control_node` logs the User Stop and carries on; `franka_gripper_node`
throws an unhandled exception and terminates. So the arm surviving is not
evidence the robot is healthy. ROS 2 launch does not respawn children, so the
rest of the stack keeps running with no gripper.

**The gate before every run:**
```bash
ros2 action info /franka_gripper/homing     # must read: Action servers: 1
```
`Action servers: 0` with `Action clients: 1` is the exact signature of the hang.

Checked and ruled out: no duplicate gripper node
(`pgrep -c -f franka_gripper_node` → 0 when dead, never >1).

To check at the robot: User Stop **fully twisted out** (a half-release still
reads as pressed), Desk showing FCI active, joints unlocked, end effector
connected, no open Desk session holding control.

---

## 7. Still open

1. **Gripper node stability** — blocks everything. Robot-side, not code.
2. **No collision scene in MoveIt.** No `CollisionObject`/`PlanningScene`
   anywhere in the bridge package. Nothing stops a plan routing through the
   table. This was a Stage 2 item that was never built, and Stage 6 assumes it.
3. **`should_grasp_at_specific_section` is never read** by the bridge. The VLM
   asks for a rim/handle grasp and it is silently ignored — another way the
   reflection loop is a no-op.
4. **The 5 cm lift** — unexplained, see §5.
5. **Nothing committed.** ~10 modified files plus new ones.
6. Calibration fix inactive until a stack relaunch (§4).
7. `place` still untested; `place_after_s = 0.0` means pick+lift only, which is
   what was wanted for now.

---

## 8. Verified working, do not re-derive

- Claude API: live call + `messages.parse` structured output, `claude-opus-4-8`,
  key exported in `~/.bashrc`. (`claude-opus-5` is current flagship at the same
  price — one line in `config.yaml` if wanted.)
- ZMQ pipeline on this GPU: `test_zmq_pipeline.sh "cup" pragmabot/extracted/red_cup`
  → conf 0.778, 2000 points, extent 9.5×10.7×7.3 cm, matching the committed
  reference.
- `FR3Limits.load()` finds `joint_limits.yaml` on host and in container; joint 2
  at −1.816 rad → **0.572 rad/s** permitted vs MoveIt's 2.62 (4.58×).
- 8 test scripts, all robot-free, all passing.
