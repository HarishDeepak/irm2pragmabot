# PragmaBot × Franka Panda — Full Status Handoff

**Purpose of this file:** a single, complete, self-contained brief for taking this project to a fresh Claude chat (or any new collaborator) alongside other lab MDs, to get a plan for what comes next. It supersedes/consolidates `PragmaBot_Panda_Status.md`, `CLAUDE_CODE_PLAN.md`, `notes.md`, `SETUP.md`, and `panda_notes/*.md` — those are kept as historical detail, this is the "read this first" document.

**Repo:** `https://github.com/HarishDeepak/IRM2` · **Branch:** `feature/panda-adaptation` · **Base:** ETH Zurich RSL's PragmaBot (IEEE RAL 2026) · **Lab:** PEARL Lab, TU Darmstadt (IRM2 project) · **Timeline:** Apr 24 – Sep 30, 2026 · **As of:** 2026-07-10

---

## 1. The one thing to understand before anything else

**There are two different, non-overlapping bodies of work in this repo, built in two different sessions, and they contradict each other on robot architecture:**

1. **What is actually committed and working in code right now** (commits `a7b710d`, `e8cfca5`, `4227629`): a full **ROS1 Noetic + MoveIt + actionlib** execution layer for the Panda, built and tested in **rosbag replay mode only** (no real robot, no real camera — synthetic/logged data). This is complete end-to-end at the software level.

2. **What the most recent planning session (`PragmaBot_Panda_Status.md`, uncommitted, 2026-07-09) decided must happen instead**: the ROS1/MoveIt path is **dead** for the physical robot, because the lab's Panda firmware/libfranka version cannot be driven by any ROS1 or `franka_ros2`-based stack available. The agreed replacement is a **ROS2 dual-container architecture** (Polymetis-based control + separate perception container), forked from a labmate's ("rickmer") working setup. **Zero code has been written for this yet** — it's a decision, not an implementation.

So: **the perception/segmentation/grasp-generation logic and the VLM/memory pipeline are solid and reusable. The motion-execution layer (MoveIt, actionlib, the three `.action` files, the three action servers) is fully implemented but built against an architecture that cannot run on the real robot and will need to be replaced or substantially rewired.** Any new plan needs to treat these as separate tracks.

---

## 2. What PragmaBot is (unchanged, for context)

VLM-driven robotic task-learning system. Runs a 7-step loop per task:

1. **VLMSceneDescriber** — VLM describes the camera scene in natural language
2. **MemoryManager** — retrieves top-k similar past episodes from LTM via embedding cosine similarity (RAG)
3. **VLMTaskPlanner** — picks next skill (`pick`/`place`/`push`/`done`) + target, with an explicit `reasoning` field forcing chain-of-thought before commitment
4. **Action Execution** — the integration point this whole project exists to fill
5. **VLMSuccessDetector** — compares before/after images, decides success/failure
6. **STM update** — appends `(action, evaluation)`; loops to step 3 on failure
7. **VLMExperienceSummarizer** — on completion, distills the episode into an LTM entry

**Hard rule that has been followed throughout:** the upstream VLM/memory files are never modified except for a small, fixed set of edits in `pragmabot_node.py`. Untouched: `vlm_client.py`, `vlm_task_planner.py`, `vlm_success_detector.py`, `vlm_scene_describer.py`, `vlm_exp_summarizer.py`, `memory_manager.py`, `scene_observer.py`, `conversation_builder.py`.

---

## 3. What is actually built and committed (Phases 1–6, ROS1/MoveIt track)

All of this exists in the repo today, builds, and has been exercised in **rosbag replay mode** (`rosbag_replay: true` in `config.yaml` — bypasses real execution triggers, but the code paths below were also unit-tested standalone: import, instantiate, `.execute()` on fake goals).

### 3.1 New files added
| File | Role | Status |
|---|---|---|
| `pragmabot/action/PandaPick.action`, `PandaPlace.action`, `PandaPush.action` | ROS1 action message definitions | Done, builds |
| `pragmabot/nodes/panda_pick_server.py` | ROS1 `actionlib` server: observation pose → capture RGBD → GroundedSAM → point cloud → GraspGen → MoveIt execute → gripper close → lift | Done (placeholder constants, see §5) |
| `pragmabot/nodes/panda_place_server.py` | Observation pose → capture → GroundedSAM on target location → centroid → 3D point → MoveIt → descend → open gripper → retreat | Done (placeholders) |
| `pragmabot/nodes/panda_push_server.py` | GroundedSAM on object + goal region → push vector → MoveIt Cartesian path | Done (placeholders) |
| `pragmabot/src/pragmabot/panda_skill_executor.py` | `actionlib.SimpleActionClient` bridge; dispatches `NextBestAction` → the three action servers over `/panda/pick`, `/panda/place`, `/panda/push` | Done |
| `pragmabot/src/pragmabot/grounded_sam.py` | GroundingDINO (open-vocab text→box) + SAM ViT-H (box→mask) wrapper | Done, tested (0.592 confidence on demo image) |
| `pragmabot/src/pragmabot/graspgen_client.py` | **ZMQ** client (not HTTP — plan doc is stale here) to a GraspGen server running on the **host machine**, port 5556, MsgPack-serialized point cloud → ranked SE(3) grasp poses | Done, not yet tested against a live GraspGen server |
| `pragmabot/launch/launch_panda_servers.launch` | Launches all three action servers | Done |
| `scripts/test_grounded_sam.py` | Offline GroundedSAM validation script | Done, passing |

### 3.2 Edits to `pragmabot_node.py` (the only file that touches upstream code)
Exactly as constrained — import + instantiate `PandaSkillExecutor`, replace both `NotImplementedError` sites (action execution, and re-planning after evaluation) with real calls. Verified via `git diff` — no other upstream logic touched.

### 3.3 VLM backend abstraction (separate, later piece of work — commit `4227629`)
Originally OpenAI-only. Now three interchangeable backends selected by substring match on `config.yaml`'s `vlm.vlm_model`:
- `vlm_client.py` (OpenAI, untouched, original) — `chat.completions.parse()`, `text-embedding-3-large` (3072D)
- `claude_vlm_client.py` (new) — `messages.parse()` with adaptive thinking; **no embeddings API**, falls back to local `sentence-transformers` (`all-MiniLM-L6-v2`, 384D)
- `gemini_vlm_client.py` (new) — `GenerativeModel` + JSON schema response; native embeddings via `text-embedding-004` (768D)

**Load-bearing gotcha:** these three produce **different embedding dimensions**, so LTM CSVs are not portable across backends — switching backends requires a fresh LTM or keeping separate CSV sets. Currently configured backend: `claude-opus-4-8` (see `config.yaml`).

### 3.4 What is explicitly a placeholder, not a bug
- `OBSERVATION_JOINT_CONFIG = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]` in all three servers — needs real robot measurement
- Camera intrinsics `fx=fy=700` — needs real ZED2 calibration values
- No `static_zed2_tf.launch` yet — extrinsic calibration (`zed2_left_camera_frame` → `panda_link0`) not done
- GraspGen server has never been started/connected to live — only the client exists

---

## 4. What the latest planning session decided (ROS2 pivot, no code yet)

Documented in `PragmaBot_Panda_Status.md` (uncommitted). Summary:

**Root cause:** the lab's physical Panda runs firmware v5.9.0, requiring libfranka ≥ 0.15. `frankarobotics/franka_ros2` has **dropped Panda/FER support entirely** (ships only FR3 configs now), and every community Panda fork targets libfranka 0.8–0.10. There is no viable ROS1 + MoveIt + `franka_gripper`/actionlib path to the real robot with current firmware — this had already been assumed workable and is now known not to be.

**Agreed replacement ("Path A"):** fork two of a labmate's (rickmer) already-working containers rather than build a new stack from scratch:

| Container | Fork of | Base | Stack | Role |
|---|---|---|---|---|
| Control | `rickmer-ros2_polymetis-1` | ROS2 Foxy | libfranka 0.18.2, Polymetis 0.2, `role_ros2` | robot motion |
| Perception | `rickmer-ros2_cu118-1` | ROS2 Humble | ZED SDK 4.1, zed-ros2-wrapper, CUDA 11.8 | camera/vision |

Both share `ROS_DOMAIN_ID=7` for cross-container topic visibility.

**Key implication for this codebase:** MoveIt `plan_and_execute` + `franka_gripper` actionlib is replaced by `role_ros2` ROS2 services:
- `/panda_arm/move_to_joint_positions`
- `/panda_arm/move_to_ee_pose` — **IK is handled internally** by `role_ros2`'s MuJoCo/dm_control `RobotIKSolver`; do not add a separate IK layer on top (this was flagged explicitly — don't repeat past MoveIt-`compute_ik`-style layering here)
- `/panda_gripper/gripper/grasp`
- `/panda_gripper/gripper/open`

Known required config fix once forked: `gripper_type` defaults to `robotis` in `role_ros2`, must be overridden to `franka_hand`.

**What survives the pivot unchanged:** the VLM planner/STM/LTM/RAG stack (Section 3.3 above, all of it), the primitive definitions (pick/place/push semantics), the perception logic (GroundedSAM choice and design), the GraspGen choice, and the `.action`-schema-shaped thinking (needs a ROS2 equivalent — service or action interface — under `role_ros2`, not a literal reuse of the `.action` files).

**What does not survive:** the three ROS1 action servers' MoveIt calls, `panda_skill_executor.py`'s `actionlib.SimpleActionClient` usage (needs to become ROS2 service/action clients), the `franka_gripper` message types, anything assuming a single ROS1 master.

**Open scoping question raised but not answered:** should the PragmaBot planner/Gradio node itself also move to ROS2, or does it stay on ROS1 and bridge to the ROS2 control/perception containers (e.g. via `ros1_bridge`, or by having `PandaSkillExecutor` in a small ROS2 client node that talks back to the ROS1 Gradio process over a non-ROS channel like ZMQ — which is already the pattern used for GraspGen)? **This needs a supervisor decision before Phase-2-of-the-pivot work starts.**

### Status table from the pivot session
| Phase | Description | Status |
|---|---|---|
| 0 | Architecture selection (ROS1/MoveIt → ROS2/Polymetis) | Done |
| 1 | `docker inspect` both rickmer containers → confirm network mode / `ROS_DOMAIN_ID` cross-visibility | **Next up** |
| 2 | Read `role_ros2/robot_ik/robot_ik_solver.py` → verify MuJoCo model is Panda-correct | Not started |
| 3 | Fork both containers, adapt `PandaSkillExecutor` to `role_ros2` service interface | Not started |
| 4 | Perception container integration — ZED2 static TF, observation pose | Not started |
| 5 | Port pick/place/push dispatch to `role_ros2` services | Not started |
| 6 | GroundedSAM/GraspGen install in new containers | Not started |
| 7 | Wire executor into `pragmabot_node.py` (already done once for ROS1 — needs redo for ROS2 client) | Not started |
| 8 | STM/LTM/RAG end-to-end validation on real robot | Not started |

Session was paused here deliberately — Harish stopped to study the pivot before starting Phase 1 of it.

---

## 5. Dependencies — full picture

### 5.1 Python packages actually imported by new code but **not yet added to `requirements.txt`**
This is a real gap, not a style nit — `requirements.txt` still only lists the pre-pivot core (`numpy`, `opencv-python`, `openai`, `gradio`, `omegaconf`, `pydantic`, etc.). Missing:
- `pyzmq`, `msgpack`, `msgpack-numpy` — used by `graspgen_client.py`
- `sentence-transformers` — used by `claude_vlm_client.py`'s embedding fallback
- `anthropic` — used by `claude_vlm_client.py`
- `google-generativeai` — used by `gemini_vlm_client.py`
- `groundingdino` (editable install from `/tmp/GroundingDINO`, **not pip-published**) and `segment-anything` — used by `grounded_sam.py`

### 5.2 System/container-level dependencies (ROS1 track, current container `pragmabot_panda` / `3liyounes/pearl_robots:franka`)
- ROS Noetic, Ubuntu 20.04, Python 3.8
- Present: `panda_moveit_config`, `franka_gripper`, MoveIt (KDL IK), `actionlib`
- Installed during this project: GroundingDINO (`/tmp/GroundingDINO` — **not persisted, lost on every container restart, must `pip install -e .` again**; weights persist under `/catkin_ws/src/pragmabot/weights/`, ~3.2GB total), `segment-anything`
- **Not installed in-container:** GraspGen — it runs on the **host machine** (needs Python 3.10 + CUDA, incompatible with the container's Python 3.8), communicating over ZMQ. This split-process design is deliberate, not a workaround.
- CPU-only inference for GroundedSAM currently — the CUDA extension fails to compile against this container's Python 3.8 + old CUDA combination.

### 5.3 External API keys
- `OPENAI_API_KEY` (paid credits, not ChatGPT Plus — flagged previously as a real blocker) if using GPT-4o path
- `ANTHROPIC_API_KEY` if using Claude path (currently configured default)
- `GOOGLE_API_KEY` if using Gemini path

### 5.4 Dependencies for the ROS2 pivot (not yet started, per §4)
- Two Docker containers to fork: `rickmer-ros2_polymetis-1` (ROS2 Foxy, libfranka 0.18.2, Polymetis 0.2), `rickmer-ros2_cu118-1` (ROS2 Humble, ZED SDK 4.1, CUDA 11.8)
- `role_ros2` package (source of the motion/gripper services and the internal IK solver)
- Physical robot: firmware v5.9.0, IP `10.10.10.10`
- Compute: Alonnisos machine, RTX 4080, driver 535.183.01

### 5.5 Disqualified / dead-end dependencies (don't re-investigate these)
- `franka_ros2` official Panda/FER support — dropped upstream
- Community libfranka forks targeting 0.8–0.10 — incompatible with firmware v5.9.0 requiring ≥0.15
- AnyGrasp — binary license is machine-locked, breaks in Docker; GraspGen was chosen instead
- SAM2 alone — no open-vocabulary text→mask; GroundedSAM (Grounding DINO + SAM) required for text-driven segmentation

---

## 6. Action files detail (the ROS1 `.action` definitions, current track)

```
# PandaPick.action
string target_object
bool use_annotation
---
bool success
string message
geometry_msgs/PoseStamped grasp_pose
---
string status

# PandaPlace.action
string target_location
geometry_msgs/PoseStamped place_pose
---
bool success
string message
---
string status

# PandaPush.action
string target_object
string goal_region
---
bool success
string message
---
string status
```

These generate `pragmabot.msg.PandaPickAction` etc. via `add_action_files` + `generate_messages` in `CMakeLists.txt` (catkin's standard actionlib codegen). **Under the ROS2 pivot these need a `role_ros2`-compatible equivalent** — likely a ROS2 `.action` (same three-section goal/result/feedback shape is directly portable) or plain services if `role_ros2` doesn't expose an action-server pattern for these calls. This has not been designed yet; it's listed as part of pivot Phase 5 in §4.

---

## 7. ROS1 → ROS2 feasibility — direct answer to "can this whole project move to ROS2"

**Short answer: partially forced already, and yes it's feasible, but it's a real port, not a recompile.** Three sub-questions, answered separately because they have different answers:

### 7.1 Can the motion/gripper/perception execution layer move to ROS2? — **Yes, already decided, already in progress**
This is exactly what §4 describes. Not a hypothetical — it's the current plan, because ROS1 + MoveIt physically cannot drive this robot's firmware. The `role_ros2`/Polymetis stack replaces MoveIt's role entirely, including IK (handled internally, do not re-add a separate IK layer).

### 7.2 Can/should the PragmaBot planner node (`pragmabot_node.py`, Gradio UI, VLM/STM/LTM loop) move to ROS2? — **Feasible, not yet decided, genuinely optional**
Nothing in the planner stack is ROS1-specific in a deep way:
- `scene_observer.py` uses `message_filters.ApproximateTimeSynchronizer` — has a direct ROS2 equivalent (`message_filters` is available for ROS2 too)
- `rospy.loginfo`/`rospy.Time` calls scattered through `pragmabot_node.py` and the new Panda files map 1:1 to `rclpy` equivalents (`self.get_logger().info`, `self.get_clock().now()`) — mechanical, not conceptual, work
- Gradio itself is ROS-version-agnostic; it's just a Python process with a UI thread
- `actionlib.SimpleActionClient` → `rclpy.action.ActionClient` is the main structural change, and it's already isolated in `panda_skill_executor.py` — a single, well-bounded file
- The catkin `package.xml`/`CMakeLists.txt` → `ament`/`colcon` `package.xml`/`CMakeLists.txt` conversion is boilerplate for a package this size (one package, few messages)

The **actual risk** is not the port mechanics — it's whether porting the planner is in scope at all. The pivot session explicitly flagged this as an open question for the supervisor: *"full ROS2 port of PragmaBot planner nodes, or control-layer-only port with planner staying on its original stack, bridged to the ROS2 containers"*. A bridge (`ros1_bridge`, or the ZMQ-style pattern already used for GraspGen) is a legitimate lower-effort alternative to a full planner port, and is arguably lower-risk given the planner code is meant to stay untouched/upstream-compatible.

### 7.3 What would NOT survive a full ROS2 port unchanged
- All three current ROS1 action servers' MoveIt calls — full rewrite against `role_ros2` services (already required regardless of whether the planner ports, per §4)
- `panda_skill_executor.py` — rewrite from `actionlib` to `rclpy.action`/service clients
- `franka_gripper` action messages — replaced by `role_ros2`'s gripper services
- The three `.action` files — either re-expressed as ROS2 `.action`/`.srv` or dropped in favor of calling `role_ros2` services directly from `panda_skill_executor.py` without an intermediate per-skill action server layer (worth considering — the original three-server split existed to isolate blocking motion from the Gradio/VLM process; `role_ros2` services may already provide that isolation, making the extra server layer redundant)
- `catkin build` / `catkin_make` workflow → `colcon build`
- Python 3.8 constraint — ROS2 Foxy/Humble containers use newer Python (3.8/3.10 respectively per the two forked containers), so the "no walrus operator, no 3.10+ syntax" constraint from `CLAUDE_CODE_PLAN.md` may actually be safe to relax once on the new containers — worth confirming per-container Python version before assuming this still applies

### 7.4 Recommendation to bring to the planning chat
Don't frame this as "should we port to ROS2" — that decision is already made for the control layer. Frame it as: **(a)** confirm whether the planner/Gradio process should also become a ROS2 node or stay ROS1-and-bridge, given the untouched-upstream-code constraint likely favors *not* touching its ROS1 assumptions; **(b)** decide whether the three-action-server split (pick/place/push as separate processes) is still worth keeping under `role_ros2`, or whether `panda_skill_executor.py` should call `role_ros2` services directly, since the original rationale (isolating blocking motion from the planning loop) may already be handled by `role_ros2`'s service model.

---

## 8. Known gotchas and prior mistakes (don't re-discover these)

- `catkin build` fails due to a build-space conflict in this container — use `catkin_make --pkg pragmabot` instead
- GroundingDINO must be reinstalled every container restart (`/tmp` is not persisted); weights under `/catkin_ws/src/pragmabot/weights/` do persist
- `sentence-transformers` needs a separate pip install for the Claude backend's embedding fallback
- LTM CSVs are embedding-model-specific — don't mix across VLM backends (3072D OpenAI vs 384D Claude-fallback vs 768D Gemini)
- Gemini's assistant role is `"model"`, not `"assistant"`
- Claude's system prompt is a top-level API parameter, not a message in the `messages` list; `client.messages.parse()` requires `max_tokens` explicitly (no default)
- A Docker image tag was fabricated in an earlier session, and the FER/firmware incompatibility wasn't proactively caught before it became a blocker — flag uncertainty rather than guess, especially about container/robot compatibility
- Stale `__pycache__` artifacts exist at `pragmabot/src/pragmabot/panda_adapter/__pycache__/` (for `object_detector.py`, `grasp_estimator.py`, an older `panda_skill_executor.py`) with **no corresponding source files** — these are leftovers from an earlier, since-abandoned module layout (a separate `panda_adapter` subpackage was tried, then consolidated directly into `src/pragmabot/`). Harmless but should be deleted as cleanup; don't mistake them for missing files.
- `IRM1/` directory at repo root is empty — likely a placeholder for a previous IRM1-project workspace/submodule that was never populated in this checkout.

---

## 9. Immediate next steps (both tracks)

**ROS1/current-code track (can continue without lab access):**
1. Add the missing packages to `requirements.txt` (§5.1)
2. Start a GraspGen server on the host and validate the ZMQ round-trip from `graspgen_client.py` for the first time — it has never been tested against a live server
3. Clean up stale `panda_adapter/__pycache__`

**ROS2 pivot track (the actual path to the physical robot):**
1. `docker inspect` both rickmer containers → confirm network mode and `ROS_DOMAIN_ID=7` cross-container visibility
2. Read `role_ros2/robot_ik/robot_ik_solver.py` → confirm the MuJoCo model is Panda-correct (not FR3 or a different variant)
3. Fork both containers into the project; begin adapting `PandaSkillExecutor` to `role_ros2`'s service interface
4. Apply the known `gripper_type: franka_hand` config fix
5. Raise the scoping question from §7.4 with the supervisor before investing in a full planner port

**Lab-access-only items (either track):**
1. Measure real observation joint config (jog arm out of camera FOV, read `/joint_states`)
2. ZED2 → `panda_link0` static TF calibration
3. Real camera intrinsics (replace `fx=fy=700` placeholder)

---

## 10. Source documents this file consolidates
`PragmaBot_Panda_Status.md` (pivot decision, most current architecture), `CLAUDE_CODE_PLAN.md` (original ROS1 phase-by-phase implementation plan, Phases 1–6 executed from this), `notes.md` (running technical log, VLM backend details), `SETUP.md` (install steps, currently ROS1-only), `panda_notes/panda_design_spec.md`, `panda_notes/panda_gap_analysis.md`, `panda_notes/pragmabot_panda_project_setup.md` (earlier planning docs, largely superseded by `notes.md` and actual commits but retained for perception/grasp design rationale), git history on `feature/panda-adaptation`, and the auto-memory project log.
