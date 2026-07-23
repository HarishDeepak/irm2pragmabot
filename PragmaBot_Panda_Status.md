# PragmaBot × Franka Panda — Project Status
**PEARL Lab, TU Darmstadt | IRM2 | Timeline: Apr 24 – Sep 30, 2026**
**Repo:** `https://github.com/HarishDeepak/IRM2` | **Branch:** `feature/panda-adaptation`
**Status as of:** 2026-07-09

---

## 1. Project Summary

PragmaBot (ETH Zurich RSL, IEEE RAL 2026) is a VLM-driven task planning framework: GPT-4o plans manipulation actions, self-reflects on failures via short-term memory (STM), and distills successful episodes into long-term memory (LTM) for retrieval-augmented generation (RAG) on future tasks — no model fine-tuning. Upstream code ships the full planner/STM/LTM/RAG pipeline; **action execution is left as `NotImplementedError`** (built for an ANYmal quadruped + 6-DoF arm, Robotiq 2F-140 gripper, arm-mounted ZED X Mini, Pinocchio IK).

**Task:** rebuild the entire execution/perception layer for a **Franka Emika Panda** with a **static ZED2** camera, while leaving planner/STM/LTM/RAG logic untouched.

**Reusable unchanged:** `vlm_task_planner.py`, `vlm_success_detector.py`, `vlm_scene_describer.py`, `vlm_exp_summarizer.py`, `memory_manager.py`, overall algorithmic loop.

**Must be rebuilt:** everything downstream of the planner's `NextBestAction` output — object grounding, grasp generation, motion execution, gripper control.

---

## 2. ⚠️ Architecture Pivot (current state supersedes all earlier planning)

The original plan (Sections 3–4 below) assumed **ROS1 Noetic + MoveIt**, matching the lab's `franka_zed_gazebo` / `3liyounes/pearl_robots:franka` Docker stack. This is **no longer viable**:

| Root cause | Detail |
|---|---|
| Robot firmware | v5.9.0, requires libfranka ≥ 0.15 |
| `frankarobotics/franka_ros2` | Dropped FER/Panda support entirely — ships only `franka_fr3_moveit_config` |
| Community Panda forks | All target old libfranka (0.8–0.10), incompatible with current firmware |

**Agreed replacement — dual-container architecture ("Path A")**, built by forking a lab colleague's ("rickmer") already-working containers rather than assembling a new stack from scratch:

| Container | Fork of | Base | Key stack | Role |
|---|---|---|---|---|
| Control | `rickmer-ros2_polymetis-1` | ROS2 Foxy | libfranka 0.18.2, Polymetis 0.2, `role_ros2` | Robot motion control |
| Perception | `rickmer-ros2_cu118-1` | ROS2 Humble | ZED SDK 4.1, zed-ros2-wrapper, CUDA 11.8 | Camera / vision |

Both share `ROS_DOMAIN_ID=7` for cross-container topic visibility.

**MoveIt plan+execute is replaced by `role_ros2` ROS2 services**, called from `PandaSkillExecutor`:
- `/panda_arm/move_to_joint_positions`
- `/panda_arm/move_to_ee_pose` — IK handled internally by `role_ros2`'s MuJoCo/dm_control-based `RobotIKSolver` (do **not** add a separate IK layer)
- `/panda_gripper/gripper/grasp`
- `/panda_gripper/gripper/open`

**Known config fix required:** `gripper_type` must be overridden from default `robotis` → `franka_hand`.

Everything in Sections 3–4 that references MoveIt/ROS1/`franka_gripper`/actionlib is **historical context**, not the active plan — kept here for the parts that still hold (perception logic, primitive definitions, memory-layer boundary).

---

## 3. Current Implementation Phase Status

| Phase | Description | Status |
|---|---|---|
| 0 | Architecture selection (ROS1/MoveIt → ROS2/Polymetis pivot) | **Done** |
| 1 | `docker inspect` both rickmer containers → confirm network mode / `ROS_DOMAIN_ID` cross-visibility | **Next up** |
| 2 | Read `role_ros2/robot_ik/robot_ik_solver.py` → verify MuJoCo model is Panda-correct | Not started |
| 3 | Fork both containers, begin adapting `PandaSkillExecutor` to `role_ros2` service interface | Not started |
| 4 | Perception container integration — ZED2 static TF calibration, observation pose definition | Not started |
| 5 | Port pick/place/push dispatch logic to `role_ros2` services | Not started |
| 6 | Segmentation + grasp stack install (GroundedSAM, GraspGen) | Not started |
| 7 | Wire `PandaSkillExecutor` into `pragmabot_node.py` (replace `NotImplementedError`) | Not started |
| 8 | STM/LTM/RAG end-to-end validation on real robot | Not started |

**Session paused** — Harish stopped after the pivot to study findings before proceeding to Phase 1.

**Open blockers:**

| Blocker | Blocks |
|---|---|
| Physical robot access (observation pose definition) | Phase 4+ |
| OpenAI API key with paid credits (not ChatGPT Plus) | Phase 7+ (full VLM loop) |
| GroundedSAM install | Phase 6 |
| GraspGen install + offline spike | Phase 6 |
| Supervisor confirmation: is a full ROS2 port of PragmaBot planner nodes in scope, or only the control layer? | Scoping of remaining work |

---

## 4. Superseded Plan — ROS1 / MoveIt (historical reference)

Kept for parts still valid: perception pipeline logic, `.action` schema shapes, primitive definitions, GroundedSAM/GraspGen selection rationale.

### 4.1 Original environment (no longer the active container)
- Container: `3liyounes/pearl_robots:franka`, ROS1 Noetic, Ubuntu 20.04, Python 3.8, catkin workspace `/catkin_ws/src/pragmabot`
- Confirmed present: `panda_moveit_config`, `franka_gripper`, MoveIt (KDL IK), `actionlib`
- Confirmed absent: ZED2 ROS wrapper, GroundedSAM, GraspGen, TRAC-IK, Pinocchio (not needed — MoveIt IK was the plan)

### 4.2 Original architecture (superseded — MoveIt/actionlib layer no longer used)

```
pragmabot_node.py (Gradio UI + VLM loop)
        │
        ▼
panda_skill_executor.py (action clients)
        ├──► /panda/pick  → PandaPickServer  (perception → GroundedSAM → GraspGen → MoveIt → gripper)
        ├──► /panda/place → PandaPlaceServer (MoveIt above target → descend → open gripper → retreat)
        └──► /panda/push  → PandaPushServer  (GroundedSAM → push vector → MoveIt Cartesian path)
```
Rationale (still valid conceptually, now applies to `role_ros2` services instead of MoveIt): pick/place/push motions block 10–20s; isolating execution from the Gradio/VLM/STM/LTM process prevents a motion failure from crashing the planning loop, and allows independent restart + live feedback.

### 4.3 Perception design (still valid)

- **Camera:** ZED2, static mount facing table (not arm-mounted, unlike original ANYmal setup) — simplifies to a one-time extrinsic calibration: static TF `zed2_left_camera_frame` → `panda_link0`.
- **Observation pose:** fixed arm joint configuration that clears the camera FOV before every scene capture; must be recorded once physical access is available.
- **Segmentation:** GroundedSAM (Grounding DINO + SAM), not SAM2 — SAM2 alone cannot do open-vocabulary text→mask; GroundedSAM takes `target_object` (a resolved noun from the planner, e.g. "red cup") + RGB → binary mask. It never sees the raw user instruction; spatial language ("the object on the left") is resolved upstream by `VLMSceneDescriber`.
- **Grasp generation:** **GraspGen (NVlabs)** chosen over AnyGrasp — AnyGrasp's binary license is machine-locked and breaks in Docker; GraspGen ships a pretrained Franka Panda model, open weights, and a client-server HTTP mode suited to a robot pipeline.

```
User instruction → VLMSceneDescriber → concrete scene description
                                          ↓
                                   VLMTaskPlanner → target_object = "red cup"
                                                          ↓
                                               GroundedSAM("red cup", rgb) → mask
                                                          ↓
                                               mask + depth → point cloud → GraspGen
```

### 4.4 VLM output schema (unchanged, still active)

```python
class NextBestAction(BaseModel):
    reasoning: str          # chain-of-thought — VLM reasons here BEFORE committing
    skill: str              # "pick" | "place" | "push" | "done"
    target_object: str
    target_location: str | None = None
    use_annotation: bool = False
```
The `reasoning` field must come first in the schema — forces self-reflection before the skill choice commits; omitting it degrades planning quality (structured-output constraint otherwise skips reasoning).

### 4.5 Original `.action` definitions (ROS1-specific; will need a ROS2 equivalent — service/action interface — under `role_ros2`)

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

### 4.6 Files touched vs untouched (original plan; module list still roughly applicable under new stack)

| File | Status | Change |
|---|---|---|
| `nodes/pragmabot_node.py` | Minimal edit | Replace `NotImplementedError` with `executor.execute(action)` |
| `src/pragmabot/vlm_task_planner.py` | Edit | Add `reasoning` field, use `.parse()` |
| `src/pragmabot/panda_skill_executor.py` | **New** | Dispatches pick/place/push (now: to `role_ros2` services) |
| `src/pragmabot/grounded_sam.py` | **New** | GroundedSAM wrapper |
| `src/pragmabot/graspgen_client.py` | **New** | GraspGen HTTP client |
| `nodes/panda_pick_server.py` / `panda_place_server.py` / `panda_push_server.py` | **New** | Per-skill execution loops (ROS1 actionlib in original plan — needs ROS2 equivalent) |
| `requirements.txt` | Edit | Add `groundingdino`, `graspgen` |
| Everything else in `src/pragmabot/` | Unchanged | `vlm_client.py`, `vlm_success_detector.py`, `vlm_scene_describer.py`, `vlm_exp_summarizer.py`, `memory_manager.py`, `scene_observer.py` |

---

## 5. Gap Analysis — Reuse vs. Adapt

| Component | Original (PragmaBot/ANYmal) | Panda-side replacement | Status |
|---|---|---|---|
| Robot platform | ANYmal + 6-DoF arm | Franka Panda (fixed-base, no locomotion) | Pivoted to ROS2/Polymetis |
| Motion execution | Original robot backend | `role_ros2` services (was: MoveIt) | Not started |
| Gripper | Robotiq 2F-140 | Franka Hand — **must set `gripper_type: franka_hand`** (default is `robotis`) | Known fix, not applied |
| Camera input | ZED X Mini, arm-mounted | ZED2, static mount facing table | Not started |
| Coordinate transforms | Original robot frames | Static TF `zed2_left_camera_frame → panda_link0` | Not started |
| IK / kinematics | Pinocchio-based filtering | `role_ros2`'s internal MuJoCo/dm_control `RobotIKSolver` (was: MoveIt `compute_ik`/KDL) | Needs verification (Phase 2) |
| Skill execution API | Original primitives | Panda wrapper via `role_ros2` services | Not started |
| Workspace assumptions | Mobile robot workspace | Tabletop Panda workspace | Prompts/safety constraints not yet updated |
| Segmentation | Part of shared annotation tool | GroundedSAM (Grounding DINO + SAM) | Not installed |
| Grasp generation | AnyGrasp | GraspGen (NVlabs), Franka Panda pretrained | Not installed/tested |

**Fully reusable, no changes needed:** VLM planner concept/prompt structure, success detector concept, STM loop, LTM storage logic, RAG pipeline, overall algorithmic flow.

---

## 6. Primitives Reference

| Primitive | Inputs | Internal steps |
|---|---|---|
| `pick(target_object, use_annotation)` | object name, bool | observation pose → GroundedSAM → point cloud → GraspGen → IK filter → move to grasp → gripper close |
| `place(target_location)` | location string/pose | move above target → descend → gripper open → retreat |
| `push(target_object, goal_region)` | object name, goal region | GroundedSAM both → compute push vector → Cartesian-style path execution |
| `move_to_observation_pose()` | — | move to fixed joint config, clears camera FOV |
| `open_gripper()` / `close_gripper()` | — | `/panda_gripper/gripper/open` or `/grasp` |
| `home()` | — | move to safe home joint configuration |
| `recover()` | — | error recovery (utility, not yet designed) |

---

## 7. Key Learnings & Principles

- **FER/Panda + modern libfranka incompatibility is real and unresolvable via `franka_ros2`** — don't retry that path; community forks all target stale libfranka.
- **Reuse over rebuild:** forking rickmer's already-working containers is far lower risk than assembling a merged ROS2+libfranka+ZED stack from scratch.
- **IK lives inside `role_ros2`** (`move_to_ee_pose` calls `RobotIKSolver` internally) — do not add a separate IK layer on top.
- **AnyGrasp is disqualified** (machine-locked binary license breaks in Docker); **GraspGen** is the settled choice.
- **SAM2 alone cannot do open-vocabulary text-to-mask** — GroundedSAM is required for text-driven segmentation.
- **MoveIt's `compute_ik`** is available in the (old) container without TRAC-IK/Pinocchio, relevant only as a ROS1 fallback if ever needed.
- **Docker fundamentals:** flags like `NVIDIA_DRIVER_CAPABILITIES` must be set at `docker run` time, not added post-creation.
- **Fabrication risk flagged previously:** a Docker image tag was fabricated and the FER incompatibility wasn't proactively caught — always flag uncertainty rather than guess.

---

## 8. Working Preferences (for anyone picking this up)

- Technical, direct, concise — no fluffy summaries.
- When code is pasted: explicitly flag robot-specific assumptions, distinguish reusable vs. must-adapt, suggest Panda-side replacements.
- Structured outputs: tables for status/comparisons, Python skeletons for code.
- Phased, plan-before-implement: explicit design docs + done-conditions per phase before writing code; one phase per session, manual verification before proceeding.
- Single `panda_skill_executor.py` inside the existing `pragmabot` package (not a separate package) until complexity demands otherwise.

---

## 9. Lab Environment Reference

- **Machine:** Alonnisos — RTX 4080, driver 535.183.01
- **Robot IP:** 10.10.10.10
- **ROS2 workspace:** `~/ros2_ws/franka_ros2`
- **Reference logs:** Hydra run logs at `config/outputs/2026-04-21/` on Alonnisos confirm the Polymetis container ran against the real robot with the Franka Hand gripper.

---

## 10. Immediate Next Steps

1. `docker inspect` both rickmer containers (control + perception) → confirm network mode and `ROS_DOMAIN_ID=7` cross-container topic visibility.
2. Read `role_ros2/robot_ik/robot_ik_solver.py` → confirm the MuJoCo model matches the real Panda kinematics.
3. Fork both containers under the project repo; begin adapting `PandaSkillExecutor` to call `role_ros2` services (`move_to_joint_positions`, `move_to_ee_pose`, `gripper/grasp`, `gripper/open`).
4. Apply the known `gripper_type: franka_hand` config fix.
5. Once containers are confirmed working: proceed to perception container integration (ZED2 static TF calibration) and GroundedSAM/GraspGen install.
6. Raise open scoping question with supervisor: full ROS2 port of PragmaBot planner nodes, or control-layer-only port with planner staying on its original stack.

---

*Source docs synthesized: project memory log (architecture pivot, most current), `pragmabot_panda_plan.md`, `handoff.md`, `panda_gap_analysis.md`, upstream `README.md`, `requirements.txt`.*
