# PragmaBot × Franka Panda — Full Project Overview

**Repo:** `HarishDeepak/IRM2` · **Branch:** `feature/panda-adaptation` · **Base project:** [leggedrobotics/pragmabot](https://github.com/leggedrobotics/pragmabot) (ETH Zürich RSL, IEEE RAL 2026) · **Lab:** PEARL Lab, TU Darmstadt — IRM2 / iRobMan Praktikum · **Timeline:** Apr 24 – Sep 30, 2026 · **Doc as of:** 2026-07-23

This is a single top-to-bottom reference: what the project is, what upstream provides, what has been built on top of it, the full folder structure, and current status. It is a synthesis of `README.md`, `CHANGELOG.md`, `notes.md`, `PragmaBot_Panda_Handoff.md`, `PragmaBot_Panda_Status.md`, `panda_notes/*.md`, and git history — read this one file for the complete picture.

---

## 1. What PragmaBot Is (Upstream)

PragmaBot is a **VLM-driven robotic task-learning system**: instead of hard-coded task logic, a large vision-language model observes a scene, plans a manipulation skill, executes it, checks whether it worked, and remembers the experience for next time. No fine-tuning, no per-task code — the VLM generalizes via prompting + retrieval-augmented memory.

Upstream results (on the original legged-manipulator platform, ETH Zürich): short-term-memory self-reflection raises task success **35% → 84%**; long-term-memory RAG raises single-trial success on unseen scenarios **22% → 80%**.

### The 7-step pipeline (runs once per planning step, loops until task complete)

```
1. VLMSceneDescriber       → natural-language description of the camera observation
2. MemoryManager           → retrieves top-k relevant past experiences (cosine similarity, LTM)
3. VLMTaskPlanner          → picks next skill: PICK / PLACE / PUSH / DONE + target object/location
4. Action Execution        → *** upstream: NotImplementedError — this is the integration point ***
5. VLMSuccessDetector      → compares before/after images → success/failure signal
6. STM update              → appends (action, evaluation); loops to step 3 on failure
7. VLMExperienceSummarizer → on task completion, distills the STM episode into an LTM entry
```

Upstream was built for an **ANYmal quadruped + 6-DoF arm**, Robotiq 2F-140 gripper, arm-mounted ZED X Mini, Pinocchio-based IK. **Action execution (step 4) is left as `NotImplementedError`.**

### Hard rule followed throughout this fork

The upstream VLM/memory files are **read-only** and never modified except for a small, fixed set of surgical edits in `pragmabot_node.py` (importing/instantiating the executor and replacing the two `NotImplementedError` sites). Untouched upstream files: `vlm_client.py`, `vlm_task_planner.py`, `vlm_success_detector.py`, `vlm_scene_describer.py`, `vlm_exp_summarizer.py`, `memory_manager.py`, `scene_observer.py`, `conversation_builder.py`.

### VLM prompt structure (every component)

1. **System prompt** — role, capabilities, output schema
2. **Task prompt** — user instruction + current observation image
3. **Instruction prompt** — reasoning rules, hard constraints; STM/LTM injected here as template variables

### Action skills & constraints (enforced via VLM prompts, not code)

- **PICK** — target must be graspable (not flat/tiny), gripper must be empty
- **PLACE** — must already be holding an object
- **PUSH** — table-top objects only; used when an object is too flat/small to pick

### Memory system

- **STM** — per-episode list of `{time_step, action}` / `{time_step, evaluation}`, JSON, injected into the planner prompt each loop
- **LTM** — two CSVs merged on a `scenario` key: `ltm.csv` (human-readable) + `ltm_<model>.csv` (base64 embeddings); scenario key = `"Instruction: {task}\nScene: {initial_scene_description}"`; retrieval = cosine similarity, top-k

---

## 2. What This Fork Set Out to Do

**Task:** implement the action-execution layer (step 4) for a **Franka Emika Panda** with a **static ZED2** camera, while leaving the planner/STM/LTM/RAG logic untouched. Two side investigations also in scope per the course project: ontology-based experience representations, and local VLM acceleration to cut API latency.

**Fully reusable, no changes needed:** VLM planner prompt structure/concept, success-detector concept, STM loop, LTM storage logic, RAG retrieval, overall algorithmic flow.

**Had to be rebuilt:** everything downstream of the planner's `NextBestAction` output — object grounding, grasp generation, motion execution, gripper control.

---

## 3. The Critical Fact: Two Contradictory Tracks Exist in This Repo

**Track A (ROS1 + MoveIt)** is fully implemented, committed, and building — but was built assuming an architecture that a later investigation found **cannot drive the real robot**.

**Track B (ROS2 + Polymetis)** is the decided replacement for the physical robot, but as of this writing **zero code has been written for it**.

| | Track A — ROS1/MoveIt | Track B — ROS2/Polymetis |
|---|---|---|
| Status | Code complete, builds, tested in rosbag replay only | Decision made, no implementation yet |
| Commits | `a7b710d`, `e8cfca5`, `4227629` | None |
| Can run on real robot? | **No** — firmware/libfranka incompatibility | Yes (once implemented) |
| What survives regardless | Perception (GroundedSAM), grasp choice (GraspGen), VLM/memory stack, skill semantics | — |

### Why Track A can't reach the real robot

The lab's physical Panda runs firmware **v5.9.0**, requiring **libfranka ≥ 0.15**. `frankarobotics/franka_ros2` has dropped Panda/FER support entirely (ships only FR3 configs now); every community Panda fork targets libfranka 0.8–0.10. There is no viable ROS1 + MoveIt + `franka_gripper`/actionlib path to this specific robot's firmware.

### The agreed replacement ("Path A" pivot)

Fork two of a labmate's ("rickmer") already-working Docker containers rather than build a new stack from scratch:

| Container | Fork of | Base | Stack | Role |
|---|---|---|---|---|
| Control | `rickmer-ros2_polymetis-1` | ROS2 Foxy | libfranka 0.18.2, Polymetis 0.2, `role_ros2` | robot motion |
| Perception | `rickmer-ros2_cu118-1` | ROS2 Humble | ZED SDK 4.1, zed-ros2-wrapper, CUDA 11.8 | camera/vision |

Both share `ROS_DOMAIN_ID=7` for cross-container topic visibility. MoveIt's `plan_and_execute` + `franka_gripper` actionlib is replaced by `role_ros2` ROS2 services: `/panda_arm/move_to_joint_positions`, `/panda_arm/move_to_ee_pose` (IK handled **internally** by `role_ros2`'s MuJoCo/dm_control `RobotIKSolver` — do not add a separate IK layer), `/panda_gripper/gripper/grasp`, `/panda_gripper/gripper/open`. Known required fix: `gripper_type` defaults to `robotis` in `role_ros2`, must be overridden to `franka_hand`.

**Open question, not yet answered by supervisor:** should the PragmaBot planner/Gradio node itself move to ROS2, or stay on ROS1 and bridge to the ROS2 containers (via `ros1_bridge`, or a ZMQ-style bridge like the one already used for GraspGen)?

---

## 4. Repository Folder Structure

```
pragmabot/                                  # repo root (catkin package parent)
├── CLAUDE.md                               # instructions for Claude Code on this repo
├── CLAUDE_CODE_PLAN.md                     # original ROS1 phase-by-phase implementation plan
├── README.md                               # public-facing project description, upstream citation
├── CHANGELOG.md                            # upstream v1.0.0 release notes (Keep a Changelog format)
├── CONTRIBUTING.md                         # upstream contribution guide
├── LICENSE                                 # BSD-3-Clause
├── SETUP.md                                # install steps (ROS1-track only)
├── notes.md                                # running technical log — architecture understanding, VLM backend details, gotchas
├── mynotes.md                              # personal scratch notes
├── requirements.txt                        # Python deps (core pipeline + ROS build tools + scripts)
│
├── PragmaBot_Panda_Handoff.md              # ★ single most complete "read this first" status doc — consolidates all planning docs
├── PragmaBot_Panda_Status.md               # pivot-session doc: ROS1→ROS2 architecture decision, gap analysis
├── panda_notes/
│   ├── panda_design_spec.md                # early perception/grasp/action design rationale
│   ├── panda_gap_analysis.md               # original reuse-vs-adapt component table
│   └── pragmabot_panda_project_setup.md    # earliest planning doc, largely superseded
│
├── docs/                                   # upstream README assets (screenshots, teaser gif)
│   ├── memory_gui.jpg
│   ├── plan_gui.jpg
│   └── teaser.gif
│
├── scripts/                                # standalone analysis/figure-generation scripts (not part of the ROS package)
│   ├── README.md
│   ├── convert_log_to_html.py              # conversation log → HTML viewer
│   ├── evaluate_first_action.py            # metrics on first-action accuracy
│   ├── extract_numbers_from_markdown.py
│   ├── plot_annotion_ablation.py
│   ├── plot_failure_flow.py
│   ├── plot_rag_ablation_spider_split.py
│   ├── test_grounded_sam.py                # offline GroundedSAM validation (passing)
│   ├── gsam_result.png
│   └── output/.gitkeep
│
├── weights/                                # model weights (persisted, volume-mounted; ~3.2GB)
│   ├── groundingdino_swint_ogc.pth
│   └── sam_vit_h_4b8939.pth
│
└── pragmabot/                              # the actual catkin ROS package
    ├── package.xml
    ├── CMakeLists.txt                      # declares action-message generation (add_action_files)
    ├── setup.py
    │
    ├── config/
    │   └── config.yaml                     # ALL runtime config: rosbag_replay, STM/LTM toggles, vlm_model, ROS topic names
    │
    ├── action/                             # ROS1 actionlib message definitions (Track A only)
    │   ├── PandaPick.action
    │   ├── PandaPlace.action
    │   └── PandaPush.action
    │
    ├── launch/
    │   ├── launch_pragmabot.launch         # main UI + pipeline
    │   ├── launch_panda_servers.launch     # launches the 3 Panda action servers
    │   ├── manage_memory.launch            # LTM inspection UI
    │   ├── record_rosbag.launch
    │   ├── replay_rosbag.launch            # replay without a physical robot
    │   └── visualize_rosbag.launch
    │
    ├── nodes/                              # executable ROS nodes
    │   ├── pragmabot_node.py               # ★ main orchestrator — Gradio UI + 7-step loop (423 lines)
    │   ├── memory_manager_node.py          # separate Gradio UI for LTM inspection/heatmaps (296 lines)
    │   ├── panda_pick_server.py            # actionlib server: PICK (188 lines)
    │   ├── panda_place_server.py           # actionlib server: PLACE (171 lines)
    │   ├── panda_push_server.py            # actionlib server: PUSH (194 lines)
    │   ├── image_republisher_node.py       # rosbag replay frame control (121 lines)
    │   └── image_decompressor_node.py      # (57 lines)
    │
    ├── src/pragmabot/                      # importable Python package (library code)
    │   ├── __init__.py
    │   ├── vlm_client.py                   # ⛔ upstream, untouched — OpenAI client (93 lines)
    │   ├── vlm_task_planner.py             # ⛔ upstream, untouched — NextBestAction schema + planning (206 lines)
    │   ├── vlm_scene_describer.py          # ⛔ upstream, untouched (95 lines)
    │   ├── vlm_success_detector.py         # ⛔ upstream, untouched (130 lines)
    │   ├── vlm_exp_summarizer.py           # ⛔ upstream, untouched (100 lines)
    │   ├── memory_manager.py               # ⛔ upstream, untouched — STM/LTM logic (308 lines)
    │   ├── scene_observer.py               # ⛔ upstream, untouched — camera sync (92 lines)
    │   ├── conversation_builder.py         # ⛔ upstream, untouched — dual OpenAI/Gradio log builder (129 lines)
    │   ├── utils.py                        # ⛔ upstream, untouched (67 lines)
    │   ├── simple_config.py                # ⛔ upstream, untouched (29 lines)
    │   │
    │   ├── claude_vlm_client.py            # ✅ new — Anthropic Claude backend (120 lines)
    │   ├── gemini_vlm_client.py            # ✅ new — Google Gemini backend (125 lines)
    │   ├── panda_skill_executor.py         # ✅ new — actionlib client bridge, planner → action servers (93 lines)
    │   ├── grounded_sam.py                 # ✅ new — GroundingDINO + SAM wrapper (76 lines)
    │   ├── graspgen_client.py              # ✅ new — ZMQ client to host-side GraspGen server (95 lines)
    │   ├── geometry.py                     # ✅ new — pixel/depth → 3D point-cloud math (85 lines)
    │   │
    │   └── panda_adapter/                  # ⚠️ stale __pycache__ only — abandoned earlier module layout, no source files, safe to delete
    │
    └── data/
        ├── images/.gitkeep                 # captured observation frames (gitignored)
        ├── logs/.gitkeep                   # conversation logs (gitignored)
        └── ltm/ltm.csv                     # long-term memory store
```

**Legend:** ⛔ = upstream file, never modified · ✅ = new file added for the Panda fork · ★ = primary entry point / most important doc

**Note:** `IRM1/` at the repo root (visible in some checkouts) is empty — a placeholder for a different, unrelated project workspace, never populated here.

---

## 5. Changes Made — Commit by Commit

| Commit | Summary |
|---|---|
| `ee68710` | Initial commit — upstream PragmaBot import |
| `1b56929` | docs: Franka adaptation README, planning phase |
| `a7b710d` | Phase 2 scaffold: action msgs (`.action` files), stub action servers, `panda_skill_executor.py` stub, GroundedSAM/GraspGen stubs |
| `e8cfca5` | **Phases 3–6**: full action servers (pick/place/push), complete skill executor, GraspGen ZMQ client, GroundedSAM, `pragmabot_node.py` wiring — this is where Track A became functionally complete |
| `4227629` | Added Claude and Gemini VLM client forks with config-based backend switching |
| *(working tree, uncommitted)* | `CLAUDE.md`, updated `CLAUDE_CODE_PLAN.md`, `PragmaBot_Panda_Handoff.md`, `PragmaBot_Panda_Status.md`, `panda_notes/*.md` — the ROS2-pivot planning/decision documents |

### 5.1 Phase 2 → Phase 6 detail (Track A, ROS1/MoveIt — fully built)

| File | Role | Status |
|---|---|---|
| `pragmabot/action/PandaPick.action`, `PandaPlace.action`, `PandaPush.action` | ROS1 action message definitions | Done, builds |
| `pragmabot/nodes/panda_pick_server.py` | actionlib server: observation pose → RGBD capture → GroundedSAM → point cloud → GraspGen → MoveIt execute → gripper close → lift | Done (placeholder constants) |
| `pragmabot/nodes/panda_place_server.py` | Observation pose → capture → GroundedSAM on target → centroid → 3D point → MoveIt → descend → open gripper → retreat | Done (placeholders) |
| `pragmabot/nodes/panda_push_server.py` | GroundedSAM on object + goal region → push vector → MoveIt Cartesian path | Done (placeholders) |
| `pragmabot/src/pragmabot/panda_skill_executor.py` | `actionlib.SimpleActionClient` bridge — dispatches planner's `NextBestAction` → `/panda/pick`, `/panda/place`, `/panda/push` | Done |
| `pragmabot/src/pragmabot/grounded_sam.py` | GroundingDINO (open-vocab text→box) + SAM ViT-H (box→mask) wrapper | Done, tested (0.592 confidence on demo image) |
| `pragmabot/src/pragmabot/graspgen_client.py` | ZMQ client (MsgPack) to a GraspGen server on the **host machine**, port 5556 | Done, never tested against a live server |
| `pragmabot/launch/launch_panda_servers.launch` | Launches all three action servers | Done |
| `scripts/test_grounded_sam.py` | Offline GroundedSAM validation | Done, passing |
| `pragmabot_node.py` edits | Import + instantiate `PandaSkillExecutor`; replace both `NotImplementedError` sites (action execution, replanning after evaluation) | Done — verified minimal diff |

### 5.2 VLM backend abstraction (commit `4227629`)

Originally OpenAI-only. Now three interchangeable backends selected by substring match on `config.yaml`'s `vlm.vlm_model`:

| Backend | File | Structured output | Embeddings |
|---|---|---|---|
| OpenAI (original, untouched) | `vlm_client.py` | `chat.completions.parse()` | `text-embedding-3-large`, 3072D |
| Claude (new) | `claude_vlm_client.py` | `messages.parse()` with adaptive thinking | No native API → local `sentence-transformers` (`all-MiniLM-L6-v2`), 384D |
| Gemini (new) | `gemini_vlm_client.py` | `GenerativeModel` + JSON schema response | `text-embedding-004`, 768D (native) |

**Load-bearing gotcha:** the three backends produce different embedding dimensions, so LTM CSVs are not portable across backends — switching requires either a fresh LTM or separate CSV sets per backend. Currently configured default: `claude-opus-4-8` (`pragmabot/config/config.yaml:16`).

Key implementation differences handled per-backend:
- Claude's system prompt is a top-level API parameter, not a `messages` entry; `messages.parse()` requires `max_tokens` explicitly (no default)
- Gemini's assistant role is `"model"`, not `"assistant"`
- Image payload format differs per API: OpenAI `image_url` (base64 data-URI) → Claude `{"type":"image","source":{"type":"base64",...}}` → Gemini `inline_data` (raw base64, no prefix)

### 5.3 Explicit placeholders (not bugs — flagged, waiting on lab access)

- `OBSERVATION_JOINT_CONFIG = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]` in all three servers — needs real robot measurement
- Camera intrinsics `fx=fy=700` — needs real ZED2 calibration
- No `static_zed2_tf.launch` yet — extrinsic calibration (`zed2_left_camera_frame` → `panda_link0`) not done
- GraspGen server has never been started/connected live — client exists, round-trip untested

---

## 6. Perception & Grasp Design (survives the pivot unchanged)

```
User instruction → VLMSceneDescriber → concrete scene description
                                          ↓
                                   VLMTaskPlanner → target_object = "red cup"
                                                          ↓
                                               GroundedSAM("red cup", rgb) → mask
                                                          ↓
                                               mask + depth → point cloud → GraspGen → SE(3) grasp pose
```

- **Camera:** ZED2, static mount facing the table (not arm-mounted, unlike upstream's ANYmal setup) — one-time extrinsic calibration only.
- **Segmentation:** GroundedSAM (GroundingDINO + SAM), not SAM2 alone — SAM2 has no open-vocabulary text→mask capability; GroundedSAM takes `target_object` (a resolved noun from the planner) + RGB → binary mask. It never sees the raw user instruction — spatial language ("the object on the left") is resolved upstream by `VLMSceneDescriber`.
- **Grasp generation:** GraspGen (NVlabs), chosen over AnyGrasp — AnyGrasp's binary license is machine-locked and breaks in Docker. GraspGen ships a pretrained Franka Panda model, open weights, client-server mode. Runs on the **host machine** (needs Python 3.10 + CUDA, incompatible with the container's Python 3.8); container talks to it over ZMQ — a deliberate split-process design, not a workaround.

### VLM planner output schema (unchanged, still active)

```python
class NextBestAction(BaseModel):
    reasoning: str          # chain-of-thought — must come first; forces reflection before commitment
    skill: str              # "pick" | "place" | "push" | "done"
    target_object: str
    target_location: str | None = None
    use_annotation: bool = False
```

---

## 7. Dependencies

### 7.1 Declared in `requirements.txt` (core pipeline + build tools)

`numpy`, `opencv-python`, `matplotlib`, `Pillow`, `openai`, `pandas`, `gradio`, `omegaconf`, `pydantic`, `PyYAML`, plus `catkin_pkg`/`rospkg`/`rosdep` for ROS builds and `seaborn`/`plotly`/`webcolors` for the analysis scripts.

### 7.2 Imported by new code but **missing from `requirements.txt`** (known gap)

- `pyzmq`, `msgpack`, `msgpack-numpy` — `graspgen_client.py`
- `sentence-transformers` — Claude backend's embedding fallback
- `anthropic` — `claude_vlm_client.py`
- `google-generativeai` — `gemini_vlm_client.py`
- `groundingdino` (editable install from `/tmp/GroundingDINO`, not pip-published) and `segment-anything` — `grounded_sam.py`

### 7.3 System/container-level (Track A container: `pragmabot_panda` / `3liyounes/pearl_robots:franka`)

ROS Noetic, Ubuntu 20.04, Python 3.8. Present: `panda_moveit_config`, `franka_gripper`, MoveIt (KDL IK), `actionlib`. Installed for this project: GroundingDINO (`/tmp/GroundingDINO` — **not persisted, reinstall every container restart**; weights persist under `weights/`, ~3.2GB), `segment-anything`. GraspGen deliberately **not** installed in-container — runs on host, CPU-only inference for GroundedSAM currently (CUDA extension fails to compile against Python 3.8 + old CUDA).

### 7.4 External API keys

`OPENAI_API_KEY` (paid credits, not ChatGPT Plus), `ANTHROPIC_API_KEY` (current default backend), `GOOGLE_API_KEY` (Gemini path).

### 7.5 Track B (ROS2 pivot) dependencies — not yet started

Two containers to fork (`rickmer-ros2_polymetis-1`, `rickmer-ros2_cu118-1`), `role_ros2` package, physical robot at `10.10.10.10` (firmware v5.9.0), compute on the Alonnisos machine (RTX 4080, driver 535.183.01).

### 7.6 Disqualified dependencies (don't re-investigate)

`franka_ros2` official Panda/FER support (dropped upstream); community libfranka forks targeting 0.8–0.10 (incompatible with firmware ≥0.15 requirement); AnyGrasp (machine-locked license, breaks in Docker); SAM2 alone (no open-vocabulary text→mask).

---

## 8. Configuration Reference (`pragmabot/config/config.yaml`)

```yaml
rosbag_replay: true        # skips real action execution, calls success detector directly — primary dev/test mode
gradio_share: false

activate_stm: true
activate_ltm: true
save_to_ltm: true
use_random_retrieval: false
retrieval_top_k: 5         # -1 = everything, unsorted

vlm:
  vlm_model: claude-opus-4-8     # gpt-4o-2024-08-06 | claude-opus-4-8 | gemini-2.5-pro
  text_embedding_model: text-embedding-3-large   # only consumed by the OpenAI path

topics:
  color_image: /zedxm/zed_node/rgb/image_rect_color/compressed
  depth_image: /zedxm/zed_node/depth/depth_registered
  camera_info: /zedxm/zed_node/rgb/camera_info
```

---

## 9. Known Gotchas (don't re-discover these)

- `catkin build` fails on a build-space conflict in this container — use `catkin_make --pkg pragmabot` instead
- GroundingDINO lives in `/tmp` — not persisted, must `pip install -e .` again after every container restart; weights under `weights/` do persist
- `sentence-transformers` needs a separate pip install for the Claude backend's embedding fallback
- LTM CSVs are embedding-model-specific — never mix across VLM backends (3072D OpenAI vs 384D Claude-fallback vs 768D Gemini)
- Gemini's assistant role is `"model"`, not `"assistant"`
- Claude's system prompt is a top-level API parameter, not a message; `client.messages.parse()` requires `max_tokens` explicitly
- Stale `__pycache__` artifacts at `pragmabot/src/pragmabot/panda_adapter/__pycache__/` have no corresponding source — leftovers from an abandoned earlier module layout (a separate `panda_adapter` subpackage, later consolidated directly into `src/pragmabot/`). Harmless, safe to delete.
- `IRM1/` at repo root (when present) is empty — unrelated placeholder, not part of this project

---

## 10. Current Status & Next Steps

### Track A (ROS1, can continue without lab access)

1. Add the missing packages to `requirements.txt` (§7.2)
2. Start a GraspGen server on the host and validate the ZMQ round-trip from `graspgen_client.py` for the first time
3. Clean up stale `panda_adapter/__pycache__`

### Track B (ROS2 pivot — the actual path to the physical robot)

| Phase | Description | Status |
|---|---|---|
| 0 | Architecture selection (ROS1/MoveIt → ROS2/Polymetis) | Done |
| 1 | `docker inspect` both rickmer containers → confirm network mode / `ROS_DOMAIN_ID` cross-visibility | **Next up** |
| 2 | Read `role_ros2/robot_ik/robot_ik_solver.py` → verify MuJoCo model is Panda-correct | Not started |
| 3 | Fork both containers, adapt `PandaSkillExecutor` to `role_ros2` service interface | Not started |
| 4 | Perception container integration — ZED2 static TF, observation pose | Not started |
| 5 | Port pick/place/push dispatch to `role_ros2` services | Not started |
| 6 | GroundedSAM/GraspGen install in new containers | Not started |
| 7 | Wire executor into `pragmabot_node.py` (redo for ROS2 client) | Not started |
| 8 | STM/LTM/RAG end-to-end validation on real robot | Not started |

Paused deliberately after Phase 0 — pending study of the pivot before Phase 1 begins.

### Lab-access-only items (either track)

1. Measure real observation joint config (jog arm out of camera FOV, read `/joint_states`)
2. ZED2 → `panda_link0` static TF calibration
3. Real camera intrinsics (replace `fx=fy=700` placeholder)

### Open question for supervisor

Should the PragmaBot planner/Gradio node move to ROS2 too, or stay on ROS1 and bridge to the ROS2 control/perception containers? This decision gates whether the three-action-server split (pick/place/push as separate processes) is still worth keeping, since `role_ros2`'s service model may already provide the process isolation that split was designed for.

---

## 11. Source Documents This File Synthesizes

`README.md`, `CHANGELOG.md`, `notes.md`, `PragmaBot_Panda_Handoff.md` (most detailed prior status doc), `PragmaBot_Panda_Status.md` (pivot decision), `CLAUDE_CODE_PLAN.md`, `panda_notes/panda_design_spec.md`, `panda_notes/panda_gap_analysis.md`, `panda_notes/pragmabot_panda_project_setup.md`, `pragmabot/config/config.yaml`, `requirements.txt`, and git history on `feature/panda-adaptation` (commits `ee68710` → `4227629`).
