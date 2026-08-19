# HANDOFF — read this first

**For:** any assistant or teammate resuming this project.
**Purpose:** work immediately without re-reading the paper, re-tracing the code, or re-deriving decisions already made.
**Written:** 2026-08-19.

---

## 0. Read in this order — do not skip

| # | File | Why |
|---|---|---|
| 1 | **this file** | current state + rules |
| 2 | `00_MASTER_REFERENCE.md` | paper + system + data + gotchas, 1096 lines. **The lookup doc.** |
| 3 | `04_ROS2_MIGRATION.md` | the active work item |
| 4 | `02_REPRODUCTION_PLAN.md` | ordered steps with done-conditions |
| 5 | `01_PAPER_ANALYSIS.md` / `03_PROJECT_NARRATIVE.md` | scope decisions / career material |

Paper study output: `../papers/pragmabot/` (`README.md`, `summary.md`, `insights.md`, `method.md`, `mental-model.md`, `qa.md`, `code/memory_retrieval_demo.py`).

**If you are about to re-read the paper or re-trace `bridge_node.py`: stop.** It is already in `00_MASTER_REFERENCE.md`.

---

## 1. Hard rules

1. **`D:/pb/home` is READ-ONLY.** The user has explicitly required this twice. Read, grep, run analysis — never write, never edit, never `git commit`. Verified zero files written so far.
2. **Write all output to `C:/Users/haris/claude-papers/`**, not the workspace.
3. **Do not use git history as evidence.** The folders are copies from the lab machine; their VC state is not meaningful. Every claim must come from files on disk.
4. **Never modify the 14 upstream VLM/memory modules** in `pragmabot/src/pragmabot/` (listed in `04_ROS2_MIGRATION.md` §2 KEEP). Project rule inherited from upstream.
5. **Say "I could not verify this"** rather than inventing. The user checks.

---

## 2. What the project is, in 5 lines

- Reproducing **PragmaBot** (IEEE RAL 2026, arXiv 2507.16713, ETH Zürich RSL) on a **Franka FR3** at TU Darmstadt.
- The paper = a VLM plans a skill, visually checks if it worked, self-reflects on failure into short-term memory, distils success into long-term memory, retrieves by RAG. **No fine-tuning.**
- **The released code leaves action execution as `NotImplementedError`.** Building that layer is the project.
- Course: iRobMan Praktikum Project 3, *"Memory representations for Robotic Task Planning"*, supervisor **Vignesh Prasad**.
- **Report due 2026-09-14** · presentation 09-21/22 · grading **30% supervisor / 40% report / 30% presentation**.

---

## 3. Current state

### Works, verified on real data
- Grounded-SAM-2 segmentation → mask
- `mask_to_pointcloud.py` → object cloud (with original depth-noise fix)
- GraspGen → 6-DoF grasps (conf 0.92–0.96 on stored artifacts)
- `grasp_transform.py` math — verified correct against real `grasps.npz`
- Hand-eye calibration performed in lab (eye-on-base, `easy_handeye2`)

### Written but NEVER run on the robot
- `bridge_node.py::execute_pick()` — code complete, zero live executions

### Missing
- **Planner ⇄ executor connection** (the critical gap)
- `execute_place()` / `execute_push()` — `NotImplementedError`
- **All memory experiments** — nothing evaluated yet. This is the paper's actual contribution and the project's title.
- Both course extensions (local VLM, ontology memory)

### Not available from home
- No VPN to the robot (`10.10.10.10` unreachable)
- **No hand-eye `.calib` file in this copy** — verified absent. Cannot transform grasps to robot frame here.

---

## 4. The single most important planning fact

**Most of the paper is reproducible without the robot.**

- `pragmabot_node.py:139-141` — with `rosbag_replay: true`, execution is skipped entirely and the success detector is called directly.
- The retrieval ablation (paper §V-D) executes **nothing at all**.

**5 of 6 reproduction requirements need no lab access.** Do not wait for robot time.

Local data available for this: `pragmabot/bags/red_cup/red_cup_0.db3` (raw bag) + `pragmabot/extracted/{red_cup,cup}/` (rgb, depth, intrinsics, masks, clouds, grasps).

---

## 5. Active work item

**Port to ROS 2.** See `04_ROS2_MIGRATION.md` for the full plan.

- Port: `pragmabot_node.py` (423) + `scene_observer.py` (92) = **515 lines**
- Delete: 3 × `panda_*_server.py` + `graspgen_client.py` = **648 lines**
- Rewrite: `panda_skill_executor.py` (93) as `rclpy` ActionClient
- Untouched: **14 modules, ~1,400 lines** — the whole paper contribution is ROS-free Python
- **≈2 days total**

---

## 6. Findings that cost real effort — do not re-derive

### Verified by re-derivation this session
1. `grasps.npz` ↔ `object_pcd.npy` are a consistent pair; saved `centroid` == cloud mean to 1e-6.
2. All 6 grasp rotations valid (`det=1.000000`, orthogonality err ≤2.1e-7).
3. GraspGen origin = **gripper base, not fingertip** — advancing 0.75×depth along +Z lands fingertips on the object in all 6 cases.
4. `estimate_gripper_width` gives **12.1–33.2 mm across 6 grasps on one object** (~3× spread) — justifies per-grasp estimation.
5. **Flying-pixel fix reproduced:** raw 17.7 cm z-extent → 7.8 cm after 3px erosion → 7.3 cm after AABB. Boundary far-outlier rate **11.9% vs interior 1.5%**.
6. **Full pipeline reproduces the shipped `object_pcd.npy` extent exactly** (9.5 × 10.7 × 7.3 cm).
7. `easy_handeye2` is **unmodified upstream** — zero fr3/franka/zed strings. Claim integration, never authorship.
8. `foundationpose/` is **completely empty** (0 files).
9. `/pragmabot` IS mounted (`docker-compose.yml:21`) but is **outside** the colcon build path (`:17`) — which is why rsync is still needed. Both the notes and the compose file are correct.
10. **Bug:** `panda_skill_executor.py:30` reads `action.skill` → `AttributeError`. Field is `chosen_skill`. `:63`/`:79` read `target_location`, silently `""`; field is `placement_object`. `PROJECT_OVERVIEW.md` §6 documents the WRONG schema.

### Stale docs — do not trust
- **`PROJECT_OVERVIEW.md`** — predates the FR3 + host-only architecture. Its "Track A / Track B" framing and its `NextBestAction` schema are both wrong.
- **`config.yaml` topics** — `/zedxm/...` is the upstream ANYmal camera. Yours is `/zed/zed_node/...`.

### Duplicate-copy hazards
- `pragmabot_bridge` + `pragmabot_interfaces` exist in **2 places** (`pragmabot/ros2_ws/src/` and `ros2_ws/franka_ros2/`). Currently byte-identical. One missed rsync from divergence.
- `zed-ros2-wrapper` exists in **2 places**; only `zed_ros2_ws/src/` has the FR3 config changes.

---

## 7. Environment facts

- Robot: **FR3 "Athna"**, firmware 5.9.0, IP `10.10.10.10`, Franka Hand
- Lab machine: **Alonnisos**, RTX 4080, compute capability **8.9**, shared, disk has hit 100%
- **`export ROS_DOMAIN_ID=7` on every terminal** or ZED topics are invisible
- **Activate FCI in Desk** before MoveIt, or `libfranka: Connection to FCI refused`
- Franka Hand needs **`Homing`** after connect or any fault, else unresponsive
- GraspGen venv `torch==2.1.0` / Grounded-SAM-2 venv `torch>=2.3.1` — **cannot share**
- `--no-build-isolation` mandatory for Grounded-SAM-2 installs; pin `transformers<5`
- GraspGen needs **object-scale clouds (~2000 pts)**; scene-scale → CUDA OOM

---

## 8. Open questions for the supervisor

1. Is `fr3_hand` or `fr3_hand_tcp` the correct `eef_link`? (`bridge_node.py:140-145` — **highest-risk unverified assumption**; wrong = every grasp off by ~10 cm)
2. Given ~26 days: place/push breadth, or a rigorous single-skill evaluation with the memory loop closed?
3. Is a rosbag-replay memory experiment acceptable evidence for the report if robot time is short?

---

## 9. Tooling state

- **Live config dir:** `C:/Users/haris/.claude-account1` (NOT `.claude`)
- **`study` skill:** fixed and working. 4 defects repaired (empty `${CLAUDE_PLUGIN_ROOT}` + duplicated path, missing `package.json` for ESM, `pdf-parse` v2-vs-v1 trap → pinned v1 + deep import `pdf-parse/lib/pdf-parse.js`, `python3`→`python`). Original at `SKILL.md.bak`.
- **`i-have-adhd` skill:** installed at `.claude-account1/skills/i-have-adhd`. User has ADHD — **format output that way**: action first, numbered steps, cap lists at 5, no preamble, no closer, concrete time estimates.
- `ponytail` + `recursive-research` are installed under the **inactive** `.claude` profile — not loadable this session.

---

## 10. Next action

Run this, paste the output:

```bash
grep -n "rospy\." D:/pb/home/pragmabot/pragmabot/nodes/pragmabot_node.py
```

That list is the port checklist for step 4 of `04_ROS2_MIGRATION.md` §6.
