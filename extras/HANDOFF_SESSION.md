# HANDOFF — read this first in a fresh chat

**Written:** 2026-08-22 · **Last re-verified:** 2026-08-22 (same day, no new
commits or lab data since) · **Report due:** 2026-09-14 (~23 days)
**Purpose:** resume without re-reading the paper, re-tracing code, or re-deriving
decisions already made. Everything here was verified against files on disk or
computed from real data — nothing is assumed.

---

## 0. What this project is, in five lines

- Reproducing **PragmaBot** (IEEE RAL 2026, ETH Zürich, arXiv 2507.16713) on a
  **Franka FR3** at TU Darmstadt.
- The paper: a VLM plans one skill → visually verifies success by comparing
  before/after images → self-reflects on failure into short-term memory (STM) →
  summarises success into long-term memory (LTM) → retrieves past experience by
  RAG. **No fine-tuning.**
- **The released code leaves action execution as `NotImplementedError`.**
  Building that layer is the project.
- Course: iRobMan Praktikum **Project 3, "Memory representations for Robotic Task
  Planning"**. Supervisor **Vignesh Prasad**.
- Grading **30% supervisor / 40% report / 30% presentation**. Presentation Sept 21/22.

---

## 1. Hard rules — do not violate

1. **Never modify these 11 UPSTREAM modules** in `pragmabot/src/pragmabot/`:
   `vlm_client.py`, `vlm_task_planner.py`, `vlm_scene_describer.py`,
   `vlm_success_detector.py`, `vlm_exp_summarizer.py`, `memory_manager.py`,
   `conversation_builder.py`, `geometry.py`, `utils.py`, `simple_config.py`,
   `__init__.py` — plus `scene_observer.py`*, the 12th.

   **Verified 2026-08-24** against a fresh clone of
   `github.com/leggedrobotics/pragmabot` (commit `ee68710`): 11 of the 12 are
   **byte-identical** to upstream. Diff with `--strip-trailing-cr` — the repo has
   CRLF endings, so a plain `diff` reports every line as changed (equal `+N/-N`)
   and looks alarming for no reason.

   *(`scene_observer.py` WAS ported rospy→rclpy — unavoidable, it is the only
   ROS-coupled module. Public surface unchanged; `get_scene_observation()` still
   returns the same 4-tuple. Every hunk is forced by ROS 2: rclpy has no implicit
   global node, callbacks need a spin rather than `time.sleep`, and QoS must be
   explicitly BEST_EFFORT or the subscription silently receives nothing.)*

   **These are OURS, not upstream — the old list wrongly protected them:**
   `claude_vlm_client.py`, `gemini_vlm_client.py`, `panda_skill_executor.py`.
   They may be edited. `grounded_sam.py` was also ours and was **deleted**
   (`a0e3510`): unreachable, ROS 1 catkin paths, and SAM 1 while the working
   path uses SAM 2.

   Upstream also has **no** `calibration/`, `ros2_ws/`, `bags/`, `extracted/`
   or `weights/`. All of that is ours and freely editable.
2. **Flag uncertainty rather than guess.** This project has already had two real
   incidents from assuming: a fabricated Docker image tag, and Panda/FR3 identity
   confusion that triggered an unnecessary architecture pivot.
3. **Teach, don't vibecode.** Harish must defend every line in a graded
   presentation. Lead with the concept and search terms; code after.
4. **Robot is an FR3, never a Panda.** Never reference `role_ros2`, Polymetis, or
   `rickmer-ros2_*` containers.

---

## 2. Git state

```
main       21804dd   (untouched, also on GitHub)
ros2-port  d6c03aa   8 commits  ← 2 NEW, NOT YET PUSHED
tag        pre-phase1-ros2port  → rollback point, pushed
```

New since the last handoff (both local — `git push origin ros2-port` when ready):
- `a0e3510` Delete dead `grounded_sam.py` and its test
- `d6c03aa` Add ZMQ perception server, client and offline guard tests

No lab session has happened yet — `ltm.csv` is still header-only (0 rows).

Review the diff: `https://github.com/HarishDeepak/irm2pragmabot/pull/new/ros2-port`
Roll back: `git checkout main && git branch -D ros2-port`

**GitHub is now the single source of truth.** The old "copy folders to GDrive"
workflow is obsolete and dangerous (silent divergence, no undo). Lab machine and
laptop both `git pull` / `git push`.

---

## 3. What was done this session (all on `ros2-port`)

| Commit | What |
|---|---|
| `4e68c8c` | `scene_observer.py` rospy→rclpy; **camera topics fixed** |
| `f00a268` | Planner⇄robot connection: `ExecuteSkill.action`, executor rewrite, `execute_place()` |
| `65fa145` | Offline schema contract test |
| `68e7175` | catkin → ament_python; deleted 4 dead ROS 1 files (648 lines) |
| `3ba0df5` | Recursion cap, fabricated LTM removed, embedding config fixed |
| `33166c2` | Executor gets its own rclpy node |

**Three tests pass, all robot-free**, and each verified to fail when the bug it
guards is reintroduced (each was deliberately re-broken to check):
```bash
python scripts/test_executor_schema.py    # planner↔executor field contract
python scripts/test_planning_loop.py      # loop is bounded
python scripts/test_perception_guards.py  # 9 detection guards (no torch/GPU)
```

### The camera topic fix was a hard blocker
`config.yaml` pointed at `/zedxm/zed_node/...` — the **upstream ANYmal robot's**
camera. Verified against `bags/red_cup/metadata.yaml`, yours is:
- `/zed/zed_node/rgb/color/rect/image` — **raw `Image`, NOT `CompressedImage`**
  (no `/compressed` topic exists in the bag at all)
- `/zed/zed_node/depth/depth_registered`
- `/zed/zed_node/rgb/color/rect/image/camera_info` — **nested under the image
  topic**, not a sibling

The planner literally could not see anything before this.

### Three bugs fixed that had NEVER run
`config.yaml` sets `rosbag_replay: true`, and `pragmabot_node.py` skips the
executor entirely in that mode — so this code path had never executed:
- `action.skill` → `AttributeError`; the field is `chosen_skill`
- `target_location` → silently `""`; the field is `placement_object`
- dead `elif skill == "done"` branch; `RobotSkill` only has push/pick/place

`PROJECT_OVERVIEW.md` §6 documents the **wrong** schema. Code against
`vlm_task_planner.py`, never that doc.

### Integrity fixes
- **`data/ltm/ltm.csv` held 4 fabricated entries** describing the *paper's*
  scenarios (egg/banana, plate/apple, candy/sponge), timestamped `2026-02-07` at
  round hours. No such robot run happened. Archived to
  `data/ltm/archive/` with a README; `ltm.csv` now has a **`provenance`** column
  (`real_robot` | `replay` | `authored`).
  They also **crashed retrieval**: no embeddings file → every row gets
  `embedding=None` → `np.linalg.norm(None)` raises `TypeError`. Reproduced
  directly. Empty LTM is safe (`.apply()` over 0 rows never calls it) but the bug
  is **dormant, not fixed** — the guard belongs in `memory_manager.cosine_similarity`,
  which is on the never-modify list.
- **Unbounded recursion**: `handle_planning_request` → `handle_evaluation_request`
  → `handle_planning_request`, uncapped. Now a bounded loop with `max_steps: 10`.
  The cap is also the paper's headline metric — an uncapped run has no denominator.
- **`text_embedding_model`** said `text-embedding-3-large` while the Claude path
  hardcodes `all-MiniLM-L6-v2` (384-d). `MemoryManager` builds the embeddings
  *filename* from this value, so the file was named after a model that never
  computed its contents.

---

## 4. Verified facts computed from real data — do not re-derive

### Calibration (`D:/Downloads/zed_franka_base_calibration.md`)
High quality: 30/30 frames, translation std `[0.195, 0.501, 0.218] mm`, rotation
RMS `0.0501°`, 4 reference tags visible in every frame, measured 2026-08-19.

- **Direction is `T_base←cam`** — proven: `T @ [0,0,0,1]` returns
  `[0.912, -0.064, 0.514]`, exactly the stated camera origin in base frame.
  *(Getting this backwards is the classic bug in this work. It is settled.)*
- Valid rigid transform: `det(R) = 1.0`, orthogonality error `1.1e-10`.
- Camera **0.91 m in front** of the base, **0.51 m up**, pitched **62.2° down**,
  looking back toward the robot. On a **tripod**, not robot-mounted.
- Intrinsics (real): `fx = fy = 528.604`, `cx = 635.405`, `cy = 363.729`, 1280×720.

### The camera sees MORE than the arm can reach
| x from base | camera sees (y) | arm reaches |
|---|---|---|
| 0.30 m | −0.89 … +0.90 | ~±0.5 m |
| 0.50 m | −0.79 … +0.77 | ~±0.5 m |
| 0.70 m | −0.69 … +0.64 | ~±0.4 m |
| 0.90 m | −0.59 … +0.51 | ✗ beyond reach |

**Reach is the binding constraint, not visibility.** Size the workspace box by the
arm. Past x ≈ 0.95 m the table leaves the image, but that is already beyond the
FR3's ~0.855 m max reach.

### ⚠ UNVERIFIED — an 11 cm discrepancy (measure in lab)
Transforming the real cup cloud (`extracted/red_cup/detections/object_pcd.npy`)
into `fr3_link0`:
- cup spans **z = [−0.109, −0.041] m** → base at **−0.109 m**
- but the calibration file's `table_z` implies ≈ **+0.005 m**

The cup itself is internally consistent (**6.8 cm tall, 9.8 × 9.3 cm footprint,
0.67 m from base** — a real cup, in reach), so the *shape* is right; only the
height reference is in question. Most likely `link0`'s origin sits ~11 cm above
the tabletop on its mounting plate. **Every crop number depends on this.**

---

## 5. Two traps that look like the answer and are not

- **`WorkspaceParameters` is a silent no-op** on a fixed-base arm. Verified from
  source: `ModelBasedStateSpace::setPlanningVolume()`
  (`model_based_state_space.cpp:245-267`) branches only on `PLANAR`/`FLOATING`
  joints — mobile bases. A bolted FR3 is all `REVOLUTE`: it loops 7 joints,
  matches nothing, returns. **No error.**
- **FR3 Desk "SLP-C" safety zones** would kill the ROS 2 stack. The manual:
  *"When SLP-C is activated, the robot cannot be controlled by FCI!"*

The real answer for both is boring: **collision objects in the planning scene**,
added via `/apply_planning_scene` (the **service**, not the topic — a scene
published before `move_group` subscribes is silently lost).

Also: **`moveit_py` does not exist for Humble** (`ros-humble-moveit-py`: 0 hits on
packages.ros.org). Use pure rclpy + `moveit_msgs`, ~60 lines, no build step.

---

## 6. Environment facts

- Robot: **FR3 "Athna"**, arm ID `10070378`, firmware 5.9.0, IP `10.10.10.10`,
  Franka Hand. **Bolted to the table it manipulates on.**
- Lab machine **Alonnisos**: RTX 4080 (sm_89), shared, `/` has hit 100% full.
- **`export ROS_DOMAIN_ID=7` in every shell** or ZED topics are invisible.
- **Activate FCI in Desk** before MoveIt, or `libfranka: Connection to FCI refused`.
- Franka Hand needs **`Homing`** after connect or any fault, else unresponsive.
- Venvs cannot share: GraspGen `torch==2.1.0`/py3.10, Grounded-SAM-2 `torch>=2.3.1`.
  `mask_to_pointcloud.py` is **pure numpy** for this reason (no cv2/scipy/Open3D).
- GraspGen needs **object-scale clouds (~2000 pts)**; scene-scale → CUDA OOM.
- Live interfaces: `/move_action`, `/execute_trajectory`, `/compute_cartesian_path`,
  `/compute_ik`, `/franka_gripper/{grasp,homing,move}`.
  **Never** `/fr3_gripper/gripper_action` (dead stub, silently hangs).

---

## 7. The strategic picture

**You have zero memory results, and "Memory representations" is your project title.**

The grasping pipeline is visible and satisfying, which is why it pulls attention —
but the paper treats grasping as an off-the-shelf given (AnyGrasp for them,
GraspGen for you). It is what you are graded *least* on.

**Correction to an earlier claim in this project's docs:** an old handoff said
"5 of 6 requirements need no lab access." That is wrong. The success detector
compares **before vs after** images; on a static rosbag nothing moves, so it
always sees an unchanged scene. Only **one** of four experiments truly runs offline:

| Experiment | Robot? | Why |
|---|---|---|
| **Retrieval ablation (Fig. 7)** | ❌ No | First-action accuracy only. No execution. **Genuinely offline.** |
| STM ablation (Table II) | ✅ Yes | Needs real failures to reflect on |
| LTM ablation (Table III) | ✅ Yes | Needs real trial outcomes |
| Pick-and-place demo | ✅ Yes | Obviously |

**So the goal is not "avoid the robot" — it is "make every robot minute count."**
The failure mode to avoid: get access → burn it debugging → collect no data.
**Every robot session must end with logged trials.**

Realistic target: **5 trials × 3 tasks, STM on/off ≈ 30 runs ≈ one afternoon**, *if*
the system already works when you walk in.

---

## 8. Open questions for Vignesh

1. **`fr3_hand` or `fr3_hand_tcp`?** Highest-risk unverified assumption — wrong
   choice puts every grasp ~10 cm off. (`bridge_node.py` assumes `fr3_hand`;
   GraspGen's origin is the gripper **base**, which supports that, and
   `easy_handeye2` was run with `robot_effector_frame:='fr3_hand'`. Not conclusive.)
2. Given uncertain access: **fewer trials (5×3) with a rigorous retrieval
   ablation**, rather than broad coverage — acceptable?
3. Is **rosbag-replay acceptable as primary evidence** for the report?
4. Reproduction depth, or the **ontology extension**?

---

## 8b. Session log — 2026-08-24

Append one block per working session. Newest last.

**Upstream comparison (settled a long-open question).** Cloned
`github.com/leggedrobotics/pragmabot` (`ee68710`) and diffed. Findings:
- **11 of 12 protected modules are byte-identical** to upstream. Only
  `scene_observer.py` differs, and every hunk is forced by the ROS 1→2 port.
  Use `diff --strip-trailing-cr` — the repo has CRLF, so a plain diff reports
  every line changed (equal `+N/-N`) and looks alarming for nothing.
- Upstream has **no** `calibration/`, `ros2_ws/`, `bags/`, `extracted/`,
  `weights/`. All ours.
- `claude_vlm_client.py`, `gemini_vlm_client.py`, `panda_skill_executor.py` are
  **ours**, not upstream — the old "14 modules" rule wrongly protected them.
- **Verdict: not over-engineered.** ~3,500 added lines, ~1,900 of it
  `calibration/` — work upstream never faced (elbow-mounted camera vs our
  tripod). Comment density 5–9%. No speculative abstractions.

**Done this session:**
| Commit | What |
|---|---|
| `a0e3510` | Delete dead `grounded_sam.py` + its test (unreachable, catkin paths, SAM 1 while the working path is SAM 2) |
| `d6c03aa` | ZMQ perception server/client + 9 guard tests (B4 half done) |
| `1e927a6` | Per-trial JSONL logging (A1 done) |

**`perception_server.py` NOW VERIFIED END TO END** (laptop, 2026-08-24, `ea1ad15`):
```
perception_client.py --prompt "red cup." --rgb extracted/red_cup/rgb.png ...
  -> OK: 'red cup' conf=0.937, 2000 points, extent 9.5 x 10.7 x 7.3 cm
```
That **matches `extracted/red_cup/detections/object_pcd.npy` from the earlier
manual lab run exactly** — the wrapper reproduces the known-good result.
Running it found two real bugs, both fixed (intrinsics dict-vs-matrix; the
hardcoded `~/groundedsam` path). Guard tests now 11.

**`bridge_node.py` IS NOW WIRED** (`aa35384`). Each pick resolves
`target_object` live via `live_perception.py`; `use_live_perception:=false`
restores the old `grasp_file` path for replay. 5 test scripts pass (31 checks).

**Still NOT verified:**
1. **`bridge._scene_source` must be injected** — a callable returning
   `(rgb, depth, intrinsics)`. Deliberately left None so the bridge does not
   open a competing subscriber on the planner's BEST_EFFORT camera topics.
   Until it is set, every pick fails with a reason (by design, not a bug).
2. Nothing has run on the **lab GPU**, and no real pick has happened.
3. `place` still uses the fixed `place_offset_xyz` — FPS is implemented in
   `calibration/test_fps.py` but not called from the loop.

### Laptop vs lab — environment split (laptop-only, NOT in the repo)
- Created `D:/irm2pragmabot/groundedsam/.venv` (torch 2.5.1+cu121,
  **transformers==4.44.2** — 5.x removed `BertModel.get_head_mask` and
  GroundingDINO breaks). Plus `hydra-core iopath addict yapf timm supervision
  pycocotools msgpack pyzmq`.
- **Ran with `--device cpu` (~18s/request).** GroundingDINO's `_C` CUDA
  extension is not compiled here: the laptop has the NVIDIA driver but no CUDA
  toolkit (`nvcc` absent). `ms_deform_attn.py:330` takes the CUDA branch when
  tensors are on GPU and raises `NameError: _C`; CPU routes to the pure-PyTorch
  fallback already in that file. **The code default is still `cuda`** — Alonnisos
  has the toolkit, so the lab runs the GPU path unchanged.
- To use the GPU *on the laptop* you would install the CUDA 12.1 toolkit and
  rebuild the extension (`pip install --no-build-isolation -e grounding_dino/`,
  with MSVC 2022 already present). Not needed for the project — the lab GPU is
  the target — so it was left alone.

---

## 9. What to do next

See `TASKS_AND_GAPS.md` in this folder for the full backlog, prompts and effort
estimates. The short version:

**No robot needed (do these first):**
1. Per-trial JSONL logging ← *highest value; you cannot re-run experiments in report week*
2. `max_retries=5` on the Anthropic client
3. Bootstrap LTM via replay runs

**Needs robot:**
4. Measure table plane + settle `eef_link` (10 min, blocks the workspace work)
5. Observation-pose retract before before/after capture ← *protects the graded mechanism*
6. Wire perception via ZMQ ← *biggest correctness hole: "pick the cup" currently
   picks whatever was in a stale npz*
7. Table collision object + Cartesian velocity scaling
