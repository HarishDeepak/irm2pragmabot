# Deliverable 1 — Paper Analysis

**Paper:** *A Pragmatist Robot: Learning to Plan Tasks by Experiencing the Real World* — Qu, Lan, Zurbrügg, Chen, Mower, Bou-Ammar, Hutter. IEEE RAL 2026. arXiv [2507.16713](https://arxiv.org/abs/2507.16713).
**Date:** 2026-08-19.
**Conventions:** every claim about your code cites `path:line`. **[VERIFIED HERE]** = I re-derived it from the raw files myself this session. **[UNVERIFIED]** = I could not confirm it; treat as open.

---

## Part A — Full `study` output

The complete study materials are at `C:/Users/haris/claude-papers/papers/pragmabot/`:

| File | Contents |
|---|---|
| `README.md` | Navigation, difficulty, key takeaways, study plan |
| `summary.md` | Background, problem, contributions, **all results tables**, failure analysis, stated limitations |
| `insights.md` | Core idea, why it works, the retrieval finding, trade-offs, **critical reading**, emergent behaviours, prior-work comparison |
| `method.md` | Architecture diagram, annotated Algorithm 1, prompt templates, **real output schemas**, pseudocode, pitfalls, hyperparameter sensitivity, reproduction risks |
| `mental-model.md` | How to classify the paper, prerequisites, position in the literature, what "reproducing it" means |
| `qa.md` | 15 questions (5 basic / 5 intermediate / 5 advanced), answers collapsed |
| `code/memory_retrieval_demo.py` | Runnable, deterministic, numpy-only demo reproducing the RAG > all > random ablation |
| `paper.txt`, `paper.pdf`, `page_01..08.png`, `images/` (63), `meta.json` | Sources |

Deep technical background — the paper *plus* your system, in one place — is in `00_MASTER_REFERENCE.md`.

### Condensed version of the paper

A frozen VLM plays four roles: describe the scene, plan the next skill, judge visually whether it worked, and summarise a finished episode. Failures produce a natural-language self-critique (a **"linguistic gradient"**) that enters a **short-term memory** and reshapes the next decision within the task. Completed episodes are distilled into a **long-term memory** keyed by `(instruction, initial scene description)` and retrieved by top-$k$ cosine similarity for future tasks. No weights change anywhere.

Results: STM raises success **35% → 84%** (4 tasks, two attempts allowed); LTM+RAG raises **single-trial** success **22% → 80%** (12 scenarios, 8 unseen). Retrieval ablation: **RAG 89% > entire-LTM 74% > random 17%**, with full LTM also costing **7.5×** the prompt tokens.

The three things worth carrying away: learning can live in the context window instead of the weights; selective retrieval is an *accuracy* mechanism, not just a cost saving; and geometry proposes while semantics disposes.

---

## Part B — What has to be reproduced for this to count as "recreated"?

This is the question that determines whether the project succeeds, so it is worth being exact.

### B.1 The decisive structural fact

**The released code deliberately does not include action execution.** The upstream repository leaves step 4 of the pipeline as `NotImplementedError`, and its README directs users to supply their own detection, grasp generation, motion planning and control — recommending Grounded SAM, GraspGen and Pinocchio.

So the paper cleanly splits in two:

| Layer | Provided by the paper | Provided by you |
|---|---|---|
| Scene description, planning, success detection, summarisation, STM, LTM, RAG | ✅ code released | — |
| Segmentation, grasp synthesis, IK/motion planning, gripper control, calibration | ❌ | ✅ everything |

**"Recreating the paper" therefore means recreating the *memory claims* on your own robot with your own skill layer.** It cannot mean matching their numbers.

### B.2 What must NOT be the target

Matching 86% / 100% / 89% is not achievable and not the goal:

- Different embodiment (ANYmal + 6-DoF arm + Robotiq 2F-140, elbow-mounted ZED X Mini) vs yours (fixed-base FR3 + Franka Hand + statically mounted ZED2).
- Different grasp model (AnyGrasp vs GraspGen) and different IK (Pinocchio vs MoveIt 2).
- Different VLM snapshot; `gpt-4o` behaviour drifts.
- **5–10 trials per task.** One trial is worth 10–20 percentage points. Their own numbers carry no confidence intervals.
- Their protocol permits a **human operator to reset the scene** after a destructive failure.

The course brief agrees with this reading: it asks you to *"study and re-implement the method on our robot"* and to explore extensions — not to match a table.

### B.3 The minimum bar — what genuinely counts as "recreated"

Six things. The first four are the reproduction; the last two are the course's extensions.

**R1. The seven-step loop closes on the FR3.**
Instruction → scene description → LTM retrieval → plan → **execute on the real robot** → visual success detection → STM update → (loop) → summarise into LTM. At least one skill (**pick**) must run end to end, autonomously, from a natural-language instruction.

**R2. STM self-reflection demonstrably helps.**
Reproduce the *shape* of Table II on your platform: a no-STM baseline (`activate_stm: false`) versus STM enabled, on a small set of tasks with a designed-in failure mode (e.g. target occluded by a nearby object). The claim to establish is *"reflection converts repeated identical failures into an adapted, successful plan."* Report trial counts honestly.

**R3. LTM + RAG demonstrably helps first-attempt planning.**
Reproduce the shape of Table III: single-trial success with an empty LTM vs a populated one, including at least one *structurally similar but unseen* scenario, to show transfer rather than memorisation.

**R4. Retrieval strategy matters.**
Reproduce the §V-D ablation — RAG top-$k$ vs entire-LTM vs random — measuring **first-action accuracy without execution**. This is the cheapest headline result in the paper to reproduce faithfully, needs **no robot at all**, and is the one whose mechanism generalises beyond robotics.

**R5. Local-VLM extension** (course requirement). Baseline to beat, read off Fig. 7: **~10 s** per planning call for `gpt-4o`, **~7 s** for `gpt-4o-mini`, at ~1,100 prompt tokens under RAG.

**R6. Ontology-based memory extension** (course requirement). An alternative to free-text LTM entries, compared on the same tasks.

### B.4 Explicitly out of scope for "recreated"

- Matching the published percentages (§B.2).
- All three skills. Pick is the paper's own dominant skill and the one your stack already targets; place and push are breadth, not proof.
- The ANYmal platform, mobility, or eye-in-hand camera geometry.
- AnyGrasp specifically — the released README recommends GraspGen, so substituting it follows the authors' guidance rather than deviating from it.
- Reusing their LTM. Lessons encode *their* robot's limits, and embeddings are stored per-model and joined on the scenario key, so they are not portable.

---

## Part C — What already exists in your workspace

Substantial. The body is built; the perception→grasp chain is verified on real data.

### C.1 Perception → grasp → motion, verified

| Capability | Where | State |
|---|---|---|
| Open-vocabulary segmentation | `pragmabot/calibration/detect_object.py` (192 lines) | **Working.** Text-prompted, outputs `mask.npy`, `annotated.jpg`, `detections.json`. Correctly chose a cube (conf 0.57) over a false-positive e-stop button (conf 0.48) |
| Mask + depth → object cloud | `pragmabot/calibration/mask_to_pointcloud.py` (326 lines) | **Working, with an original noise fix.** Three filters: `erode_mask` `:67`, `remove_z_outliers` `:81`, `aabb_crop` `:113` |
| Grasp synthesis | GraspGen ZMQ server, port 5556 | **Working on real ZED data.** 50 grasps, confidence 0.84–0.92, orthonormal rotations |
| Grasp persistence | `GraspGen/client-server/graspgen_client.py:99-107`, `:218-219` | **Your addition.** `--save_grasps` writes `grasps`, `confidences`, `centroid` |
| Grasp math | `pragmabot_bridge/pragmabot_bridge/grasp_transform.py` (160 lines) | **Complete.** Un-centering `:40`, top-down selection `:49-59`, standoff `:62-70`, TF `:73-95`, width estimation `:98-148`, pose conversion `:151-160` |
| Pick execution | `pragmabot_bridge/pragmabot_bridge/bridge_node.py` (511 lines) | **Code complete, never run on the robot.** `execute_pick()` `:111-313` |
| Camera↔robot calibration | `easy_handeye2` (vendored) | **Performed in lab** (eye-on-base, `fr3_link0` ↔ `zed_camera_link`). Result file **not present in this copy** |
| VLM planner / STM / LTM / RAG | `pragmabot/pragmabot/src/pragmabot/` | **Upstream, runnable.** Untouched per the project's own rule |
| Multi-backend VLM | `claude_vlm_client.py`, `gemini_vlm_client.py` | **Your addition.** OpenAI / Claude / Gemini selected by config substring |
| FPS annotation prototype | `pragmabot/calibration/test_fps.py` (103 lines) | **Standalone only.** `farthest_point_sampling` `:24`, numbered overlay `:50` |

### C.2 Real data present locally — this is what makes offline work possible

```
pragmabot/bags/red_cup/red_cup_0.db3        raw ROS 2 bag
pragmabot/extracted/red_cup/                rgb.png, depth.npy, intrinsics.json,
                                            grasps.npz, detections/{mask,object_pcd}.npy
pragmabot/extracted/cup/cup/                same, second scene
ros2_ws/franka_ros2/{grasps.npz,object_pcd.npy}   pick-execution test inputs
```

**[VERIFIED HERE]**

- `red_cup` intrinsics: `fx=fy=528.604, cx=635.405, cy=363.729`, 1280×720, frame `zed_left_camera_frame_optical`; depth 94.0% finite, range 0.416–2.960 m; mask 6,746 px. `cup` scene: `fx=fy=521.536`. **Real per-session calibration — the old `fx=fy=700` placeholder is gone.**
- `grasps.npz` keys are exactly `grasps (6,4,4) float32`, `confidences (6,)`, `centroid (3,)` — matching `grasp_transform.py:38-41`.
- Saved `centroid` equals the point cloud's own mean to ~1e-6 → **the un-centering contract is correct and the file pair is consistent.**
- All six rotations valid: `det = 1.000000`, orthogonality error ≤ 2.1e-7, no NaN/Inf.
- **Gripper-base convention check:** advancing each grasp origin 0.75 × `gripper_depth` along its +Z lands the fingertip band on the object in all six cases (0.058–0.072 m from centroid, inside bbox + 2 cm). Grasp origins lying *outside* the cloud is expected, not a bug.
- `estimate_gripper_width` succeeds for all six (180–375 points in band), giving **12.1 / 18.2 / 18.8 / 21.3 / 33.1 / 33.2 mm** — a ~3× spread on one object, which is the empirical justification for the function existing (`grasp_transform.py:104-130`).
- `ros2_ws/franka_ros2/object_pcd.npy` is **byte-identical** to `extracted/red_cup/detections/object_pcd.npy`; `grasps.npz` differs (confidences 0.925–0.941 vs 0.924–0.956) — a separate GraspGen run on the same cloud. GraspGen is stochastic; expected.
- **Full pipeline reproduces the shipped artifact exactly.** Running the three filters at defaults on raw `depth.npy` + `mask.npy`: unfiltered 6334 pts / 17.7 cm z-extent → erode 3px 5786 / 7.8 cm → z-MAD 5786 / 7.8 cm → AABB 5673 / **9.5 × 10.7 × 7.3 cm**, matching the shipped `object_pcd.npy` extent exactly (point count differs only by the final downsample to 2000). Centroid shift 0.19 cm.
- **Flying-pixel root cause independently reproduced:** boundary pixels 11.9% far-outlier rate vs interior 1.5% (~8×). The cube scene in your notes logged 41% vs 10% — different scene, same mechanism.

### C.3 Engineering knowledge already captured

Diagnosed and recorded, and worth as much as the code:

- **Franka Hand homing requirement** — the driver goes unresponsive after connect or after any Grasp/Move fault ("End Effector: Not connected" in Desk) until `Homing` is called. Handled at `bridge_node.py:173` and `:256-262`, with the docstring `:15-19` explicitly recording that this is *not* a network/Docker fault.
- **Singularity guard** — `revolute_jump_threshold` at `bridge_node.py:404-412`, aborting on `fraction < 1.0` rather than executing a truncated path `:237-247`.
- **Lift along base +Z, not the grasp's own −Z** `:264-268`, to avoid re-colliding with the table.
- **FCI must be activated in Desk** before MoveIt, else `libfranka: Connection to FCI refused`.
- **GraspGen needs object-scale clouds** (~2000 pts); scene-scale causes CUDA OOM (40 GiB attempted) or a discriminator crash.
- **`ROS_DOMAIN_ID=7` on every terminal**, or ZED topics are invisible.
- **Environment isolation is venv-level, not container-level** — GraspGen `torch==2.1.0` vs Grounded-SAM-2 `torch>=2.3.1` cannot share an environment.

---

## Part D — What does not exist yet

Ordered by how much it blocks "recreated."

### D.1 The critical gap: nothing connects the planner to the FR3 executor **[LAB to validate, HOME to build]**

This is a gap of *kind*, not degree. Everything else is incremental.

- The brain is **ROS 1**: `panda_skill_executor.py:1-2` imports `rospy` and `actionlib`, dispatching to `/panda/pick|place|push` `:15-17`.
- The body is **ROS 2**: `bridge_node.py` is `rclpy`, and exposes **no interface at all** — `main()` `:476-507` reads a `grasp_file` parameter, runs **one** `execute_pick()`, then spins. No action server, no service, no topic.
- The package designed to be that contract **generates nothing**: `pragmabot_interfaces/CMakeLists.txt:16-22` has `rosidl_generate_interfaces` commented out, with the open decision recorded in the comment (one collapsed `ExecuteSkill.action` vs three separate actions). There is no `action/` directory.

Without this, R1 is impossible and R2/R3 cannot be run on the robot.

### D.2 A real bug in the existing ROS 1 executor **[HOME]** **[VERIFIED HERE]**

`PandaSkillExecutor` reads fields that do not exist on the planner's actual output schema:

| Line | Code | Actual behaviour |
|---|---|---|
| `panda_skill_executor.py:30` | `skill = action.skill.lower()` | **`AttributeError`** — the field is `chosen_skill` (`vlm_task_planner.py:107`) |
| `:38` | `elif skill == "done"` | dead branch — `RobotSkill` (`vlm_task_planner.py:73-79`) has only push/pick/place |
| `:63`, `:79` | `getattr(action, "target_location", "")` | **silently returns `""`** — the field is `placement_object` (`:116`) |

Never caught because `config.yaml` sets `rosbag_replay: true` and `pragmabot_node.py:139-141` branches past the executor entirely. **The only code path that would expose it has never been run.**

**Consequence for the new bridge:** it must read `chosen_skill`, `target_object`, `placement_object`, `push_direction`, `should_grasp_at_specific_section`, `should_place_at_specific_section`. Note that `PROJECT_OVERVIEW.md` §6 documents a **different, wrong** schema (`skill`, `target_location`, `use_annotation`) — writing the new executor against that doc reproduces exactly this bug.

### D.3 Stale camera topics in config **[HOME]**

`pragmabot/config/config.yaml` points at `/zedxm/zed_node/...` — the upstream ANYmal ZED X Mini. Your camera publishes under `/zed/zed_node/...`. Cheap to fix, easy to forget, and blocks the planner from seeing anything.

### D.4 Live pick has never run end to end **[LAB]**

Per your own session notes: components were smoke-tested for build/import correctness in isolation; **no live execution has happened.** Two specific unverified assumptions inside it:

- **`eef_link = fr3_hand` is not verified.** `bridge_node.py:140-145` states plainly that it must match whatever link GraspGen's gripper-base convention corresponds to in this MoveIt config, and that some setups use `fr3_hand_tcp`. **This is the highest-risk unverified assumption in the execution path** — a wrong tip link offsets every grasp by the hand-to-TCP distance (~10 cm).
- **`revolute_jump_threshold = 0.2`** is "a conservative starting point, not empirically tuned for this robot/workspace" (`:404-412`).

### D.5 Two of three skills unimplemented **[HOME to write, LAB to validate]**

`bridge_node.py:315-316` `execute_place()` and `:318-319` `execute_push()` both `raise NotImplementedError`.

### D.6 The annotation module is not in the pipeline **[HOME]**

`test_fps.py` implements FPS candidate generation with numbered overlays — the mechanism of §IV-F — but is standalone. Missing:
- No wiring to the planner's `should_grasp_at_specific_section` / `should_place_at_specific_section` flags.
- No `s_conf · s_loc` grasp selection (paper Eq. 6). Current selection is `select_topdown_index` (`grasp_transform.py:49-59`) — a *kinematic* heuristic, not a semantic one.
- No IK feasibility pre-filter on the grasp set (the paper uses Pinocchio; MoveIt's `/compute_ik` is available and unused for this).

Worth calibrating against Fig. 8: annotation gives **zero** benefit on box/mug/banana and large gains on skewer (100 vs 20), brush (80 vs 20), drumstick (80 vs 40). If your objects are simple, this module is low priority.

### D.7 No memory experiment has been run at all **[HOME — this is the surprise]**

Nothing in the workspace evaluates STM, LTM, or retrieval. No results, no logs, no LTM beyond upstream's `data/ltm/ltm.csv`. **This is the paper's actual contribution and the course project's actual title, and it is the least-progressed part of the project.**

The important and non-obvious point: **R4 (the retrieval ablation) needs no robot**, and R2/R3 can be run in `rosbag_replay: true` mode, which skips execution entirely (`pragmabot_node.py:139-141`) and calls the success detector directly. **Most of the paper's scientific content is reproducible from this laptop tonight.**

### D.8 Course extensions not started **[HOME]**

- **Local VLM** — baseline to beat is ~10 s (`gpt-4o`) / ~7 s (`mini`) per planning call at ~1,100 prompt tokens.
- **Ontology-based memory** — the cleanest insertion point is `memory_manager.py:119-123`, where retrieved entries are serialised to JSON strings for the planner. A structured representation can be swapped in **at that boundary without modifying the planner**, preserving the "never modify upstream" rule.

### D.9 Not verifiable from home at all **[LAB]**

- No VPN or network path to the robot exists, so `robot_ip:=10.10.10.10` is unreachable.
- **No hand-eye calibration result file exists in this copy** **[VERIFIED HERE]** — a full search found no `.calib`. The TF chain `fr3_link0 → zed_camera_link` cannot be reconstructed here, so no grasp can be transformed into robot frame at home.
- FoundationPose is empty — zero files. Its calibration path was designed but never completed (and `easy_handeye2` already solved the problem).

---

## Part E — Honest summary

**What is genuinely strong.** The perception-to-grasp chain is real, verified against real captures, and in one respect **better than the paper's**: the flying-pixel fix attacks precisely the failure mode that dominates the paper's own failure analysis (8/19 execution failures, of which 3 are "inaccurate depth"; §V-E further attributes annotation failures to "inaccurate 3D point clouds"). The bridge code is careful, honest about its assumptions, and encodes hardware knowledge that took real debugging to obtain.

**What is genuinely missing.** The memory layer — the paper's contribution and the project's title — has not been evaluated at all, and the planner is not connected to the robot.

**The most important realisation from this analysis:** the two are separable. R4 needs no robot; R2 and R3 can run in rosbag-replay mode. **The scientifically central part of the reproduction is not blocked by lab access.** With 26 days to the report and 70% of the grade on report + presentation, this reorders the plan completely — which is Deliverable 2.
