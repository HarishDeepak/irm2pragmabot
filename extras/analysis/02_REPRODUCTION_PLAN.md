# Deliverable 2 — Reproduction & Completion Plan

**Date:** 2026-08-19 · **Report due:** 2026-09-14, 23:59 CET (**26 days**) · **Presentation:** 2026-09-21/22
**Grading:** 30% supervisor · 40% report · 30% presentation

Every claim about your code cites `path:line`. Paths are relative to `D:/pb/home/`.

---

## 1. Gap analysis — the paper's pipeline vs what the code does today

Read from source, not filenames.

### 1.1 Step by step

| # | Paper (Algorithm 1) | What exists | Verdict |
|---|---|---|---|
| 1 | $\mathcal{D}(o_0)$ — scene description | `vlm_scene_describer.py`, called `pragmabot_node.py:121` | ✅ upstream, works |
| 2 | RAG top-$k$ over LTM | `memory_manager.py:84-138`, called `pragmabot_node.py:128` | ✅ upstream, works |
| 3 | $\mathcal{P}$ — plan next action | `vlm_task_planner.py:88-128`, called `pragmabot_node.py:135` | ✅ upstream, works |
| **4** | **Execute $a_t$** | `bridge_node.execute_pick()` `:111-313` exists; **`execute_place()` `:315` and `execute_push()` `:318` raise `NotImplementedError`**; **nothing routes the planner's action into it** | ❌ **the gap** |
| 5 | $\mathcal{R}$ — visual success detection | `vlm_success_detector.py:45-60`, called `pragmabot_node.py:167` | ✅ upstream, works |
| 6 | STM append + reflect | `pragmabot_node.py:136`, `:173` | ✅ upstream, works |
| 7 | $\mathcal{S}$ — summarise into LTM | `pragmabot_node.py:187-196` | ✅ upstream, works |

**Six of seven steps are done and were never the problem. Step 4 is the whole project.**

### 1.2 Inside step 4 — what the paper does vs what the bridge does

| Paper §IV-F | `pragmabot_bridge` today | Gap |
|---|---|---|
| Grounded SAM → object mask | `detect_object.py`, run **manually** from the shell | Not callable from the node; no ROS interface |
| Point cloud from mask | `mask_to_pointcloud.py`, run **manually** | Same |
| **AnyGrasp** → grasp hypotheses + $s_{conf}$ | **GraspGen** via ZMQ, run **manually**, saved to `grasps.npz` | Model substitution is fine (the released README recommends GraspGen). But it is an offline file hand-off, not a live call |
| IK filter (Pinocchio) → feasible set $G$ | **none** | MoveIt `/compute_ik` available, unused. Infeasible grasps are only discovered when `MoveGroup` fails at `bridge_node.py:230` |
| On-demand annotation → numbered candidates | `test_fps.py` standalone; planner flags ignored | Not wired |
| $g^* = \arg\max s_{conf}\cdot s_{loc}$ | `select_topdown_index` `grasp_transform.py:49-59` — most top-down | **Kinematic heuristic, not semantic.** $s_{conf}$ is saved but unused for selection; $s_{loc}$ does not exist |
| pick / place / push | pick only | 2 of 3 missing |

### 1.3 The structural gap, precisely

- **Brain is ROS 1.** `panda_skill_executor.py:1-2` (`rospy`, `actionlib`) → `/panda/pick|place|push` `:15-17`.
- **Body is ROS 2.** `bridge_node.py` is `rclpy` and exposes **no interface**: `main()` `:476-507` reads a `grasp_file` parameter, runs one `execute_pick()`, spins.
- **The intended contract generates nothing.** `pragmabot_interfaces/CMakeLists.txt:16-22` — `rosidl_generate_interfaces` commented out, no `action/` directory, open decision recorded in the comment.

Plus the schema defect (**[VERIFIED HERE]**): `panda_skill_executor.py:30` calls `action.skill`, which does not exist — the field is `chosen_skill` (`vlm_task_planner.py:107`). Raises `AttributeError`. `:63`/`:79` read `target_location`, which is `placement_object` (`:116`), failing *silently* to `""`. Unreached today only because `rosbag_replay: true` short-circuits at `pragmabot_node.py:139-141`.

### 1.4 What the bridge does that the paper does not

Worth stating, because it is contribution, not deviation:

- **Depth de-noising before grasp synthesis** — `mask_to_pointcloud.py` erosion `:67` + MAD `:81` + AABB `:113`. Attacks the paper's largest failure bucket.
- **Per-grasp gripper width from the cloud** — `grasp_transform.py:98-148`, measured 12–33 mm across six grasps on one object.
- **Robot-frame top-down selection** — `:49-59`, correctly noting camera tilt makes "top-down" meaningless in camera frame.
- **Gripper fault recovery** — homing + one retry, `bridge_node.py:173`, `:256-262`.
- **Singularity guard** — `:404-412`, abort rather than execute a truncated path `:237-247`.

---

## 2. The planning insight that reorders everything

**The memory experiments do not need the robot.**

`pragmabot_node.py:139-141`: when `rosbag_replay: true`, execution is skipped entirely and the success detector is called directly on two observations. And the retrieval ablation (paper §V-D) measures **first-action accuracy without execution at all**.

So of the six reproduction requirements in Deliverable 1:

| | Needs robot? |
|---|---|
| R2 STM helps | **No** — rosbag replay |
| R3 LTM+RAG helps | **No** — rosbag replay |
| R4 retrieval ablation | **No** — no execution at all |
| R5 local VLM | **No** |
| R6 ontology memory | **No** |
| R1 closed loop on FR3 | **Yes** |

**Five of six are achievable from this laptop.** With 26 days to a report worth 40% and a presentation worth 30%, the plan below front-loads everything that does not need lab access, so that lab time — whenever it comes — is spent only on R1.

---

## 3. Ordered next steps

Each step has a **done-condition** that is checkable. No step is "investigate."

### Phase 0 — Unblock (today, ~2 hours) `[HOME]`

**Step 0.1 — Retarget camera topics.** `[HOME]`
Edit `pragmabot/config/config.yaml` topics from `/zedxm/zed_node/...` to `/zed/zed_node/rgb/image_rect_color`, `/zed/zed_node/depth/depth_registered`, `/zed/zed_node/rgb/image_rect_color/camera_info`.
**Done when:** `ros2 topic list` names match the config exactly, and replaying `bags/red_cup/red_cup_0.db3` makes `scene_observer.get_scene_observation()` return a non-`None` image.

**Step 0.2 — Fix the executor schema defect.** `[HOME]`
In `panda_skill_executor.py`: `:30` → `action.chosen_skill.value`; `:63`/`:79` → `action.placement_object`; delete the dead `"done"` branch `:38-40`.
**Done when:** a unit test constructs a real `NextBestAction(chosen_skill=RobotSkill.PICK, ...)`, passes it to `execute()`, and no `AttributeError` is raised and no goal field is empty.

**Step 0.3 — Pin the environment.** `[HOME]`
Add the packages imported but absent from `requirements.txt`: `pyzmq`, `msgpack`, `msgpack-numpy`, `sentence-transformers`, `anthropic`, `google-generativeai`.
**Done when:** `pip install -r requirements.txt` in a clean venv is followed by a successful `import` of every module in `src/pragmabot/`.

---

### Phase 1 — Reproduce the retrieval ablation (days 1–3) `[HOME]` ← **start here**

The cheapest headline result in the paper, no robot, and the one whose mechanism generalises furthest.

**Step 1.1 — Build an LTM of your own.** `[HOME]`
Author 30–60 entries in `data/ltm/ltm.csv` covering your intended task families (occlusion, stacking, container-with-contents, small/flat object). Generate embeddings via the configured backend.
**Done when:** `MemoryManager.n_ltm_entries` ≥ 30 and `n_embedding_entries == n_ltm_entries` (no `None` embeddings — `memory_manager.py:56`, `:74`).

**Step 1.2 — Run the three-condition ablation.** `[HOME]`
For 10–12 held-out scenarios, measure first-planned-action accuracy under `retrieval_top_k: 5` / `retrieval_top_k: -1` (all) / `use_random_retrieval: true` — all three already exist as config switches, and random retrieval is implemented at `memory_manager.py:116`.
**Done when:** a table of three accuracies over ≥10 scenarios exists, with per-condition prompt-token counts and response times logged (both already returned by `retrieve_relevant_experiences` `:132-138`).

**Step 1.3 — Report it against the paper.** `[HOME]`
**Done when:** a figure mirroring the paper's Fig. 7 exists, with your numbers beside theirs (89 / 74 / 17) and trial counts stated.

> This alone is a defensible report section, produced without a robot.

---

### Phase 2 — Close the loop offline (days 3–8) `[HOME]`

**Step 2.1 — Port the planner to ROS 2.** `[HOME]`

> **This recommendation was revised on 2026-08-19** after measuring how deep the ROS 1 coupling actually goes. The earlier recommendation was a ZMQ bridge, chosen to *avoid* a port assumed to be large. It is not large. **Port instead, and go all-ROS 2.**

**The measurement that settles it.** Of the 17 library modules in `pragmabot/src/pragmabot/`, only **three** import ROS:

| ROS 1-coupled | Lines | Disposition |
|---|---|---|
| `scene_observer.py` | 92 | **port** to `rclpy` subscriptions |
| `panda_skill_executor.py` | 93 | **rewrite** (required anyway — it carries the schema bug in §1.3) |
| `graspgen_client.py` | 95 | **drop** — the standalone GraspGen client already covers this |

Everything else is **pure Python with no ROS dependency at all** — `vlm_task_planner.py` (206), `memory_manager.py` (308), `vlm_success_detector.py` (130), `vlm_exp_summarizer.py` (100), `vlm_scene_describer.py` (95), `vlm_client.py` (93), `conversation_builder.py` (129), `claude_vlm_client.py` (120), `gemini_vlm_client.py` (125), `utils.py` (67), `geometry.py` (85), `grounded_sam.py` (76), `simple_config.py` (29).

**≈1,400 lines of the paper's actual contribution port for free.** They are not touched at all — which satisfies the project's "never modify upstream VLM modules" rule perfectly rather than by exception.

Node-level work:

| File | Lines | Action |
|---|---|---|
| `pragmabot_node.py` | 423 | **port** — `rospy` → `rclpy`, keep the Gradio UI |
| `scene_observer.py` | 92 | **port** — message-filter sync → `rclpy` |
| `panda_pick_server.py` / `place` / `push` | 553 | **delete** — ROS 1 action servers, already superseded by `pragmabot_bridge` |
| `memory_manager_node.py` | 296 | optional (LTM inspection UI) — port later or run standalone |
| `image_decompressor_node.py`, `image_republisher_node.py` | 178 | optional — rosbag replay helpers; ROS 2 bag replay may remove the need |

**Realistic port surface: ~515 lines**, plus the new executor you must write regardless.

**Why this beats the alternatives:**
- vs **`ros1_bridge`** — no second ROS distribution to install, no bridge process to keep alive, no message-type mapping to maintain, no dual-sourcing of environments.
- vs **ZMQ** — ZMQ remains right for GraspGen (genuinely incompatible CUDA env, and it ships a server). It is the *wrong* tool between planner and executor, where both sides would be Python in the same ROS graph and you would be hand-rolling what ROS 2 actions already give you: goals, feedback, cancellation, and introspection with `ros2 action`.
- **Single workspace, single build system.** `colcon` builds `pragmabot`, `pragmabot_bridge`, and `pragmabot_interfaces` together. The rsync-into-`franka_ros2` workaround (needed because `/pragmabot` is mounted outside the colcon build path, §3.4 of the master reference) disappears.
- **`ROS_DOMAIN_ID=7` already unifies discovery** across the container and host, so planner and executor see each other with no extra transport.

**Done when:** `ros2 run pragmabot pragmabot_node` starts under `rclpy`, the Gradio UI loads, and a rosbag-replay planning step completes end to end — with **zero diffs** to the 13 pure-Python modules listed above (verify with a diff against their originals).

**Step 2.2 — Define the skill contract as a ROS 2 action.** `[HOME]`
Enable `rosidl_generate_interfaces` in `pragmabot_interfaces/CMakeLists.txt:16-22` and add **one collapsed `ExecuteSkill.action`** rather than three separate ones — the open decision recorded in that comment. One action is right here because the three skills share a dispatch path in `bridge_node.py` and the planner emits a single `chosen_skill` enum; three actions would mean three servers and three clients for no gain.

Carry the *actual* schema fields: `chosen_skill`, `target_object`, `placement_object`, `push_direction`, `should_grasp_at_specific_section`, `should_place_at_specific_section`. **Do not** use `PROJECT_OVERVIEW.md` §6's schema — it is wrong (see §1.3), and writing against it reproduces the existing bug.

```
# ExecuteSkill.action
string  chosen_skill                        # "pick" | "place" | "push"
string  target_object
string  placement_object                    # "" when not a place
string  push_direction                      # "" when not a push
bool    should_grasp_at_specific_section
bool    should_place_at_specific_section
---
bool    success
string  message
---
string  status                              # feedback
```
**Done when:** `colcon build --packages-select pragmabot_interfaces` succeeds and `ros2 interface show pragmabot_interfaces/action/ExecuteSkill` prints all six request fields.

**Step 2.3 — Implement both ends against the contract.** `[HOME]`
Server side in `bridge_node.py`: an `ActionServer` that maps `chosen_skill` onto `execute_pick()` (and later place/push). Client side: rewrite `panda_skill_executor.py` as an `rclpy` `ActionClient`, reading `chosen_skill` — **not** `skill` (see §1.3).
**Done when:** with the node running in a mocked mode (no robot), a `ros2 action send_goal` carrying a real `NextBestAction`'s fields is accepted, all six fields are logged intact, and a success/failure result returns. Add a schema round-trip test so this boundary cannot silently drift again.

**Step 2.4 — Automate perception→grasp as a callable step.** `[HOME]`
Wrap `detect_object.py` → `mask_to_pointcloud.py` → GraspGen client into one function taking `(rgb, depth, intrinsics, target_object)` and returning `grasps.npz`. It is currently a manual three-shell-command chain across two venvs.
**Done when:** one call on `extracted/red_cup/` reproduces `object_pcd.npy` (extent 9.5 × 10.7 × 7.3 cm — **already verified reproducible**) and returns ≥1 grasp with confidence > 0.9, with no manual step.

**Step 2.5 — Full loop in rosbag-replay mode.** `[HOME]`
**Done when:** `launch_pragmabot.launch` with `rosbag_replay: true` runs instruction → describe → retrieve → plan → (skipped execution) → detect → STM → summarise → LTM write, and `ltm.csv` gains a new row at the end.

---

### Phase 3 — Reproduce the memory claims offline (days 8–14) `[HOME]`

**Step 3.1 — STM ablation (paper Table II shape).** `[HOME]`
Run each scenario with `activate_stm: false` and `true`, ≥8 trials each, on scenarios with a designed-in first-action failure.
**Done when:** a table of per-task success with both settings exists, with n stated per cell, and at least one transcript showing the reflection text that changed the plan.

**Step 3.2 — LTM ablation (paper Table III shape).** `[HOME]`
Empty LTM vs populated, single-trial, including ≥2 structurally-similar-but-unseen scenarios.
**Done when:** a table of single-trial success for both conditions exists, and at least one transcript shows `applicable_knowledge` (`vlm_task_planner.py:95`) naming a lesson from a *different* task.

**Step 3.3 — Log the override failure mode.** `[HOME]`
The paper's most interesting failure (5/19) is the VLM retrieving the right memory and ignoring it. `applicable_knowledge` makes it observable.
**Done when:** every planning call's `applicable_knowledge` is logged, and the count of "retrieved-but-not-used" cases is reported.

---

### Phase 4 — Course extensions (days 14–20) `[HOME]`

**Step 4.1 — Ontology-based memory.** `[HOME]`
Insert at `memory_manager.py:119-123`, where retrieved entries are serialised as JSON `{"scenario", "experience"}`. Emit a structured form instead — e.g. `{"precondition", "failure_mode", "remedy", "object_class"}` — **without touching the planner**, preserving the never-modify-upstream rule.
**Done when:** the same scenarios from Step 3.2 are re-run with structured entries, and first-action accuracy plus the override rate from Step 3.3 are reported for free-text vs structured.

**Step 4.2 — Local VLM.** `[HOME]`
Add a local backend beside `claude_vlm_client.py` / `gemini_vlm_client.py`, selected by the same config substring mechanism. Baseline to beat: **~10 s** per planning call (`gpt-4o`), **~7 s** (`mini`), ~1,100 prompt tokens under RAG.
**Done when:** median planning-call latency and first-action accuracy are reported for local vs API on the same ≥10 scenarios.
**Caveat to state up front:** the local model must support **structured output** for `NextBestAction`, or the whole pipeline breaks. Verify this before committing — it is the single thing most likely to make this extension fail.

---

### Phase 5 — Robot validation (whenever lab access happens) `[LAB]`

Ordered so the riskiest, cheapest check comes first.

**Step 5.1 — Confirm the TF chain.** `[LAB]`
**Done when:** `ros2 run tf2_ros tf2_echo fr3_link0 zed_left_camera_frame_optical` prints a stable transform with `easy_handeye2 publish.launch.py name:=fr3_zed_right` running.

**Step 5.2 — Resolve `eef_link`.** `[LAB]` ← **do this before any motion**
`bridge_node.py:140-145` flags that `fr3_hand` may need to be `fr3_hand_tcp`. A wrong tip link offsets every grasp by the hand-to-TCP distance (~10 cm).
**Done when:** the planning group's actual tip link is read from `/move_group` (or the RViz MotionPlanning panel), compared against GraspGen's gripper-base convention, and the correct value is set as the parameter default.

**Step 5.3 — Dry-run the motion with no object.** `[LAB]`
Run `execute_pick()` with `home_gripper_first: true` on a saved `grasps.npz`, with the table clear.
**Done when:** the arm reaches the standoff, the Cartesian approach returns `fraction == 1.0`, and the gripper homes and closes — no object required.

**Step 5.4 — First real pick.** `[LAB]`
**Done when:** `execute_pick()` on a freshly captured scene lifts the object and `place_after_s > 0` returns it, three times consecutively.

**Step 5.5 — One closed autonomous loop (R1).** `[LAB]`
**Done when:** a natural-language instruction produces plan → real execution → visual success detection → LTM write, with no human intervention between instruction and completion.

**Step 5.6 — On-robot memory comparison.** `[LAB]`
**Done when:** ≥5 trials per condition, empty vs populated LTM, on ≥2 scenarios.

---

### Phase 6 — Report and presentation (days 20–26) `[HOME]`

**Step 6.1 — Report.** Blogpost format, due 2026-09-14.
**Done when:** submitted, covering: the paper; what was reproduced and what was not, with reasons; the retrieval ablation; the STM/LTM results; the extensions; and an honest limitations section stating trial counts.

**Step 6.2 — Presentation.** 15 minutes, 2026-09-21/22.
**Done when:** rehearsed to time with a fallback for the live demo (a recording).

---

## 4. Biggest risk, and the fallback

### The risk

**Lab access is the single point of failure, and only R1 depends on it.**

Concretely: there is no VPN to the lab network, so `robot_ip:=10.10.10.10` is unreachable from home; the hand-eye calibration result is **not in this copy** (**[VERIFIED HERE]** — a full search found no `.calib` file), so grasps cannot even be transformed into robot frame here; **pick has never run end to end on the FR3**; and the highest-risk assumption in the execution path (`eef_link`, `bridge_node.py:140-145`) can only be resolved at the robot.

Secondary, smaller risks: the Franka Hand fault mode (already mitigated at `:173`, `:256-262`); near-singular grasp poses (already guarded at `:404-412`); and Alonnisos being a shared machine that has hit 100% disk before.

### The fallback

**Make the report's core results independent of the robot, and treat R1 as the demonstration rather than the evidence.**

This is not a consolation prize. It is the correct structure given that 70% of the grade is report + presentation, and that five of six reproduction requirements never needed the robot:

1. **Primary evidence — offline.** Phases 1 and 3 give the retrieval ablation, the STM ablation, and the LTM ablation, all in rosbag-replay mode using the two captured scenes and the raw bag already on this laptop. These reproduce the paper's actual scientific claims.
2. **Contribution — offline.** The depth-quality work is already verified and reproducible here: the three-filter pipeline reproduces the shipped `object_pcd.npy` exactly (9.5 × 10.7 × 7.3 cm), turning a 17.7 cm z-extent into 7.3 cm, with the boundary-vs-interior outlier analysis (11.9% vs 1.5%) as root-cause evidence. Framed against the paper's own failure analysis — where execution is the largest bucket (8/19) and depth error is named explicitly — this is a genuine improvement on a published weakness.
3. **Extensions — offline.** Both course extensions (Phase 4) run without a robot.
4. **If lab access does happen**, Phase 5 is short and pre-planned, and R1 becomes a demonstration video for the presentation rather than the thing the report depends on.
5. **If lab access never happens**, the report states plainly what was and was not validated on hardware. Given the course's stated goal — *"you learn what it takes to fully understand a paper and reproduce its results"* — a rigorous account of a partially-validated reproduction, with the boundary drawn honestly, is a legitimate and defensible outcome.

**The one thing that would make this fail:** spending the remaining 26 days waiting for robot access instead of executing Phases 1–4. Start Phase 1 today.
