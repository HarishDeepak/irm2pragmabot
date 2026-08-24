# TASKS, GAPS & READY-TO-PASTE PROMPTS

**Companion to `HANDOFF_SESSION.md`.** Read that first for verified facts.
**Written:** 2026-08-22 · **Last re-verified:** 2026-08-22 (no code changes or
lab session since — all tasks below still OPEN, none completed) · **Report due:** 2026-09-14

Each task below has: what, why it matters for the **grade**, effort, whether it
needs the robot, a **recommended model + effort level**, and a **paste-ready
prompt** for a fresh chat. Work several in parallel across chats.

---

## How to pick a model

| Model | Use for |
|---|---|
| **Opus 5** (high/max effort) | Architecture decisions, debugging that needs real reasoning, anything touching the graded memory system, research with contradictory sources |
| **Sonnet 5** (medium) | Mechanical ports, adding logging, writing tests, doc writing from known facts |
| **Haiku 4.5** | Formatting, renaming, small mechanical edits |

**Rule of thumb:** if getting it wrong costs robot time or grade marks, use Opus.

---

# TRACK A — Experiments & the graded contribution
*This is what you are actually marked on. Highest priority.*

## A1. Per-trial JSONL logging ⭐ DO THIS FIRST
**Robot:** ❌ No · **Effort:** 4h · **Model:** Sonnet 5, medium

> ### ✅ DONE 2026-08-24 (commit `1e927a6`)
>
> `pragmabot/src/pragmabot/trial_logger.py` + wiring in `pragmabot_node.py`
> (no protected module touched). `scripts/test_trial_logger.py`: 8 checks, passing.
>
> One finding worth writing up: retrieval similarities were being **discarded**
> at the call site — `self.ltm, _, _, _, _ = retrieve_relevant_experiences(...)`.
> Those are exactly what Fig. 7 is computed from. Silent data loss, now fixed.
>
> Logged per step: skill, target, placement, chain-of-thought,
> `ltm_retrieved_scenarios[]`, `ltm_similarities[]`, STM size, exec success +
> message, VLM action/task verdicts, tokens, latency. Per trial: config
> snapshot at start; outcome on all four paths (completed / max_steps /
> crashed / aborted). `load_trials()` → tidy DataFrame in Table II shape.
>
> **REMAINING:** the Gradio ground-truth control calling
> `TrialLogger.label_step()`. Until it exists `human_gt_action_success` is
> always None and **no FP/FN confusion matrix can be built** — that is the
> paper's 5% FP / 6.67% FN comparison, so it matters.

**Why it is #1:** On Sept 10 you cannot re-run experiments. Today
`save_conversation_log()` only fires in `main()`'s `finally` — a crash bypasses it —
and it contains **no retrieval similarities, no ground-truth labels, no timings,
no token counts**. None of the numbers a results table is made of.

**Gap:** every experiment you run before this exists is wasted.

```
Read extras/HANDOFF_SESSION.md first.

Build a per-trial JSONL logger for the PragmaBot reproduction, so that
experiment runs produce data a report can be written from three weeks later.

Requirements:
- One JSONL file per trial in pragmabot/data/logs/, named
  <timestamp>_<uuid6>.jsonl. Append one line per step, flush immediately —
  it must survive a crash mid-run.
- Wire it into pragmabot_node.py at: plan, execute, evaluate, summarise.
- Per-step fields that CANNOT be reconstructed later:
  time_step, chosen_skill, target_object, placement_object,
  chain_of_thought_reasoning, ltm_retrieved_scenarios[], ltm_similarities[],
  stm_len_chars, exec_success, exec_message, vlm_is_action_successful,
  vlm_is_task_completed, human_gt_action_success, input_tokens,
  output_tokens, latency_s, before_img_path, after_img_path, error.
- Trial-end record: outcome in {completed, max_steps, aborted, crashed},
  n_steps, total_tokens, wall_time_s, notes.
- human_gt_action_success needs a control in the Gradio UI — the paper
  reports FP/FN rates, so a human label per step is required for a
  confusion matrix. This cannot be added after the fact.
- Do NOT modify the 14 protected modules listed in the handoff.

Then show me how to load a directory of these into pandas and produce the
paper's Table II shape.
```

---

## A2. Retrieval ablation — the one offline experiment ⭐
**Robot:** ❌ No · **Effort:** 1–2 days · **Model:** Opus 5, high

**Why:** the only paper experiment that needs no robot (Fig. 7 measures
*first planned action accuracy* — no execution, no before/after). Your one
bankable result if robot access never materialises.

```
Read extras/HANDOFF_SESSION.md first.

Reproduce PragmaBot's memory-retrieval ablation (paper Fig. 7) offline.
It compares first-planned-action accuracy across retrieval strategies:
RAG top-k vs entire-LTM vs random-k. It executes NOTHING, so no robot.

Config knobs already exist: retrieval_top_k, use_random_retrieval,
activate_ltm in pragmabot/config/config.yaml.

I need:
1. A runner script that, for each scenario and each condition, calls the
   planner once and records the first chosen action + retrieval similarities.
2. A ground-truth labelling scheme for "correct first action" — propose one
   and justify it; the paper is vague here.
3. The accuracy plot.
4. An honest statement of what differs from the paper: they used
   text-embedding-3-large (3072-d), we use all-MiniLM-L6-v2 (384-d, local).
   MiniLM also truncates at 256 word-pieces while our scenario key is
   instruction + full scene description — so retrieval may be keyed on a
   truncated prefix. Quantify this rather than hiding it.

The VLM is stochastic (claude-opus-4-8 REMOVES the temperature parameter —
setting it returns HTTP 400). Design the protocol around that: N>=3 repeats,
fixed scenario order, report mean +/- range.
```

---

## A3. Bootstrap LTM honestly via replay runs
**Robot:** ❌ No · **Effort:** 1 day · **Model:** Opus 5, high

**Why:** LTM is now empty (fabricated entries archived). The paper had 100
entries. Replay runs produce **genuine** entries — real images, real Claude
reasoning, real summarisation; only the arm motion is absent.

**Gap:** no LTM → no LTM ablation → no Table III.

```
Read extras/HANDOFF_SESSION.md first, especially section 3 on why the
previous ltm.csv was archived.

I need to bootstrap a long-term memory honestly, with almost no robot time.

Plan and implement a replay-based LTM bootstrap:
- rosbag_replay: true runs the full VLM loop with no robot.
- Every entry written must carry provenance: "replay" (the column already
  exists in data/ltm/ltm.csv).
- We have ONE bag: bags/red_cup/red_cup_0.db3 (~118 RGB frames, ~13s) plus
  extracted/red_cup/ and extracted/cup/.

Address honestly: with one bag, how many genuinely DISTINCT scenarios can we
produce? Is varying only the instruction text enough for defensible LTM
diversity, or is that self-deception? If it is not enough, say so and tell me
the minimum robot time needed instead.

Also tell me how to disclose this in the report so it reads as a stated
limitation rather than a hidden shortcut.
```

---

## A4. Anthropic client robustness
**Robot:** ❌ No · **Effort:** 1h · **Model:** Haiku 4.5

```
In pragmabot/src/pragmabot/claude_vlm_client.py:
1. Set max_retries=5 on the Anthropic client (default is 2; one 529 kills a trial).
2. Return output_tokens as well as input_tokens — half the cost table is
   currently missing.
3. Pin output_config effort explicitly rather than relying on the default,
   to remove one axis of run-to-run drift.
4. Add a comment that temperature is REMOVED on claude-opus-4-8 and returns
   HTTP 400 — so nobody "fixes" determinism by adding it.

Do not change anything else in that file. Show me the diff before applying.
```

---

# TRACK B — Making robot time productive
*Do these BEFORE the next lab session, so the session collects data instead of debugging.*

## B1. Measure the table plane + settle `eef_link` ⭐ LAB, 10 min
**Robot:** ✅ Yes · **Effort:** 10 min · **Model:** n/a — just run the commands

**Why:** blocks all workspace work, and settles the **highest-risk unverified
assumption** in the repo.

```bash
# 1. Jog until the fingertips just touch the tabletop, then:
ros2 run tf2_ros tf2_echo fr3_link0 fr3_hand_tcp     # record z = table plane

# 2. Settles fr3_hand vs fr3_hand_tcp:
ros2 run tf2_ros tf2_echo fr3_hand fr3_hand_tcp      # expect 0 0 0.1034

# 3. Which tip link does the planning group actually use?
ros2 param get /move_group robot_description_semantic | grep -A3 'fr3_arm'

# 4. Confirm the calibration is live:
ros2 run tf2_ros tf2_echo fr3_link0 zed_left_camera_frame_optical
```

Also place an object at each corner of the usable area and read the x/y range —
that sizes the workspace box empirically. **Record all outputs in the report.**

---

## B2. Workspace restriction — one box, two consumers
**Robot:** ⚠ Needs B1 first · **Effort:** 1 day · **Model:** Opus 5, high

**Why:** two problems, one root cause — nobody told the system where the workspace
is. Perception: the e-stop scored 0.48 vs your cube's 0.57. Motion:
`avoid_collisions=True` with an **empty** planning scene means MoveIt avoids nothing.

```
Read extras/HANDOFF_SESSION.md first — sections 4 and 5 have the verified
calibration numbers and two traps to avoid.

Implement workspace restriction as ONE box in fr3_link0, consumed by BOTH the
perception crop and the MoveIt planning scene, so they can never disagree.

1. pragmabot/config/workspace.yaml — single source of truth. table_z starts
   as null and MUST be filled from the lab measurement; never guess it.
2. calibration/mask_to_pointcloud.py — add --tf and --workspace. Transform the
   cloud to fr3_link0, drop out-of-box points, transform back (GraspGen's
   contract is unchanged). Insert BEFORE the existing z-outlier/AABB stages.
   MUST stay pure numpy — this file runs in two venvs sharing no deps.
3. bridge_node.py — at startup add table + 4 walls as collision boxes via the
   /apply_planning_scene SERVICE (not the topic: a scene published before
   move_group subscribes is silently lost). Set pose in primitive_poses, not
   CollisionObject.pose (that field has a "not in use yet" disclaimer in
   Humble). Use thin BOXes, not planes.
4. Add the two missing safety lines to _compute_cartesian_path:
   max_velocity_scaling_factor = 0.1 and max_acceleration_scaling_factor = 0.1.
   _move_to_pose already scales to 0.2; the Cartesian approach/lift/retreat —
   the motions that move TOWARD the table — currently run unscaled.

DO NOT use WorkspaceParameters (verified silent no-op on a fixed-base arm) or
FR3 Desk SLP-C (disables FCI, kills the ROS 2 stack). Do not add RANSAC plane
fitting or octomap — reasons in the handoff.

Verify: the real cup cloud must survive the crop with its 6.8cm height and
9.8x9.3cm footprint intact.
```

---

## B3. Observation-pose retract ⭐ protects the graded mechanism
**Robot:** ✅ Yes · **Effort:** 4h · **Model:** Opus 5, high

**Why this is subtle and important:** `vlm_success_detector.py:121` tells the VLM
*"This image is captured after the robot completed the action and returned its arm
to the default position."* **No code does that.** Your camera is fixed on a tripod,
so the arm stays in frame and likely occludes the object — and you are *asserting*
it does not. The VLM will trust the caption over the pixels.

This maps onto the paper's **dominant** failure mode ("object still on table but
visually enclosed in gripper", their 5% FP). Your geometry makes it worse.

```
Read extras/HANDOFF_SESSION.md first.

Add an observation-pose retract to bridge_node.py so the arm never occludes
the before/after images the VLM success detector compares.

- Joint-space goal (JointConstraint), NOT Cartesian: always has an IK
  solution, never fails near a singularity, always yields the same pixels.
- Use the real measured config from CLAUDE.md:
  [-0.181, -0.420, 0.005, -2.260, -0.008, 1.806, 0.609]
  Make it a node parameter.
- Call it at the END of every skill (clean "after" frame) and once before the
  first observation (so "before" matches).
- Add a settle delay before capture — the ZED runs ~9Hz and the arm may still
  be decelerating. Put it inside SceneObserver.get_scene_observation() so both
  call sites get it and a future third one cannot forget.

This makes an existing claim in vlm_success_detector.py TRUE — that file is on
the never-modify list, so the code must meet the prompt, not the reverse.

Then tell me the residual false-positive/false-negative modes that remain even
after this fix, so I can name them in the report.
```

---

## B4. Wire perception at runtime — the biggest correctness hole
**Robot:** ⚠ Testable offline · **Effort:** 1–2 days · **Model:** Opus 5, max

> ### ✅ WIRED as of 2026-08-24 (commits `d6c03aa`, `ea1ad15`, `aa35384`)
>
> `bridge_node` now resolves `target_object` live per pick via
> `live_perception.py` (PerceptionClient :5557 → GraspGenClient :5556).
> `use_live_perception:=false` falls back to the old `grasp_file` for replay.
> The perception server is verified end to end against `extracted/red_cup/`
> (conf 0.937, 2000 pts, extent matching the earlier lab run exactly).
>
> **Two things still needed before a real pick:**
> 1. **Inject `bridge._scene_source`** — a callable returning
>    `(rgb, depth, intrinsics)`. Left None on purpose: the bridge must not
>    open a second subscriber on the same BEST_EFFORT topics the planner's
>    `SceneObserver` already reads. Until injected, picks fail with a reason.
> 2. **Run both servers on the lab GPU** and do one real pick.
>
> Earlier notes from the first half of this task:
>
> **Built:** `calibration/perception_server.py` (ZMQ REP, port 5557, loads
> SAM2+DINO once), `calibration/perception_client.py` (no torch/cv2 — safe to
> import in the ROS 2 env), `scripts/test_perception_guards.py` (9 checks,
> passing, verified to fail when a guard is weakened).
>
> **All five detection guards are in**, each returning a REASON string for STM:
> zero matches · ambiguous top-2 (<0.15, the e-stop case) · mask area outside
> 0.1–40% · fewer than 200 points · exception-to-reason.
>
> **STILL TO DO — this is the remaining work:**
> 1. `bridge_node.py`: delete the `grasp_file` parameter; call
>    `PerceptionClient.detect(rgb, depth, K, request.target_object)`, then
>    `GraspGenClient.infer(cloud)` on 5556. **The GraspGen half is just an
>    import** — `from grasp_gen.serving.zmq_client import GraspGenClient` —
>    NVIDIA ships that server at `D:/irm2pragmabot/GraspGen/client-server/`.
> 2. Run `perception_server.py` for real. It has **never been executed** — no
>    GPU or groundedsam venv on the laptop. Guard logic is tested; the
>    SAM2/DINO path is not.
> 3. Verify which gripper config GraspGen serves (`graspgen_franka_panda.yml`?)
>    — grasp origin conventions differ per gripper, same 10 cm base-vs-fingertip
>    trap as `fr3_hand` vs `fr3_hand_tcp`.

**Why:** `bridge_node.py` reads a `grasp_file` **parameter**. The action carries
`target_object` as text and the bridge **never uses it**. So:

> Planner says "pick the red cup" → robot picks whatever was segmented in the
> last offline run.

Worse, the success detector may then report True (*something* moved), so the
memory system records a **fabricated success**. **This corrupts the graded
contribution, not just the grasping.**

```
Read extras/HANDOFF_SESSION.md first.

Wire the perception pipeline into runtime execution. Right now bridge_node.py
reads a pre-computed grasp_file parameter and ignores the action's
target_object entirely — so "pick the red cup" picks a stale npz.

Constraint that shapes the design: three incompatible Python environments.
GraspGen (torch==2.1.0, py3.10), Grounded-SAM-2 (torch>=2.3.1), and the ROS 2
env cannot share. GraspGen ALREADY ships a ZMQ client/server and it works.

Proposed: one long-lived ZMQ server per venv; bridge_node is the client.
  bridge --REQ--> perception_server.py (groundedsam venv, port 5557)
                    detect_object -> mask_to_pointcloud -> object_pcd.npy
  bridge --REQ--> graspgen_server.py (graspgen venv, port 5556)  [exists]

Evaluate that against alternatives (subprocess per call, a ROS 2 service in
each venv) and justify the choice. SAM2 + GroundingDINO load is ~10s and must
not be paid per pick.

Also add detection guards, since a wrong object is worse than no object:
- zero boxes above threshold -> fail loudly, never fall through to a stale file
- conf[0] - conf[1] < 0.15 -> ambiguous, fail naming BOTH candidates
- mask area outside [0.1%, 40%] of the image -> reject
- fewer than ~200 surviving cloud points -> depth failed, reject

Every failure must return a REASON string: the planner self-reflects on that
text, so it feeds the graded memory system rather than just a log.

This is fully testable against bags/red_cup/ with no robot.
```

---

## B5. Gripper holding-state guard
**Robot:** ✅ Yes · **Effort:** 3h · **Model:** Sonnet 5, medium

**Why:** the planner's only defence against "place with nothing held" or "pick
while already holding" is a *prompt line* — a soft constraint on a stochastic
model, guarding a 7-DoF arm.

```
Add a gripper holding-state guard to bridge_node.py.

Read finger positions from /franka_gripper/joint_states (verify the topic and
joint names first with `ros2 topic list | grep gripper` and
`ros2 topic echo /franka_gripper/joint_states --once` — I could not confirm
them, and each joint may carry HALF the width).

Refuse, before dispatch:
- pick while already holding -> "pick refused: gripper already holds an object"
- place while empty -> "place refused: gripper is empty, nothing to place"

Use the gripper as ground truth rather than a bookkeeping flag — it survives a
node restart, which _last_grasp_T_base does not.

The refusal message goes into STM and the VLM reasons about it. That is the
paper's self-reflection mechanism doing real work, so treat the message text
as a deliverable, not a log line.
```

---

## B6. Grasp candidate filtering & ranking
**Robot:** ⚠ Partly offline · **Effort:** 1–2 days · **Model:** Opus 5, high

**Why:** current code commits to ONE grasp and aborts on failure — discarding ~50
candidates of information. The paper filters by IK feasibility (Pinocchio) and
scores `g* = argmax s_conf(g) · s_loc(g)`.

**Note:** grasping is NOT graded. Do this only if A1–A3 are done. Target
"robust enough", not research-grade.

```
Read extras/HANDOFF_SESSION.md first.

Improve grasp selection in bridge_node.py. Today it picks the most top-down
candidate and aborts if planning fails, throwing away ~50 GraspGen candidates.

Implement, in this order of value:
1. Hard IK filter: /compute_ik (moveit_msgs/srv/GetPositionIK) on BOTH the
   grasp and its standoff; reject if either fails. Use timeout=50ms (the 5s
   default would be 500s for 50 candidates x2) and seed from the known-good
   config [-0.181,-0.420,0.005,-2.260,-0.008,1.806,0.609] rather than the
   current state, so results do not depend on where the arm is parked.
   ik_link_name MUST match the link_name passed to /compute_cartesian_path.
2. Joint-limit margin. FR3 limits are ASYMMETRIC and never contain zero on
   joint4 (-3.0770..-0.1169) and joint6 (0.4398..4.6216) — the Panda-era
   symmetric assumption is wrong on exactly the two joints that matter for
   top-down grasping. Reject margin < 0.05, using min over joints.
3. Project candidates to TOP-DOWN 4-DoF (x,y,z,yaw) and prefer those; keep
   6-DoF as ranked fallback. For cups and cubes this loses nothing and removes
   the dominant wrist-singularity mode.
4. Retry loop over the top 5, with a 2cm/15deg diversity filter — otherwise
   the top 5 are five near-identical grasps and "five attempts" is one
   attempt tried five times.
5. Return a failure REASON per candidate ("candidate 3: Cartesian fraction
   0.4, joint4 near limit") — that text feeds STM.

Also: revolute_jump_threshold may be silently ignored in MoveIt Humble
(moveit2 issue #2404 says CartesianPathService does not forward it). VERIFY
this against the installed version, and if true, validate the returned
trajectory client-side instead. Right now we may believe we have a
singularity guard that does nothing.
```

---

# TRACK C — Report & presentation
*40% + 30% of the grade. Do not leave to the last week.*

## C1. Start the blogpost-format report NOW
**Robot:** ❌ No · **Effort:** ongoing · **Model:** Opus 5, high

```
Read extras/HANDOFF_SESSION.md first.

Draft the skeleton of my iRobMan Praktikum report. Format: BLOGPOST (course
requirement), due 2026-09-14. Project 3, "Memory representations for Robotic
Task Planning", reproducing PragmaBot on a Franka FR3.

Give me a section structure with, for each section, exactly which artefact or
number fills it — so I can see today which experiments are load-bearing and
which gaps would be fatal.

Be direct about scope: I have ~23 days and uncertain robot access. A faithful
PARTIAL reproduction with one rigorous ablation is a better report than a
broad shallow one. Structure it that way.

Deviations from the paper that must be disclosed, not buried:
- eye-on-BASE tripod camera vs their elbow-mounted camera
- all-MiniLM-L6-v2 (384-d, truncates at 256 word-pieces) vs
  text-embedding-3-large (3072-d)
- Claude instead of gpt-4o
- GraspGen instead of AnyGrasp
- far fewer trials than their 91
- push out of scope (supervisor's call)
Tell me how to frame each as a considered engineering decision with evidence.
```

---

## C2. Ontology memory extension (only if reproduction is solid)
**Robot:** ❌ No · **Effort:** 2–3 days · **Model:** Opus 5, max

**Why it fits the title:** your LTM squashes each experience into one 384-number
vector; retrieval is "nearest vector", and the robot cannot say *why* it retrieved
something. An ontology stores structured facts —
`(cup, is_a, container)`, `(occlusion, resolved_by, push_away)` — so it
generalises through **relationships**.

The paper's own Fig. 6: **4 of 19 failures were RAG retrieving the wrong memory**,
and §VI admits top-k "may become ineffective" as memory grows. This attacks the
named weakness and is literally your project title — *Memory **representations***.

```
Read extras/HANDOFF_SESSION.md first.

Design (do not yet implement) an ontology-based long-term memory as an
alternative representation to PragmaBot's flat embedding + cosine top-k.

Constraints:
- memory_manager.py is on the never-modify list. The ontology must be an
  ADDITION alongside it, comparable in the same harness.
- Must be evaluable with the offline retrieval ablation (Track A2) so it needs
  no robot time.
- ~1 week of part-time work maximum.

Cover: what the triples are, how a VLM summary becomes triples, how retrieval
works, and — most important — what MEASURABLE comparison against the
embedding baseline would be publishable in a student report.

Be honest about whether this is achievable in the time left, or whether a
smaller framing (e.g. hybrid: embedding retrieval + structured re-ranking)
gives a better result per day spent.
```

---

# GAPS — things nobody has looked at yet

| Gap | Risk | Where |
|---|---|---|
| **Depth encoding unvalidated** | `passthrough` gives float32 metres for ZED, but 16UC1 **millimetres** in some configs — a silent 1000× error | `scene_observer.py` |
| **`slop=1.0` is very loose** | RGB and depth up to 1s apart still "synchronised" — ~9 frames at 9Hz | `scene_observer.py` |
| **`should_*_at_specific_section` unused** | Carried through the action, never read — dead fields that look implemented | `ExecuteSkill.action` |
| **No `trial_id` in the action** | Bridge logs and planner logs cannot be joined afterwards | `ExecuteSkill.action` |
| **`place_offset_xyz` is fixed** | "Place on the plate" is really "place 20cm to the left". Flagged as debt; **must be stated in the report** | `bridge_node.py` |
| **Docker/DDS across the boundary** | MoveIt in container, planner on host. QoS mismatch shows **no error**, just silence | ops |
| **Disk full mid-experiment** | `/` on Alonnisos has hit 100%; would corrupt the one artefact you cannot regenerate | ops |
| **Lighting / auto-exposure** | Before/after brightness shift can read as "the scene changed" | perception |
| **Cup is the worst test object** | Reflective + transparent + concave — stereo fails on all three. Use a matte cube; keep the cup as a reported hard case | perception |

---

# Suggested parallel split across chats

| Chat | Track | Model |
|---|---|---|
| 1 | A1 logging → A2 retrieval ablation | Sonnet → Opus high |
| 2 | B4 perception wiring (offline-testable) | Opus max |
| 3 | C1 report skeleton, updated weekly | Opus high |
| 4 | B2 workspace (after B1 measurement) | Opus high |

**Do not** run B2 before B1 — every number depends on the measured table plane.
