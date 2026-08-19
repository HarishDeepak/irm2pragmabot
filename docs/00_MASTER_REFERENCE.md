# PragmaBot × Franka FR3 — Master Reference

**Purpose of this document.** This is the single place to look things up so that neither you nor any future assistant has to re-read the paper, re-trace the code, or re-derive how the system fits together. It covers the paper in depth, the course project, the system as actually built on disk, the data artifacts, the facts that have been independently verified, and the traps that have already cost time.

**Status date:** 2026-08-18.
**Verification rule used throughout:** claims about your code cite `path:line`. Claims I could not verify are labelled **[UNVERIFIED]** rather than smoothed over. Facts I re-derived myself from the raw data are labelled **[VERIFIED HERE]**.

**Important scoping note.** Version-control history was deliberately *not* used as evidence — the folders under `D:/pb/home` are copies from the lab machine and their git state does not reliably reflect reality. Every structural claim below comes from reading files on disk.

---

## 0. Orientation in one page

You are reproducing an IEEE RAL 2026 paper, **PragmaBot** (ETH Zürich RSL + Huawei Noah's Ark Lab), for the TU Darmstadt **iRobMan / IRM2 project lab**, on a **real Franka FR3**.

The paper's actual contribution is a *memory and reflection layer for task planning*: a vision-language model plans a skill, watches whether it worked, writes a reflection into short-term memory, and distils finished episodes into a long-term memory that is retrieved by RAG for future tasks. The paper explicitly does **not** contribute the low-level skills — and the released code leaves action execution as `NotImplementedError`.

That single fact defines the whole project. **The paper gives you the brain; you must build the body.** Upstream ran on an ANYmal quadruped with a 6-DoF arm; you run on a fixed-base FR3 with a statically mounted ZED2. Nothing below the planner transfers.

Where the work stands today, in one sentence: **the body is built and verified up to the point of motion, the brain runs, and the two are not yet connected on the FR3.**

| Layer | State |
|---|---|
| VLM planner / STM / LTM / RAG | Upstream code present and runnable (ROS 1) |
| Perception (open-vocab segmentation) | Working, verified on real captures |
| Object point cloud from mask + depth | Working, with an original noise fix |
| Grasp synthesis (GraspGen) | Working, verified on real ZED data |
| Camera→robot calibration | Done in lab (eye-on-base); **result not present in this copy** |
| Pick execution on FR3 (ROS 2) | Code complete, **never run end-to-end on the robot** |
| Place / push execution | `NotImplementedError` |
| Planner ⇄ FR3 executor connection | **Missing — the critical gap** |
| Course extensions (local VLM, ontology memory) | Not started |

---

# PART I — THE PAPER

**A Pragmatist Robot: Learning to Plan Tasks by Experiencing the Real World**
Kaixian Qu, Guowei Lan, René Zurbrügg, Changan Chen, Christopher E. Mower, Haitham Bou-Ammar, Marco Hutter.
Robotic Systems Lab, ETH Zürich; ETH AI Center; Huawei Noah's Ark Lab London; UCL Centre for AI. IEEE RAL 2026. 8 pages.
Project page: `https://pragmabot.github.io/` · Code: `https://github.com/leggedrobotics/pragmabot`
Local full text: `C:/Users/haris/claude-papers/papers/pragmabot/paper.txt` (49,020 chars, complete, not truncated).

## 1.1 The problem it attacks

LLMs and VLMs are trained on internet data. They therefore hold a *human* prior about what is easy: "the tennis ball is right there, just pick it up." A real robot has a specific embodiment, a specific gripper, a specific set of skills, and specific failure modes. The model is not aligned to any of them.

The paper's framing question (§I): *how can a robot align a VLM with its own capabilities without fine-tuning it?*

Fine-tuning is rejected on cost grounds. The alternative they adopt is **verbal reinforcement learning**, from Reflexion [17]: the agent improves by writing natural-language critiques of its own failures into its context, not by updating weights. The paper's own analogy, which is worth internalising because it is the conceptual core:

> The planner's self-reflection is a **"linguistic gradient"** — it plays the role gradient computation plays in RL, but the update lands in the prompt instead of the weights.

Reflexion had two limitations the paper targets: it was evaluated in simulation, and its observations came either as simulator ground truth or from hand-engineered scene descriptors. PragmaBot puts it on a physical robot and closes the perception loop with a VLM that looks at actual camera images.

## 1.2 Formal setup (§III)

The robot has $K$ predefined parameterised skills $\{\pi_k\}_{k=1}^{K}$ — assumed given, whether learned by IL/RL or written as controllers. The paper is *only* about sequencing them.

A high-level skill-selection policy $\Pi_\theta$ chooses among them:

$$\pi_k \sim \Pi_\theta(I, o_t, c_t)$$

where $I$ is the natural-language instruction, $o_t$ the current observation (an RGB image), $c_t$ additional context, and $\theta$ the VLM's frozen parameters. The research question is how to improve this policy *without touching $\theta$*. The answer: put the learning in $c_t$.

## 1.3 The five components

| Symbol | Component | Role |
|---|---|---|
| $\mathcal{D}$ | VLM Scene Describer | initial RGB → natural-language scene description |
| $\mathcal{P}$ | VLM Task Planner | (instruction, image, STM, retrieved LTM) → next action |
| $\mathcal{R}$ | VLM Success Detector | (before image, action, after image) → success / task-complete / semantic delta |
| $\mathcal{S}$ | VLM Experience Summarizer | finished STM episode → one distilled LTM lesson |
| $\mathcal{E}$ | Text embedding model | scenario key → dense vector for retrieval |

All four VLM roles use **the same model**. The paper makes a point of this in Table I under "unified visual feedback": using one model for both planning and verification is what makes the loop genuine *self*-reflection rather than one model grading another.

## 1.4 Algorithm 1, line by line

```
Given: instruction I, initial observation o₀;  Internal: long-term memory M

1: K′ ← (I, D(o₀))                          build the retrieval key
2: {(Kᵢ,Eᵢ)}ᵏ⁻¹ᵢ₌₀ ← arg top-k cos(E(K), E(K′))   RAG over LTM
3: t ← 0,  m ← ∅                            reset short-term memory
4: repeat
5:   aₜ ← P(I, oₜ, m, {(Kᵢ,Eᵢ)})            plan next action
6:   execute aₜ, receive oₜ₊₁                *** THIS IS THE NotImplementedError ***
7:   rₜ₊₁ ← R(oₜ, aₜ, oₜ₊₁)                  visual success check
8:   m ← m ∪ (aₜ, rₜ₊₁)                      append to STM
9:   t ← t + 1
10: until rₜ.completed
11: M ← M ∪ {(K′, S(m))}                     distil episode into LTM
```

Read this carefully, because the structure of your entire project is dictated by **line 6**.

### The retrieval key (line 1)

$K' = (I, \mathcal{D}(o_0))$ — the instruction concatenated with a VLM-generated description of the *initial* scene. This is a deliberate design choice worth understanding: the key is semantic, not geometric. Two scenes with different objects but the same *structure* ("target object is blocked by a nearby object") produce similar keys and therefore retrieve each other's lessons. That is precisely the generalisation mechanism the paper demonstrates in §V-C — experience from "push the can away, then grab the apple" transfers to "pick up the milk carton" because both keys encode *occlusion by a nearby object*.

The description is generated **once**, at the first step, not per step.

### Retrieval (line 2)

Cosine similarity between embedded keys, top-$k$:

$$\{(K_i,E_i)\}_{i=0}^{k-1} \leftarrow \arg\text{top-}k \; \frac{\mathcal{E}(K)^\top \mathcal{E}(K')}{\lVert\mathcal{E}(K)\rVert \lVert\mathcal{E}(K')\rVert}$$

Plain top-$k$ dense retrieval. The paper itself flags in §VI that this will not scale and suggests maximum marginal relevance (MMR) as future work.

### STM (lines 3, 8) — §IV-C

$m = \{(a_\tau, r_{\tau+1})\}_{\tau=0}^{t-1}$ — an episodic list of (action, feedback) pairs, injected into the planner prompt every iteration, **reset at task end**.

On failure, the planner performs self-reflection *as part of its chain-of-thought before emitting the action*. The ordering matters and is a real implementation detail: reasoning must be produced before the decision, which forces the model to reflect rather than rationalise. The paper's worked example:

> "Previous attempts to pick up the apple directly have failed. The apple is next to a cylindrical container, which might be causing interference. To create more space and ensure a successful grasp, I will push the can to the right, away from the apple."

### LTM (lines 11, and §IV-D)

Key–value store $\mathcal{M} \leftarrow \mathcal{M} \cup \{(K, \mathcal{S}(m))\}$. Key = scenario, value = distilled lesson. STM is transient; LTM is persistent and cross-task. This is the "lifelong learning" claim.

## 1.5 The skill layer and the annotation module (§IV-F)

Three skills: **pick**, **place**, **push**.

The paper's insight here is that geometry alone picks the wrong place to act. Purely geometric grasp planning will "seize an undesirable part (e.g. the meat on a skewer or the ice-cream top rather than its cone)." Placement and pushing likewise need semantics.

So they add an **on-demand image annotation module**, shared across all three skills:

1. VLM picks skill + parameters, and decides whether annotation is needed (a boolean it emits itself).
2. Open-vocabulary segmentation with **Grounded SAM** [30] (= GroundingDINO [31] for text→box, SAM [32] for box→mask) produces the object mask.
3. If annotation was requested, candidate locations are overlaid on the image as *numbered masks*, and the VLM chooses one.
   - **place** → candidates via **farthest-point sampling (FPS)** [33] on the mask
   - **push** → candidate goal-region masks
   - **pick** → FPS within the object mask

4. For grasping, the segmented point cloud goes to **AnyGrasp** [34], returning grasp hypotheses with confidences $s_{\text{conf}} \in [0,1]$. Kinematically infeasible ones are filtered by IK checks using **Pinocchio** [35], leaving feasible set $G$.

Final grasp selection maximises a product of two scores:

$$g^* = \arg\max_{g \in G} \; s_{\text{conf}}(g)\cdot s_{\text{loc}}(g)$$

where $s_{\text{loc}}(g) \in [0,1]$ is based on normalised Euclidean distance between the grasp and the VLM-chosen location. So: **geometry proposes, semantics disposes.**

> **Directly relevant to your build:** the paper uses **AnyGrasp + Pinocchio**. The released README instead recommends **Grounded SAM + GraspGen + Pinocchio**. Your stack uses **Grounded-SAM-2 + GraspGen + MoveIt 2**. The GraspGen substitution therefore follows the authors' own released guidance, not a deviation from it. See §7.1.

## 1.6 Experimental setup (§V-A)

- Platform: **ANYmal** quadruped [36] + 6-DoF arm, **Robotiq 2F-140** gripper, **ZED X Mini** mounted on the elbow (i.e. *eye-in-hand*, moving).
- VLM: `gpt-4o` [10]. Embeddings: `text-embedding-3-large` [37].
- **A human operator may reset the scene after a failure that alters the environment.**

That last point is a genuine human-in-the-loop caveat, and it is acknowledged in the code path: the node appends the note *"The human operator might have reset the scene after the action failure"* into STM (see §5.4).

## 1.7 Results

### Table II — effect of STM (each task 5–10 trials, **two attempts allowed**)

| Task | CaP-V | PragmaBot |
|---|---|---|
| Put apple on plate (container obstructs) | 43% | 86% |
| Move tiny candy (sponge/towel nearby) | 22% | 67% |
| Move egg (open view) | 40% | 100% |
| Pick up bowl (apple inside) | 33% | 83% |

Baseline **CaP-V** = Code as Policies [5] augmented with visual feedback, but no STM: it cannot reflect, so it repeats the same failure. Mean ≈ 35% → 84%, the abstract's headline.

### Table III — effect of LTM + RAG (single-trial, 12 scenarios, 8 unseen)

| Task | COME [14] | PragmaBot |
|---|---|---|
| Put apple on plate (container obstructs) | 29% | 100% |
| Move tiny candy (towel nearby) | 11% | 78% |
| Move egg (open view) | 20% | 100% |
| Pick up bowl (apple inside) | 17% | 83% |
| Put tennis ball in box (mug obstructs) | 29% | 71% |
| Put orange/ball on plate (fan blocks) | 10% | 80% |
| Move crumpled paper (brush nearby) | 25% | 63% |
| Move screw (towel nearby) | 0% | 86% |
| Move sushi (open view) | 14% | 71% |
| Move grape/cherry (open view) | 20% | 70% |
| Pick up box (apple on top) | 43% | 86% |
| Pick up towel (orange on top) | 50% | 75% |

Mean ≈ 22% → 80%. **Single-trial** is the important qualifier: this measures getting it right *first time*, which is what memory is supposed to buy.

The LTM contained **100 entries**: the 4 experiences generated in §V-B plus **96 "limited instructional experiences from simpler tasks."** It was **frozen** during evaluation for fair comparison.

### Success-detector reliability

On object-picking: **false negative 6.67% (4/60)**, **false positive 5% (3/60)**. Most false positives occur when the object is still on the table but *looks* enclosed by the gripper. The paper is explicit that the emergent adaptive behaviour "would not be possible without the high accuracy of the VLM's success detection" — the detector is load-bearing.

### Failure analysis (Figure 6) — 19 first-failures out of 91 trials

```
All trials (91)
├── Success (72)
└── Failure (19)
    ├── Execution (8)   ── Poor grasp generation (5)
    │                   └─ Inaccurate depth value (3)
    ├── VLM (7)         ── Overrides experience (5)
    │                   ├─ False detection (1)
    │                   └─ Wrong mask selection (1)
    └── RAG (4)         ── Retrieval failed (4)
```

**The single largest failure bucket is execution (8/19), and all of it is perception/grasping.** Note this well — it is exactly the layer you are building, and it is where the paper is weakest.

The most interesting failure mode is *"VLM overrides experience"* (5 cases): the model retrieved the right memory and then decided the current scene didn't match closely enough, ignoring it. E.g. in "put tennis ball in box (mug obstructs)," it judged the mug's obstruction negligible. This is a genuine open problem — the model has no calibrated notion of how much to trust retrieved experience.

### Retrieval ablation (§V-D, Figure 7) — accuracy of the *first planned action*, no execution

| Setting | Accuracy |
|---|---|
| Random $k=5$ | 17% |
| Entire LTM in context | 74% |
| **RAG top-$k$** | **89%** |

Two findings worth carrying: feeding the whole memory is *worse* than retrieving selectively (consistent with long-context degradation [38],[39]), and it inflates prompt length by **7.5×** excluding images, with matching cost. `gpt-4o-mini` was also tested — it behaves more conservatively, which *helps* when it has no relevant experience, and gains less from memory than the larger model.

### Annotation ablation (§V-E, Figure 8)

Tested across 7 objects; success = grasped the *correct section* (e.g. the stick of a skewer). Annotation helps most on complex shapes (drumstick, skewer); without it AnyGrasp "often favors larger surfaces due to its reliance on geometric cues." Remaining failures with annotation are mostly **inaccurate 3D point clouds**. Pushing distance-error also improves. Cost: **+5.15 s** average latency per `gpt-4o` call.

## 1.8 Limitations the authors state (§VI)

1. Vision-only; no tactile or audio.
2. Memory database limited by cost of real-world data collection. Scaling raises: should memory be pruned? Which memories forgotten? Can the VLM filter without human heuristics?
3. Top-$k$ retrieval will become ineffective as memory grows → MMR suggested.
4. Memory sharing across robots: feasible with identical hardware, open with differing morphology.

## 1.9 Critical reading — what a reviewer would press on

These are worth knowing because your supervisor may raise them, and because several are things *your* work can actually address.

1. **Small samples, no statistics.** 5–10 trials per task, no confidence intervals, no significance tests. A jump from 0% to 86% on "move screw" rests on ≤10 trials. Directionally convincing, not statistically established.
2. **The LTM is 96% hand-seeded.** Only 4 of 100 entries came from autonomous experience; the rest are "limited instructional experiences." The "learning from experience" claim is real but the *demonstrated* memory is mostly curated. The paper is honest about this, but it is easy to over-read the headline.
3. **Human scene resets.** A human may reset the environment after a destructive failure. Success rates are therefore not fully autonomous-episode rates.
4. **Table II allows two attempts**; Table III is single-trial. Different protocols; don't conflate the 84% and 80% numbers.
5. **Baselines are re-implementations.** CaP-V is CaP *plus* visual feedback, built by these authors; COME likewise as compared. Reasonable, but not authors' own tuned systems.
6. **The success detector grades its own planner.** Same model, same weights. Shared blind spots are plausible — a 5% false-positive rate is measured on picking only.
7. **Generalisation is within-distribution.** "Unseen" scenarios are structurally similar tabletop rearrangements ("structurally similar scenarios" is the paper's own wording).
8. **Execution quality is a confound.** With 8/19 first-failures from grasping and depth, a better manipulation stack would raise the numbers without any change to the memory contribution. *Your* pipeline is measurably better on exactly this axis (§8.2) — which is a legitimate, defensible contribution.

---

# PART II — THE COURSE PROJECT

Source: `D:/pb/home/extras/introduction_to_sose_irobman_lab_2026.pdf` (21 pages, parsed to `C:/Users/haris/claude-papers/papers/irobman_lab/paper.txt`).

**Praktikum zur intelligenten Robotermanipulation: Part II — 20-00-1170-pr**, TU Darmstadt (PEARL Lab / iRobMan), Summer Semester 2026.

**This is Project 3 — "Memory representations for Robotic Task Planning."** Tagged **Real World**. Supervisor: **Vignesh Prasad** (`vignesh.prasad@tu-darmstadt.de`).

The project description, verbatim in substance:
- Goals: **study and re-implement the method on our robot**; explore extensions for **speeding up the system with local VLMs** and **incorporating other memory representations such as ontology-based experiences**.
- Requirements: Python (advanced), Robotics (basic ROS), LLMs (practical, not mandatory), Enthusiasm (advanced).

### Hard dates and grading

| Item | Date |
|---|---|
| Project window | 2026-04-24 → 2026-09-30 |
| **Report** (scientific, *blogpost format*) | **Mon 2026-09-14, 23:59 CET** |
| **Presentation** (15 min, in class) | Mon 2026-09-21 & Tue 2026-09-22 |

**Grading: 30% supervisor · 40% report · 30% presentation.**

As of 2026-08-18 that leaves **27 days to the report** and ~34 to the presentation.

Two consequences that should drive every planning decision:

1. **70% of the grade is report + presentation, and only 30% is supervisor assessment of the work itself.** Communication is weighted more heavily than completion. A well-analysed partial system beats an undocumented complete one.
2. The course's stated expected outcome is explicitly about *critical thinking* — "you learn what it takes to fully understand a paper and reproduce its results" — not about matching the paper's numbers. Documenting *why* something couldn't be reproduced counts as a result.

Supervision cadence: roughly a meeting every two weeks, or feedback by message.

---

# PART III — THE SYSTEM AS BUILT

## 3.1 Hardware and machines

| Thing | Detail |
|---|---|
| Robot | **Franka FR3**, named "Athna", arm ID `10070378`, firmware **5.9.0**, IP `10.10.10.10`, **Franka Hand** gripper |
| Camera | **ZED2**, statically mounted facing the table (*not* eye-in-hand, unlike the paper) |
| Lab machine | **Alonnisos** — shared multi-user box, RTX 4080 (Ada, compute capability **8.9**), driver 535.183.01 |
| Home machine | this laptop; own NVIDIA GPU; **no VPN to the lab network** |

The FR3 (not Panda) identity was a correction that mattered — firmware 5.9.0 requires libfranka ≥ 0.15, which is why older Panda-era ROS 1 / MoveIt paths were dead ends.

**Robot access requires** unlocking the robot and **activating FCI in Desk** (`https://10.10.10.10/desk/`) before launching MoveIt, or `ros2_control_node` aborts with `libfranka: Connection to FCI refused`.

## 3.2 The six-folder workspace

`D:/pb/irm2.code-workspace` defines a multi-root VS Code workspace over:

| Folder | What it is |
|---|---|
| `home/ros2_ws/franka_ros2` | FR3 control stack + **your** `pragmabot_bridge`, `pragmabot_interfaces`, vendored `easy_handeye2` |
| `home/zed_ros2_ws` | ZED wrapper workspace (+ vendored `easy_handeye2-master`, `ros2-aruco-detector-main`) |
| `home/GraspGen` | NVIDIA GraspGen, with your local FR3 additions |
| `home/groundedsam` | Grounded-SAM-2 (IDEA-Research), effectively upstream |
| `home/pragmabot` | The PragmaBot fork — planner, memory, calibration pipeline, notes |
| `home/extras` | Paper PDFs, course PDF, session notes, setup guide |

Also on disk but **not** in the workspace file: `home/foundationpose/` — **confirmed completely empty**, zero files. FoundationPose was investigated and abandoned; no weights were ever downloaded, on Alonnisos or here.

## 3.3 Environment isolation — the design decision

The architecture is **host-only with per-tool virtual environments**, *not* multi-container. Only robot control is containerised.

The reasoning, which is sound and worth being able to state: isolation was always needed at the *Python environment* level, never at the OS level. The three GPU tools have mutually incompatible dependency sets:

| Tool | Python | torch | Notes |
|---|---|---|---|
| GraspGen | 3.10 | `2.1.0` / tv `0.16.0`, cu121 | own `uv` venv |
| Grounded-SAM-2 | 3.10 | `>=2.3.1` / tv `>=0.18.1`, cu121 | **separate** venv — cannot share with GraspGen |
| FoundationPose | 3.11 env created, nothing installed | builds PyTorch3D + NVDiffRast from source | abandoned |

`uv venv` gives that isolation directly. Containers would have added a boundary whose only effect was to *create* bugs — an SHM/DDS root-vs-non-root problem that cost a full day existed **only** because of a container boundary. Dropping containers 2 and 3 removed the entire bug class rather than patching a symptom. That is a genuinely good engineering judgement and is defensible in interview.

**Always** `export TORCH_CUDA_ARCH_LIST="8.9"` (Alonnisos value; substitute your own GPU's) before compiling any custom CUDA op — `pointnet2_ops`, `ms_deform_attn`, PyTorch3D, NVDiffRast.

## 3.4 Containers actually present

The single operational container is **`franka_ros2_humble`**, defined at `D:/pb/home/ros2_ws/franka_ros2/docker-compose.yml`:

- `network_mode: host`, `ipc: host`, `privileged: true` (lines 10–12)
- `ROS_DOMAIN_ID=7` (line 23)
- realtime: `cap_add: SYS_NICE`, `rtprio: 99`, `memlock: 8428281856` (lines 26–31)
- **No GPU reservation** — it is pure control, no vision.
- Mounts (lines 17–21): `./:/ros2_ws/src`, `/tmp/.X11-unix`, `./limits.conf`, `/dev`, and `/home/harish/pragmabot:/pragmabot`

> **Reconciling an apparent contradiction.** Your session notes say `~/pragmabot` is *not* bind-mounted and that `rsync` is required. `docker-compose.yml:21` *does* mount it — at `/pragmabot`. Both are correct: colcon builds `/ros2_ws/src` (line 17), and `/pragmabot` is **outside the build path**. So the code is visible inside the container but not buildable in place, and the rsync-into-`franka_ros2/` step remains necessary. **[VERIFIED HERE]**

Other Dockerfiles exist (GraspGen serve image, Grounded-SAM-2 demo front/back-end, libfranka CI, `franka_description` on ROS Jazzy, ZED wrapper images) but are not part of the live workflow — ZED runs host-native.

Enter and launch:
```bash
docker exec -it -e DISPLAY=$DISPLAY franka_ros2_humble bash
source /ros2_ws/install/setup.bash          # overlay is NOT sourced in a fresh shell
ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=10.10.10.10
```
If `install/` is missing entirely, the container was **recreated** rather than restarted → `colcon build --symlink-install` again.

## 3.5 Live control interfaces (confirmed working on real hardware)

`/move_action` · `/execute_trajectory` · `/compute_cartesian_path` · `/compute_ik` · `/franka_gripper/grasp` · `/franka_gripper/homing` · `/franka_gripper/move`

**Never** `/fr3_gripper/gripper_action` — dead stub, silently hangs.

## 3.6 ZED configuration

Host-native launch, **`ROS_DOMAIN_ID=7` required on every terminal** or topics are invisible:
```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2
```

Two deliberate local changes to the wrapper config (both defensible, both worth being able to explain):

1. **Positional tracking and TF publishing disabled**; raw IMU enabled. `pos_tracking_enabled`, `imu_fusion`, `publish_tf`, `publish_map_tf` → `false`; `publish_imu_raw`, `publish_cam_imu_transf` → `true`. Rationale: the FR3 setup owns its own TF tree (`robot_state_publisher` + external hand-eye calibration). Letting the ZED publish odometry-derived TF would fight the calibrated transform. **This is the right call** — two sources publishing into one tree is a classic, hard-to-debug failure.
2. **`depth_mode: NEURAL → NEURAL_PLUS`** — better close-range depth accuracy, motivated by a ~6 cm calibration cube.

Key topics (`/zed/zed_node/` root):
- `rgb/image_rect_color` (+ `/camera_info`)
- `depth/depth_registered`
- `point_cloud/cloud_registered` — `PointCloud2` XYZRGB, ~9 Hz, frame `zed_left_camera_frame`

**Frame naming trap:** the optical frame is `<camera_name>_left_camera_frame_optical`. The older `_left_camera_optical_frame` ordering is deprecated and wrong.

> **Duplicate-repo hazard.** There are **two copies** of `zed-ros2-wrapper`: the live one in `zed_ros2_ws/src/` (carrying the two config changes above) and a second nested inside `ros2_ws/franka_ros2/` at an *earlier* state without them. Only the `zed_ros2_ws` copy is authoritative. Editing the wrong one is a silent failure waiting to happen.

---

# PART IV — THE PIPELINE, COMPONENT BY COMPONENT

```
                    ┌──────────────── PragmaBot brain (ROS 1, upstream) ────────────────┐
   instruction ────▶│ SceneDescriber → MemoryManager(RAG) → TaskPlanner                 │
                    │        ▲                                      │                   │
                    │        │                                      ▼                   │
                    │  SuccessDetector ◀── STM ◀───────── NextBestAction(skill,target)  │
                    │        │                                      │                   │
                    │  ExperienceSummarizer ─▶ LTM                  │                   │
                    └───────────────────────────────────────────────┼───────────────────┘
                                                                    │
                                              ══════════ THE GAP ═══╪══════════
                                                                    ▼
                    ┌─────────────── body (built for FR3, ROS 2) ───────────────────────┐
   ZED2 RGB ───────▶│ detect_object.py (Grounded-SAM-2)  →  mask.npy                    │
   ZED2 depth ─────▶│ mask_to_pointcloud.py  →  object_pcd.npy  (erode + MAD)           │
                    │ graspgen_client.py (ZMQ:5556)  →  grasps.npz                      │
                    │ grasp_transform.py  →  un-center, TF to fr3_link0, pick top-down   │
                    │ bridge_node.execute_pick()  →  MoveIt 2 + franka_gripper           │
                    └───────────────────────────────────────────────────────────────────┘
```

## 4.1 Perception — `detect_object.py`

`D:/pb/home/pragmabot/calibration/detect_object.py` (192 lines). Text-prompted 2D detection/segmentation wrapper over Grounded-SAM-2.

```bash
~/groundedsam/.venv/bin/python calibration/detect_object.py --rgb <rgb.png> --prompt "cube."
```

Outputs to `<rgb-dir>/detections/`: `mask.npy` (bool, H×W), `annotated.jpg`, `detections.json` (class, bbox, confidence, RLE mask).

- Prompt follows GroundingDINO convention: **lowercase, each phrase ends in a period**; multiple phrases allowed (`"cube. red button."`).
- `--select best` (default) keeps only the top-confidence match; `--select all` unions everything above threshold.
- `--box-threshold` / `--text-threshold` default `0.35` / `0.25`.

**Non-obvious required hack, documented in the project notes:** the module must `sys.path.insert(0, GROUNDED_SAM2_ROOT)` before importing `groundingdino`, because the repo's `inference.py` self-imports as `grounding_dino.groundingdino...` (repo-root-relative) rather than by installed package name.

Validated behaviour: on a real capture with prompt `"cube."` it correctly chose the cube (conf 0.57) over a false positive on the e-stop button (conf 0.48).

## 4.2 Mask + depth → object point cloud — `mask_to_pointcloud.py`

`D:/pb/home/pragmabot/calibration/mask_to_pointcloud.py` (326 lines). **This is the most original piece of engineering in the project.**

Standard pinhole back-projection of masked pixels:
$$x=\frac{(u-c_x)z}{f_x},\qquad y=\frac{(v-c_y)z}{f_y},\qquad z=\text{depth}$$
then invalid/out-of-range depth filtering and downsample to `--target-points` (default 2000).

### The flying-pixel problem and its fix

**Symptom:** the point cloud didn't look like the object it came from — a 16.7 cm z-spread for a cube whose true depth extent is ~4–5 cm.

**Diagnosis** (the part that matters): comparing eroded-interior pixels against the boundary ring of the *same* mask showed interior z-std of 1.7 cm — a real, compact cube — while boundary pixels were ~4× more likely to be a >4 cm far-outlier (41% vs 10%). So the **2D mask was fine**; the *depth values at silhouette pixels* were wrong. This is the classic stereo artefact: at a depth discontinuity the matcher interpolates between near object and far background, producing a comet-tail of points trailing off the edge.

**Fix — two filters, both on by default:**

| Flag | Default | What it does | Why this design |
|---|---|---|---|
| `--erode-px` | 3 | shrinks mask inward before back-projecting | drops exactly the boundary ring where the noise lives; pure-numpy cross-kernel erosion (`erode_mask`, line 67) — **no cv2/scipy**, because GraspGen's venv has neither |
| `--z-outlier-mad` | 3.5 | drops points beyond N scaled-MAD from median depth | **MAD, not mean/std**: the artefact is *one-sided*, so it drags a mean/std filter along with it. Uses the normal-consistent `1.4826·MAD` scaling (line 89) |

A third filter (`aabb_crop`, line 113) bounds x/y by robust percentile range with fractional padding. Note the deliberate separation of concerns documented in the header: **the MAD filter is depth-only; the AABB crop is the only filter acting on x and y.**

**Documented result on the cube scene:** 1947 → 1449 points, z-extent 16.7 cm → 4.5 cm, and GraspGen confidence rose **0.77–0.95 → 0.94–0.98**. The cleaner input measurably improved the model's own confidence — not merely a prettier picture.

**[VERIFIED HERE]** I independently re-ran this logic on the `red_cup` scene from the raw `depth.npy` + `mask.npy`:

| Stage | Points | z-extent |
|---|---|---|
| raw back-projection | 6334 | **17.7 cm** |
| + 3 px erosion | 5786 | **7.8 cm** |
| + z-MAD 3.5 | 5786 | 7.8 cm |

and the root-cause split reproduced: **boundary far-outlier rate 11.9% vs interior 1.5%** (~8×). Different scene from the cube (which logged 41%/10%), same mechanism, same direction. The effect is real and replicable.

> **Why this matters scientifically:** the paper's own failure analysis lists *"inaccurate depth value"* (3/19) and *"poor grasp generation"* (5/19) as the largest failure bucket, and §V-E notes annotation failures are "mostly due to inaccurate 3D point clouds." This fix attacks the paper's single biggest measured weakness. That is a real contribution, not a workaround.

## 4.3 Grasp synthesis — GraspGen

Runs in its own venv as a **ZMQ server**:
```bash
source ~/GraspGen/.venv/bin/activate
python3 ~/GraspGen/client-server/graspgen_server.py \
    --gripper_config ~/GraspGen/GraspGenModels/checkpoints/graspgen_franka_panda.yml
python3 ~/GraspGen/client-server/graspgen_client.py --pcd_file <object_pcd.npy> --save_grasps <grasps.npz>
```
Protocol: msgpack over a ZMQ REP socket, default port **5556**. Actions: `health`, `metadata`, `infer`.

- Gripper config: `graspgen_franka_panda.yml` — correct for the Franka Hand. (Robotiq/suction checkpoints were removed as unneeded; the Robotiq generator checkpoint doesn't exist upstream anyway.)
- Server load time ~5 s.

**Grasp frame convention** (`docs/GRIPPER_DESCRIPTION.md`): approach axis is the grasp frame's **+Z**, finger-closing axis is **+X**, and the **origin is the gripper base/root link — not the fingertip/TCP**. Getting this wrong is a silent, expensive error.

**Scale requirement — a hard-won lesson.** The model expects an **object-scale** cloud (~2000 points, roughly object-sized extent), *not* a raw scene capture:
- full unfiltered ZED frame (~100k points spanning metres) → CUDA OOM (attempted 40 GiB allocation)
- depth-cropped but still room-scale (~4000 points over ~1 m) → crashes the discriminator's outlier-removal (`knn` distance matrix on a non-compact cluster removes every point → reshape error)

The GroundedSAM-driven segmentation superseded an earlier hand-picked ~30 cm crop and produces exactly the right scale automatically.

### Your local additions to GraspGen

Verified present on disk:
- **`--save_grasps <path.npz>`** (`client-server/graspgen_client.py:99-107`, writing at `:218-219`) — persists `grasps`, `confidences`, `centroid`. The help text itself documents the centroid contract: *"centroid is the mean subtracted from the input cloud before sending to the server — consumers must add it back to map returned poses into the original (uncentered) input frame."*
- `grasp_filter.py` (repo root) and `scripts/capture_zed_frame.py` — FR3-pipeline glue, not upstream NVIDIA files.

`capture_zed_frame.py` is a one-shot rclpy subscriber saving `xyz` as `.npy` in the format `--pcd_file` already accepts.

> **Broken upstream reference, worth knowing:** `docker/serve.dockerfile` and `docker/run_server.sh` both point at `tools/graspgen_server.py`, which does not exist — the directory is `client-server/`. Stale path from an upstream refactor; the documented Docker serve path is broken as written. Not your bug, and it doesn't affect you because you run the server natively.

## 4.4 Grasp → robot motion — `grasp_transform.py`

`D:/pb/home/ros2_ws/franka_ros2/pragmabot_bridge/pragmabot_bridge/grasp_transform.py` (160 lines). Pure math + TF, no ROS actions. Six functions:

**`load_all_grasps(npz_path)` — `:27-41`.** Loads the `.npz` and **adds the centroid back**: `grasps[:, :3, 3] += data["centroid"]` (`:40`). Without this every grasp is silently offset by the object's own position, because the client mean-centres the cloud before sending. This is the exact counterpart to the `--save_grasps` contract.

**`select_topdown_index(grasps_T_base)` — `:49-59`.** Computes each grasp's approach axis as $R\cdot[0,0,1]^\top$ and returns `argmin` of its z-component — i.e. the most downward-pointing approach.

> The docstring makes the key point explicitly: the input **must already be in a gravity-aligned frame** (`fr3_link0`), because *"top-down isn't a meaningful comparison in camera frame, since the camera's own tilt is arbitrary."* `bridge_node.py:194-195` honours this by selecting **after** the TF transform. This is more correct than filtering by approach angle in camera frame.

**`standoff_pose(grasp, offset_m)` — `:62-70`.** Same orientation, translated **back** along the grasp's own +Z approach axis. Frame-agnostic despite the parameter name.

**`transform_to_matrix(TransformStamped)` — `:73-83`** and **`to_robot_frame(...)` — `:86-95`.** Quaternion → 4×4 via `scipy.spatial.transform.Rotation`.

**`estimate_gripper_width(pcd_cam, grasp_T_cam, gripper_depth=0.10527314, percentile=5.0)` — `:98-148`.** Transforms the cloud into the grasp's local frame, keeps points in the fingertip contact band (local $z \in [\text{depth}/2, \text{depth}]$), and takes a **robust percentile spread along local X** (the finger-closing axis) as the width.

The reasoning is genuinely good and stated in the docstring (`:104-130`): a non-uniform object — a cup — is a very different width at the rim than at the body, and *the grasp pose determines which one the fingers actually land on*, so a single hand-measured "object diameter" can be wrong. `0.10527314` is `franka_panda.yaml`'s own `depth` value for this exact gripper config. The function raises rather than guesses if fewer than 10 points fall in the band (`:138-144`), and the docstring is careful to flag that it relies on a single-view silhouette-width assumption and should not be trusted to the millimetre.

**`matrix_to_pose(T)` — `:151-160`.** 4×4 → `geometry_msgs/Pose`.

## 4.5 Execution — `bridge_node.py`

`D:/pb/home/ros2_ws/franka_ros2/pragmabot_bridge/pragmabot_bridge/bridge_node.py` (511 lines). ROS 2 node `pragmabot_bridge`.

Clients created at `:70-77`: `MoveGroup` on `/move_action`, `ExecuteTrajectory`, `Grasp`/`Homing`/`Move` on `/franka_gripper/*`, and a service client for `/compute_cartesian_path`. TF buffer/listener at `:79-80`. Sixteen declared parameters at `:82-97`.

### `execute_pick()` — `:111-313`, the seven steps

| # | Step | Lines |
|---|---|---|
| 0 | Home the gripper first (unless disabled) | `:173-179` |
| 1 | Load all grasps (centroid re-added) | `:181` |
| 2 | One TF lookup `fr3_link0 ← camera_frame`, transform **all** grasps at once | `:184-192` |
| 3 | Auto-select most top-down grasp in robot frame, or use explicit index | `:194-205` |
| 4 | Build standoff; auto-estimate gripper width if not given | `:209-223` |
| 5 | `MoveGroup` to standoff → Cartesian approach → `Grasp` → Cartesian lift | `:228-279` |
| 6 | Optional place-back-in-place after `place_after_s` | `:281-311` |

Three engineering decisions inside this worth calling out:

**Gripper homing.** The Franka Hand driver needs a `Homing` call after connecting or after *any* `Grasp`/`Move` fault, or it goes unresponsive and Desk reports "End Effector: Not connected." The node homes up front (`:173`) and **re-homes and retries once** if a grasp fails (`:256-262`), rather than leaving the gripper stuck for the next attempt. The docstring (`:15-19`) explicitly records that this is *not* a network/Docker issue — that diagnosis is the valuable part.

**Singularity guard.** `GetCartesianPath.revolute_jump_threshold` (default 0.2 rad ≈ 11° per 1 cm Cartesian step, `:404-412`) truncates the path if a single step demands an unusually large joint change — the fingerprint of a near-singular IK solution. The node **aborts on `fraction < 1.0` rather than executing a truncated path** (`:237-247`), and the error message teaches the reader how to interpret it. The docstring is honest that 0.2 is "a conservative starting point, not empirically tuned for this robot/workspace."

**Lift along base +Z, not the grasp's own −Z** (`:264-268`): retracing the approach axis risks re-colliding with the table; a vertical lift in `fr3_link0` doesn't.

**Not yet implemented:** `execute_place()` `:315-316` and `execute_push()` `:318-319` both `raise NotImplementedError`.

**How it currently runs:** `main()` at `:476-507` reads the `grasp_file` parameter and, if set, runs **one** `execute_pick()` then spins. There is **no action server, no service, no topic** by which the planner could command it. This is the gap.

### Honest caveats already flagged in the code

The `eef_link` default `fr3_hand` is **not verified live** — the docstring (`:140-145`) states plainly that it must match whatever link GraspGen's gripper-base convention corresponds to in this MoveIt setup, and that some `franka_ros2` setups use `fr3_hand_tcp` instead. **This is the highest-risk unverified assumption in the execution path** (§8.1).

## 4.6 `pragmabot_interfaces` — the unbuilt bridge contract

`D:/pb/home/ros2_ws/franka_ros2/pragmabot_interfaces/` contains only `package.xml` and `CMakeLists.txt`. **There is no `action/` directory and no interface is generated** — `rosidl_generate_interfaces` is commented out at `CMakeLists.txt:16-22`, with the open design decision recorded in the comment:

> one collapsed `ExecuteSkill.action`, or `PandaPick`/`PandaPlace`/`PandaPush` kept separate (ported from the ROS 1 `.action` files).

The package declares the right dependencies (`rosidl_default_generators`, `geometry_msgs`, `std_msgs`, `rosidl_interface_packages` membership) — it is a correct skeleton awaiting one decision.

The ROS 1 originals do exist to port from: `D:/pb/home/pragmabot/pragmabot/action/{PandaPick,PandaPlace,PandaPush}.action`.

## 4.7 Calibration — two independent methods

**Method A — `easy_handeye2`, ArUco, eye-on-base. This is the one that was actually used.**

```bash
# marker detection
ros2 run aruco_detector aruco_detector --ros-args \
  --remap image:=/zed/zed_node/rgb/color/rect/image \
  --remap camera_info:=/zed/zed_node/rgb/color/rect/image/camera_info \
  -p marker_size:='0.05' -p image_is_rectified:=true

# calibration GUI
ros2 launch easy_handeye2 calibrate.launch.py name:=fr3_zed_right \
  calibration_type:='eye_on_base' \
  tracking_base_frame:='zed_camera_link' tracking_marker_frame:='marker_0' \
  robot_base_frame:='fr3_link0' robot_effector_frame:='fr3_hand'

# publish the result, then verify
ros2 launch easy_handeye2 publish.launch.py name:=fr3_zed_right
ros2 run tf2_ros tf2_echo fr3_link0 zed_camera_link
```
Jogging the arm for sampling uses the gravity-compensation controller:
`ros2 launch franka_bringup example.launch.py controller_names:=gravity_compensation_example_controller`

> **`easy_handeye2` is vendored upstream, unmodified.** **[VERIFIED HERE]** a recursive search for `fr3|franka|zed` across all its `.py`/`.xml`/`.yaml`/`.md` returns **zero** hits — it is configured entirely through launch arguments. Claim this as *integration and calibration work*, never as authorship.

**Method B — FoundationPose on a marked cube (designed, not completed).** Direct composition rather than AX=XB:
$$T_{\text{cam}}^{\text{base}} = T_{\text{cube}}^{\text{base}} \cdot \left(T_{\text{cube}}^{\text{cam}}\right)^{-1}$$
Tooling exists (`mesh_gen.py`, `calibrate_extrinsic.py`, `capture_calib_frame.py`, `make_object_pointcloud.py`, `T_cube_in_camera.txt`), but the FoundationPose `register()` call is a flagged placeholder and no weights were ever downloaded.

Two facts from this line of work are worth keeping regardless:
- **Cube symmetry blocker:** a plain-coloured cube has 24 rotational symmetries. FoundationPose returns *a* valid pose, not an error — possibly the wrong symmetric equivalent. One face must be uniquely marked before pose estimation is usable.
- **IMU accelerometer at rest gives orientation only** (roll/pitch from gravity) — never translation, never yaw.

## 4.8 The brain (upstream, ROS 1) and how it is wired

`D:/pb/home/pragmabot/pragmabot/`. The upstream VLM/memory modules are treated as **read-only**; the only permitted edits are importing/instantiating the executor and replacing the `NotImplementedError` sites.

Untouched upstream: `vlm_client.py`, `vlm_task_planner.py`, `vlm_scene_describer.py`, `vlm_success_detector.py`, `vlm_exp_summarizer.py`, `memory_manager.py`, `scene_observer.py`, `conversation_builder.py`, `utils.py`, `simple_config.py`.

### The main loop — `nodes/pragmabot_node.py`

- `self.executor = PandaSkillExecutor()` — `:91`
- `handle_planning_request` — `:99`; captures observation `:116`; scene description once at first step; LTM retrieval if empty; `plan_action(...)` `:135`
- Branch at `:139`: **if `rosbag_replay` → skip execution entirely** and go straight to evaluation `:141`; else `exec_result = self.executor.execute(next_action)` `:143`
- `handle_evaluation_request` — `:148`; re-captures observation `:165`; `perform_success_detection(instruction, action, before, after)` `:167`
- On completion → `handle_experience_summarization` `:187`; otherwise loops back to `handle_planning_request` `:185`

> **A correction to an easy misreading.** `PandaSkillExecutor.execute()` returns `pre_image: None, post_image: None`. That is **not** a bug: `handle_evaluation_request` captures its own before/after frames from `SceneObserver` (`:158`, `:165`), so the executor's image fields are simply unused. I checked this specifically before claiming otherwise.

### `PandaSkillExecutor` — ROS 1, and why it doesn't help you

`src/pragmabot/panda_skill_executor.py` (93 lines): `import rospy`, `import actionlib`, three `SimpleActionClient`s on `/panda/pick`, `/panda/place`, `/panda/push` (`:15-17`), 120 s timeouts. It dispatches on `action.skill.lower()` (`:30-43`) and handles `"done"` as a no-op success.

**This is ROS 1 actionlib talking to ROS 1 action servers.** The ROS 1 servers (`nodes/panda_pick_server.py` etc.) were written for the earlier Panda-era plan, contain placeholder constants, and were never run against the FR3. They are *not* the FR3 execution path — `pragmabot_bridge` is.

### VLM backends

Three interchangeable backends selected by substring match on `config.yaml`'s `vlm.vlm_model`:

| Backend | Structured output | Embeddings |
|---|---|---|
| OpenAI (upstream) | `chat.completions.parse()` | `text-embedding-3-large`, 3072-D |
| Claude (added) | `messages.parse()` | **no native embedding API** → local `sentence-transformers` `all-MiniLM-L6-v2`, 384-D |
| Gemini (added) | `GenerativeModel` + JSON schema | `text-embedding-004`, 768-D |

Per-backend gotchas that cost real time:
- Claude's system prompt is a **top-level API parameter**, not a `messages` entry; `messages.parse()` requires `max_tokens` explicitly.
- Gemini's assistant role is `"model"`, not `"assistant"`.
- Image payloads differ three ways: OpenAI `image_url` (base64 data-URI) → Claude `{"type":"image","source":{"type":"base64",...}}` → Gemini `inline_data` (raw base64, no prefix).

> **Load-bearing consequence:** the three backends produce **different embedding dimensions**, so **LTM CSVs are not portable across backends.** Switching requires a fresh LTM or separate CSV sets per backend. This is a genuine experimental-design constraint — any comparison across backends must control for it.

### Memory storage

- **STM** — per-episode JSON list of `{time_step, action}` / `{time_step, evaluation}`, injected into the planner prompt each loop.
- **LTM** — two CSVs merged on a `scenario` key: `ltm.csv` (human-readable) + `ltm_<model>.csv` (base64 embeddings). Scenario key = `"Instruction: {task}\nScene: {initial_scene_description}"`. Retrieval = cosine similarity, top-$k$.
- Live file: `pragmabot/data/ltm/ltm.csv`.

### Config — `pragmabot/config/config.yaml`

```yaml
rosbag_replay: true        # skip real execution, go straight to success detection
activate_stm: true
activate_ltm: true
save_to_ltm: true
use_random_retrieval: false
retrieval_top_k: 5         # -1 = everything, unsorted
vlm:
  vlm_model: claude-opus-4-8
  text_embedding_model: text-embedding-3-large   # only used by the OpenAI path
topics:
  color_image: /zedxm/zed_node/rgb/image_rect_color/compressed
  depth_image: /zedxm/zed_node/depth/depth_registered
  camera_info: /zedxm/zed_node/rgb/camera_info
```

> **Stale topic names.** These are `/zedxm/...` (the upstream ANYmal ZED X Mini). Your camera publishes under `/zed/zed_node/...`. **These must be changed before the planner can see your camera.** Cheap fix, easy to forget, blocks everything.

`rosbag_replay: true` is the primary offline development mode: `handle_planning_request` skips the executor entirely and calls the success detector directly — meaning **the full brain can be exercised at home with no robot.**

---

# PART V — DATA ARTIFACTS ON THIS MACHINE

Two captured scenes plus one raw rosbag are present locally. This is what makes meaningful offline work possible.

```
D:/pb/home/pragmabot/
├── bags/red_cup/red_cup_0.db3          raw ROS 2 bag + metadata.yaml
├── bags/extract_bag.py                 bag → rgb.png / depth.npy / intrinsics.json
└── extracted/
    ├── red_cup/   rgb.png, depth.npy, intrinsics.json, grasps.npz,
    │              detections/{mask.npy, annotated.jpg, detections.json, object_pcd.npy}
    └── cup/cup/   rgb.png, depth.npy, intrinsics.json,
                   detections/{mask.npy, annotated.jpg, detections.json, object_pcd_cup.npy}
```

**[VERIFIED HERE]** contents:

| Item | `red_cup` | `cup/cup` |
|---|---|---|
| intrinsics | fx=fy=**528.604**, cx=635.405, cy=363.729, 1280×720 | fx=fy=**521.536**, cx=630.688, cy=368.193, 1280×720 |
| frame_id | `zed_left_camera_frame_optical` | `zed_left_camera_frame_optical` |
| depth | (720,1280) float32, 94.0% finite, 0.416–2.960 m | (720,1280) float32, 88.9% finite, 0.218–2.308 m |
| mask | 6,746 true px | 13,609 true px |
| object cloud | (2000,3), extent 9.5×10.7×7.3 cm | (5000,3), extent 11.6×8.1×9.2 cm |
| grasps | (6,4,4), conf 0.924–0.956 | — |

Note the intrinsics differ between the two captures — **real per-session calibration, not the old `fx=fy=700` placeholder.** The placeholder is gone.

### The duplicated pair at the repo root

`D:/pb/home/ros2_ws/franka_ros2/grasps.npz` and `object_pcd.npy` are the pick-execution test inputs.
**[VERIFIED HERE]** `object_pcd.npy` is **byte-identical** to `extracted/red_cup/detections/object_pcd.npy`; `grasps.npz` is **not** identical to `extracted/red_cup/grasps.npz` (different confidences: 0.925–0.941 vs 0.924–0.956) — a separate GraspGen run on the same cloud. GraspGen is stochastic, so this is expected, not a discrepancy.

### Verification of the artifact contract **[VERIFIED HERE]**

Everything `grasp_transform.load_all_grasps` assumes was checked against the real files:

- `grasps.npz` keys are exactly `grasps (6,4,4) float32`, `confidences (6,) float32`, `centroid (3,) float32` — matching `:38-41`.
- Saved `centroid` = `[0.04458276, 0.05925157, 0.63461024]` equals the point cloud's own mean to ~1e-6. **The un-centering contract is correct and the two files are a consistent pair.**
- All six rotation matrices are valid: `det = 1.000000`, orthogonality error ≤ 2.1e-7, no NaN/Inf.
- **Gripper-base convention sanity check:** taking each grasp's origin and advancing 0.75 × `gripper_depth` along its +Z approach axis lands the fingertip band **on the object** in all six cases (0.058–0.072 m from the cloud centroid, inside the bounding box + 2 cm). This confirms the base-link-not-fingertip convention is being handled correctly — grasp origins sitting *outside* the cloud is expected, not a bug.
- `estimate_gripper_width` succeeds for all six grasps (180–375 points in the fingertip band, well above the 10-point floor), returning **12.1 / 18.2 / 18.8 / 21.3 / 33.1 / 33.2 mm**.

> That last spread is the empirical justification for the whole function: on one object, at six different valid grasps, the correct closing width varies by nearly **3×**. A single hand-measured "object width" would be wrong for most of them.

---

# PART VI — VERIFIED / UNVERIFIED LEDGER

## Verified on real hardware (per project records)
- ZED2 live feed, RViz2-confirmed.
- Grounded-SAM-2 end-to-end (`grounded_sam2_local_demo.py` on the repo's own truck image; then real captures).
- GraspGen end-to-end against a **real ZED point cloud**: 50 grasps, confidence 0.84–0.92, orthonormal rotations, grasp positions inside the input cloud's bounding box — physically plausible, not garbage.
- Segmented-object pipeline end-to-end: GroundedSAM mask → object cloud → GraspGen → 50 grasps at 0.77–0.95, rising to 0.94–0.98 after the noise fix.
- `easy_handeye2` eye-on-base calibration performed; TF `fr3_link0 → zed_camera_link` published and echoed.
- FR3 MoveIt stack live; the control interfaces in §3.5 confirmed.
- The Franka Hand homing requirement, diagnosed and fixed.

## Verified by me, offline, in this session
- Every artifact/contract check in Part V.
- Flying-pixel reproduction on `red_cup` (§4.2).
- `easy_handeye2` carries no local customisation (§4.7).
- The `/pragmabot` mount vs colcon build-path reconciliation (§3.4).
- `--save_grasps` implementation and its centroid contract (§4.3).
- `foundationpose/` is empty.

## Explicitly NOT verified — do not claim these
- **Pick has never been run end-to-end on the real robot.** Components were smoke-tested for build/import correctness in isolation; no live execution has happened.
- `eef_link = fr3_hand` correctness vs `fr3_hand_tcp` — flagged in the code, unresolved.
- `revolute_jump_threshold = 0.2` is a conservative guess, not tuned for this robot.
- The GraspGen ZMQ client↔server round-trip has never been exercised *from inside* the ROS 2 bridge process.
- No VPN/remote path to the robot exists, so nothing robot-facing can be tested from home.
- No `.calib` / hand-eye result file exists in this copy — **[VERIFIED HERE]** a full search found none. The calibration lives on Alonnisos only.

---

# PART VII — GOTCHAS (do not re-discover these)

**Environment**
- `export ROS_DOMAIN_ID=7` on **every** terminal, host and container. ZED topics are invisible otherwise — check `ps aux | grep zed` and read `/proc/<pid>/environ` before concluding the camera is down.
- `source /ros2_ws/install/setup.bash` in every fresh container shell; if `install/` is gone the container was recreated → rebuild.
- `docker exec -it -e DISPLAY=$DISPLAY ...` — the DISPLAY-mismatch fix is baked into the VS Code terminal profile.
- Unlock robot + **activate FCI in Desk** before MoveIt, or `libfranka: Connection to FCI refused`.

**Builds**
- `--no-build-isolation` is **mandatory** for both Grounded-SAM-2 editable installs; otherwise uv pulls an unrelated torch into a temp env and the CUDA-version check fails against nvcc.
- Pin `transformers<5` (4.57.6 works): transformers 5.x removed `BertModel.get_head_mask`, which GroundingDINO's `BertModelWarper` still calls.
- Runtime deps neither package installs: `addict`, `yapf`, `timm`, `supervision`, `pycocotools`.
- `export CUDA_HOME=/usr/local/cuda` and add `/usr/local/cuda/bin` to `PATH` — nvcc is not on PATH by default even though `CUDA_HOME` resolves.
- `setup.cfg` in `pragmabot_bridge` fixes an ament_python + `--symlink-install` quirk where console scripts land in `install/<pkg>/bin/` but `ros2 run` only looks in `install/<pkg>/lib/<pkg>/`.

**Data / algorithms**
- GraspGen needs **object-scale** clouds (~2000 pts). Scene-scale → OOM or discriminator crash.
- Always add the centroid back after `--save_grasps`.
- GraspGen origin = gripper **base link**, not fingertip.
- Compare "top-down" only in a **gravity-aligned frame**, never camera frame.
- MAD, not std, for one-sided depth outliers.
- ZED optical frame: `..._left_camera_frame_optical` (not `..._left_camera_optical_frame`).

**Traps specific to this repo layout**
- `~/pragmabot` is mounted at `/pragmabot`, **outside** the colcon build path → rsync into `~/ros2_ws/franka_ros2/` and rebuild after every edit.
- Two copies of `zed-ros2-wrapper` exist; only `zed_ros2_ws/src/` has the FR3 config changes.
- `config.yaml` topics still point at `/zedxm/...` and must be retargeted to `/zed/zed_node/...`.
- Alonnisos is shared and `/` has hit 100% before; other users' homes dominate usage. Check `df -h /` on any "No space left on device". **Not yours to fix by deleting others' files.**

---

# PART VIII — GAPS, RISKS, OPEN QUESTIONS

## 8.1 The critical gap

**Nothing connects the planner to the FR3 executor.**

- The brain runs in **ROS 1** (`rospy`, `actionlib`) and dispatches to `/panda/pick|place|push`.
- The body runs in **ROS 2** (`rclpy`) and exposes **no interface at all** — `main()` (`bridge_node.py:476-507`) performs a single parameter-driven pick and then spins.
- `pragmabot_interfaces` — the package designed to be that contract — **generates nothing** (`CMakeLists.txt:16-22`).

Everything else is a matter of degree; this is a matter of kind. Three routes exist:
1. Port the planner to ROS 2 (largest change; violates the "don't modify upstream" rule least cleanly, since the node itself is `rospy`-based).
2. `ros1_bridge` between the two.
3. A **ZMQ-style bridge** — the pattern already proven in this project for GraspGen, and the lowest-risk option: a small process-boundary RPC that the ROS 1 executor calls and the ROS 2 node serves.

**This decision is unresolved and is the single highest-value thing to settle.**

## 8.2 Where your build is genuinely ahead of the paper

Defensible, evidence-backed:
- **Depth/point-cloud quality.** The paper's largest failure bucket is execution, dominated by grasp generation and depth error; §V-E attributes remaining annotation failures to "inaccurate 3D point clouds." Your erosion + MAD fix measurably improves both cloud geometry (17.7 → 7.8 cm z-extent, reproduced here) and the grasp model's own confidence (0.77–0.95 → 0.94–0.98).
- **Grasp selection frame.** Choosing the most top-down grasp *after* transforming into a gravity-aligned robot frame is more correct than filtering by approach angle in camera frame.
- **Per-grasp gripper width.** Estimating width at the actual contact band, rather than using one object-level number — with 3× measured variation across grasps on a single object.
- **Licence-clean substitution.** GraspGen instead of AnyGrasp (machine-locked licence, breaks in Docker), following the authors' own released recommendation.

## 8.3 Where you are behind the paper

- **No annotation module in the live pipeline.** `test_fps.py` implements FPS candidate generation on a mask with numbered overlays — the mechanism from §IV-F — but it is a standalone test, not wired into planning. The paper shows this module is what fixes "grasped the wrong part of the object."
- **No `s_conf · s_loc` grasp selection** (Eq. 6). Current selection is most-top-down, which is a *kinematic* heuristic, not a semantic one.
- **No IK feasibility pre-filter** on the grasp set (the paper uses Pinocchio; MoveIt's `/compute_ik` is available and unused for this).
- **place and push unimplemented** — 2 of the paper's 3 skills.
- **No end-to-end memory experiment on the FR3.**

## 8.4 Course extensions not started

Both named in the project brief:
1. **Local VLMs** for latency/cost. The paper measures +5.15 s per annotated `gpt-4o` call and a 7.5× prompt-length penalty for full-LTM context — there is a clear, quantified target to beat.
2. **Ontology-based experience representations** as an alternative to free-text LTM entries. The paper's own §VI limitations (pruning, forgetting, top-$k$ breaking down at scale) are the natural motivation.

## 8.5 Open questions to put to the supervisor

1. Which bridge topology — ROS 2 port, `ros1_bridge`, or ZMQ?
2. One collapsed `ExecuteSkill.action` or three separate ones? (recorded at `pragmabot_interfaces/CMakeLists.txt:16-22`)
3. Is `fr3_hand` or `fr3_hand_tcp` the correct `eef_link` for GraspGen's base-link convention in this MoveIt config?
4. Given ~27 days to the report, which counts for more: place/push breadth, or a rigorous single-skill evaluation with the memory loop closed?
5. Is a **rosbag-replay** memory experiment (no robot) acceptable evidence for the report if live robot time is short?

---

# PART IX — GLOSSARY

| Term | Meaning |
|---|---|
| **Verbal RL** | Improving an agent by writing natural-language self-critique into its context instead of updating weights (Reflexion [17]) |
| **Linguistic gradient** | The paper's term for that critique — the analogue of a gradient step, applied to the prompt |
| **STM** | Short-term memory: within-episode (action, feedback) list; reset at task end |
| **LTM** | Long-term memory: persistent key→lesson store across tasks |
| **RAG** | Retrieval-augmented generation — here, top-$k$ cosine retrieval over embedded scenario keys |
| **Scenario key** | `(instruction, initial scene description)`, the LTM index |
| **Open-vocabulary segmentation** | Text prompt → mask, no fixed class list (GroundingDINO + SAM) |
| **FPS** | Farthest-point sampling — spread-out candidate points within a mask |
| **Eye-on-base** | Camera fixed in the world, calibrated to the robot base (your setup). Opposite: eye-in-hand (the paper's) |
| **Flying pixels** | Stereo artefact: silhouette pixels get depth interpolated between near object and far background |
| **MAD** | Median absolute deviation; ×1.4826 makes it comparable to a std for normal data |
| **FCI** | Franka Control Interface — must be activated in Desk for external control |
| **Approach axis** | Direction the gripper travels to reach the grasp; GraspGen convention = grasp frame **+Z** |
| **Standoff / pre-grasp** | Pose offset back along the approach axis, to allow a straight-line final approach |

---

# PART X — FILE INDEX

**Your code (FR3 execution path)**
- `ros2_ws/franka_ros2/pragmabot_bridge/pragmabot_bridge/bridge_node.py` (511)
- `ros2_ws/franka_ros2/pragmabot_bridge/pragmabot_bridge/grasp_transform.py` (160)
- `ros2_ws/franka_ros2/pragmabot_interfaces/CMakeLists.txt` (interfaces disabled at :16-22)

**Your code (perception / calibration)**
- `pragmabot/calibration/mask_to_pointcloud.py` (326) — erosion + MAD noise fix
- `pragmabot/calibration/detect_object.py` (192) — Grounded-SAM-2 wrapper
- `pragmabot/calibration/calibrate_extrinsic.py` (411), `make_object_pointcloud.py` (250), `mesh_gen.py` (193), `capture_calib_frame.py` (142)
- `pragmabot/calibration/test_fps.py` (103) — FPS annotation prototype
- `pragmabot/bags/extract_bag.py`
- `GraspGen/client-server/graspgen_client.py` — `--save_grasps` (:99-107, :218-219)
- `GraspGen/grasp_filter.py`, `GraspGen/scripts/capture_zed_frame.py`

**Your code (ROS 1, superseded)**
- `pragmabot/src/pragmabot/panda_skill_executor.py` (93)
- `pragmabot/nodes/panda_{pick,place,push}_server.py`
- `pragmabot/src/pragmabot/{claude,gemini}_vlm_client.py`, `grounded_sam.py`, `graspgen_client.py`, `geometry.py`

**Upstream, read-only**
- `pragmabot/src/pragmabot/vlm_*.py`, `memory_manager.py`, `scene_observer.py`, `conversation_builder.py`, `utils.py`, `simple_config.py`
- `pragmabot/nodes/pragmabot_node.py` (executor wiring at :91, :143)

**Docs on disk (note: `PROJECT_OVERVIEW.md` predates the FR3/host-only architecture and is stale in its "Track A/Track B" framing)**
- `pragmabot/CLAUDE.md` — most current architecture + gotcha record
- `extras/memory.md` — most recent session log (2026-08-14)
- `extras/SETUP.md` — laptop rebuild guide
- `pragmabot/readme.md` — lab notebook, real command transcripts
- `ros2_ws/franka_ros2/WORKSPACE_STRUCTURE.md`

**Paper materials**
- `C:/Users/haris/claude-papers/papers/pragmabot/` — `paper.pdf`, `paper.txt`, `meta.json`, `images/` (63 extracted)
- `C:/Users/haris/claude-papers/papers/irobman_lab/paper.txt`

---

# PART XI — LATE FINDINGS (added after figure inspection and full code read)

These were found on a second pass: viewing the paper's figures as images rather than relying on extracted caption text, and reading line by line the files I had initially only skimmed. Several correct or add to earlier sections.

**Paper identifier:** arXiv **2507.16713** (`https://arxiv.org/abs/2507.16713`). alphaxiv hosts the paper but carries **no community reviews, comments, or ratings** — only an auto-generated summary. There is no external review discussion to mine.

## 11.1 What the figures contain that the text does not

**Fig. 3 — the literal prompt skeletons.** Now readable in full:

| Module | Prompt structure |
|---|---|
| Scene Describer | Instruction · observation `<IMG>` · "Provide a short scene description." |
| Task Planner | Instruction · observation `<IMG>` · "Here is the action history: `<STM>`" · planning rules and constraints · "Here are the past experiences: `<LTM>`" · "Choose the next best action." |
| Success Detector | Instruction · "Action the robot just attempted: `<Action>`" · "Observation before the action: `<IMG>`" · "Observation after the action: `<IMG>`" · success/failure criteria and reasoning rules · "Output your structured evaluation." |
| Experience Summarizer | Instruction · "Scene: `<Initial scene description>`" · "Summarize the robot's experiences: `<STM>`" |

**Ordering is a deliberate design choice:** in the planner prompt, `<STM>` comes *early* (right after the observation), the rules sit in the middle, and `<LTM>` comes *last*, immediately before "Choose the next best action." Retrieved experience is placed closest to the decision point. Worth preserving if you re-implement the prompt.

**Fig. 8 (top) — annotation helps only where shape is complex.** Reading the bars per object:

| Object | w/ annotation | w/o annotation |
|---|---|---|
| box | 100% | 100% |
| mug | 100% | 100% |
| banana | 80% | 80% |
| drumstick | 80% | 40% |
| **skewer** | **100%** | **20%** |
| ice cream | 80% | 40% |
| brush | 80% | 20% |

For three of seven objects annotation changes **nothing**. The gain is concentrated entirely on objects with a functionally-correct grasp *section* (skewer, brush, drumstick, ice cream). **This is the real argument for making annotation on-demand rather than always-on** — the paper's +5.15 s cost would be pure waste on half the objects. The text says "significantly improves"; the figure says "improves a specific half, and is free of benefit on the rest."

**Fig. 8 (bottom) — push distance error (cm, lower better), w/ vs w/o annotation:** egg to sushi about 4.5 vs 9 · sushi to plate about 5 vs 12 · cherry to banana about 7 vs 8 · grape to banana about 8 vs 14 · screw to toolbox about 4 vs 10.5 · candy to banana about 2 vs 5 · paper to box about 2 vs 7. Consistent improvement, largest on the harder pushes.

**Fig. 7 — latency and token baselines** (approximate, read from the bars):

| Setting | Prompt tokens | Response time |
|---|---|---|
| rag-4o | ~1,100 | ~9.8 s |
| all-4o | ~8,300 | ~10.5 s |
| rand-4o | ~1,150 | ~10.1 s |
| rag-mini | ~1,100 | ~6.7 s |
| all-mini | ~8,300 | ~7.1 s |
| rand-mini | ~1,150 | ~7.2 s |

> **Directly useful for the course's local-VLM extension:** the number to beat is **~10 s per planning call** for `gpt-4o` and **~7 s** for `gpt-4o-mini`, at ~1,100 prompt tokens under RAG. The 7.5x token penalty of full-LTM context is confirmed visually (~1,100 to ~8,300).

**Fig. 4 — annotation made concrete.** "Pick up drumstick" shows FPS candidates 1-5 overlaid on the drumstick with a green check on candidate 2, at the **stick end rather than the meat**. "Push grape to banana" shows a grid of numbered candidate goal masks between grape and banana with the selected one checked. This is the clearest single illustration of "geometry proposes, semantics disposes."

**Fig. 2 — pipeline detail.** LTM is drawn as explicit paired columns (`Scenario i` and `Experience i`), and the STM box shows per-timestep entries `t = 0: <summary 0>`, `t = 1: <summary 1>`, and so on — i.e. STM entries are *summaries*, not raw dumps.

## 11.2 CORRECTION — the real `NextBestAction` schema

`PROJECT_OVERVIEW.md` section 6 documents this schema:

```python
class NextBestAction(BaseModel):     # <-- WRONG, does not match the code
    reasoning: str
    skill: str
    target_object: str
    target_location: str | None = None
    use_annotation: bool = False
```

The **actual** schema (`pragmabot/src/pragmabot/vlm_task_planner.py:88-128`) is:

```python
class RobotSkill(str, Enum):          # :73-79   -- note: NO "done" member
    PUSH = "push"; PICK = "pick"; PLACE = "place"

class PushDirection(str, Enum):       # :81-86
    LEFT = "left"; RIGHT = "right"

class NextBestAction(BaseModel):      # :88
    scene_description: str                              # :91
    applicable_knowledge: Optional[str]                 # :95   <- RAG integration point
    chain_of_thought_reasoning: str                     # :99
    chosen_action: str                                  # :103  <- text the detector grades
    chosen_skill: RobotSkill                            # :107
    target_object: str                                  # :108
    should_grasp_at_specific_section: Optional[bool]    # :112  <- on-demand annotation (pick)
    placement_object: Optional[str]                     # :116  <- "place ON, not next to"
    should_place_at_specific_section: Optional[bool]    # :120  <- on-demand annotation (place)
    push_direction: Optional[PushDirection]             # :124
```

Three things this reveals that the paper's prose does not:

1. **`applicable_knowledge`** is a dedicated field where the model must first name which retrieved experiences are similar and what lesson applies. RAG is not merely stuffed into the prompt — the model is *forced to write out its use of it*. This is also precisely where the paper's "VLM overrides experience" failure (5/19) becomes observable and loggable.
2. **The annotation flag is split in two** — `should_grasp_at_specific_section` and `should_place_at_specific_section` — not one `use_annotation` boolean.
3. **There is no `done` skill.** Task completion comes from the success detector, not the planner. `SuccessEvaluation` (`vlm_success_detector.py:45-60`) is `scene_description: str`, `is_action_successful: bool`, `is_task_completed: bool`.

The scenario key is built by `utils.py:24-26`:
```python
return f"Instruction: {instruction}\nScene: {initial_scene_description}"
```

## 11.3 BUG FOUND — `PandaSkillExecutor` does not match the planner schema

**[VERIFIED HERE]** — reproduced by constructing the real schema and calling the executor's access pattern.

`pragmabot/src/pragmabot/panda_skill_executor.py` reads fields that do not exist on `NextBestAction`:

| Line | Code | Actual behaviour |
|---|---|---|
| `:30` | `skill = action.skill.lower()` | **`AttributeError: 'NextBestAction' object has no attribute 'skill'`** — the field is `chosen_skill` |
| `:38` | `elif skill == "done":` | dead branch — `RobotSkill` has no `done` member |
| `:63` | `getattr(action, "target_location", "")` | silently returns `""` — the field is `placement_object` |
| `:79` | `goal_region=getattr(action, "target_location", "")` | silently returns `""` |

Line 30 raises immediately, so **the ROS 1 execution path would crash on its very first real call**. Lines 63/79 are worse in character: they fail *silently*, sending empty strings as goals.

**Why it was never caught:** `config.yaml` sets `rosbag_replay: true`, and `pragmabot_node.py:139-141` branches straight to `handle_evaluation_request` without ever calling the executor. The only code path that would expose the bug is the one that has never been run.

**Impact on the current plan:** low, because the ROS 1 executor is superseded by `pragmabot_bridge`. But it is a direct warning for the bridge work: **whatever connects the planner to the FR3 must read `chosen_skill`, `target_object`, `placement_object`, `push_direction`, and the two `should_*_at_specific_section` flags** — not the schema documented in `PROJECT_OVERVIEW.md`. Writing the new executor against that stale doc would reproduce exactly this bug.

## 11.4 `MemoryManager` internals — the extension seam

`pragmabot/src/pragmabot/memory_manager.py` (308 lines), read in full through `:150`.

- Two CSVs merged on `scenario` by **left join** (`:72`); entries lacking an embedding get `None` and are warned about (`:74`).
- Embeddings are stored **base64-encoded** in a model-specific file `ltm_<embedding_model>.csv` (`:44`, `:69`) — the mechanical reason LTM is not portable across VLM backends.
- Retrieval (`:84-138`): embed the query key, cosine-similarity across the frame, `sort_values` descending, `head(top_k)` (`:109-111`).
- **Random-retrieval ablation** is implemented as `sample(frac=1)` with similarities forced to `0.0` (`:116`) — this is the `rand-*` condition in Fig. 7.
- Each retrieved entry is handed to the planner as a **JSON string** `{"scenario": ..., "experience": ...}` (`:121-123`).

> **This last point is the cleanest insertion point for the ontology-based memory extension.** The planner consumes retrieved memory as serialized JSON text. A structured/ontological representation can be substituted at `retrieve_relevant_experiences`'s return boundary **without modifying the planner at all** — which keeps the project's "never modify upstream VLM modules" rule intact. The natural experiment is: same tasks, same LTM content, free-text vs structured encoding, measuring first-action accuracy (the paper's own section V-D metric) and the "overrides experience" failure rate.

## 11.5 `mask_to_pointcloud.py` — the third filter, and why it exists

My earlier description covered erosion and the MAD filter but under-stated the **AABB crop**, which has its own distinct justification (header, `:37-45`):

> the MAD filter is **depth-only**, so mask leakage sideways onto the table — which has a perfectly normal depth — survives it untouched while still stretching the cloud laterally and dragging the centroid off the object. **GraspGen re-centres its input on the cloud mean before denoising, so a centroid pulled off-object shifts every grasp it generates.**

That is the sharpest piece of reasoning in the codebase: the filter is justified by a specific downstream behaviour of the grasp model, not by aesthetics.

The padding is a **fraction of the object's own percentile span** (`:47-53`), not an absolute distance, because "2 cm of margin is reasonable on a 30 cm box and larger than the object itself on a 4.5 cm cube, which silently turns the filter into a no-op."

Defaults (`:275-296`): `--max-range 3.0`, `--erode-px 3`, `--z-outlier-mad 3.5`, `--aabb-percentile 2.0`, `--aabb-pad-frac 0.05`, `--target-points 2000`.

Other quality signals in this file worth citing in the report:
- **`--diagnose` mode** (`:151-203`) reports what *each* filter would remove **independently**, so you can see which one is doing the work and which is a no-op — including an explicit "dropped nothing, so the box is too loose" hint (`:194-197`).
- **Centroid-shift instrumentation** (`:238-244`): prints how far filtering moved the centroid and warns above 1 cm *with the GraspGen re-centring reason attached*.
- **Plausibility bounds** (`:63-64`, `:246-254`): warns if the largest extent exceeds 60 cm ("background is likely still leaking through") or falls under 1 cm ("--erode-px ate a small object").
- **Scale check** (`:199-202`): tells the operator that if extents disagree with a ruler, "the problem is the mask or the depth, not the filters — no amount of tuning will fix it here." Correctly refuses to let parameter-twiddling mask an upstream fault.
- Downsampling uses a seeded RNG (`:257-258`), so it is reproducible.

### Exact offline reproduction **[VERIFIED HERE]**

Running the full three-filter pipeline at default parameters on `extracted/red_cup/` (raw `depth.npy` + `mask.npy` + `intrinsics.json`):

| Stage | Points | Extent (cm) |
|---|---|---|
| unfiltered | 6334 | 11.6 x 11.4 x **17.7** |
| + erode 3 px | 5786 | 9.9 x 10.8 x **7.8** |
| + z-MAD 3.5 | 5786 | 9.9 x 10.8 x 7.8 |
| + AABB 2% / 5% pad | 5673 | **9.5 x 10.7 x 7.3** (113 dropped) |
| **shipped `object_pcd.npy`** | **2000** | **9.5 x 10.7 x 7.3** |

The pipeline **reproduces the shipped artifact's extent exactly**; the point-count difference is the final downsample to `--target-points 2000`. Centroid shift vs unfiltered: 0.19 cm.

On this scene erosion does essentially all the work (17.7 to 7.8 cm) and the MAD filter removes nothing further — consistent with the file's own point that `--diagnose` exists precisely because *which* filter matters is scene-dependent.

Root-cause split independently reproduced: **boundary pixels 11.9% far-outlier rate vs interior 1.5%** (about 8x). The cube scene in the project notes logged 41% vs 10%; different scene, same mechanism.

---

# PART XII — ROS VERSION AUDIT (2026-08-19)

Answering directly: **is anything ROS 1, given that `franka_ros2` and `zed_ros2_ws` are both ROS 2?**

## 12.1 Yes — exactly one package

**`D:/pb/home/pragmabot/pragmabot/`** — the upstream PragmaBot package — is **ROS 1 (catkin)**. Nothing else in the workspace is.

Evidence, all from files on disk:
- 10 files import `rospy` / `actionlib`: `nodes/{pragmabot_node, memory_manager_node, image_decompressor_node, image_republisher_node, panda_pick_server, panda_place_server, panda_push_server}.py` and `src/pragmabot/{graspgen_client, panda_skill_executor, scene_observer}.py`
- `pragmabot/CMakeLists.txt` + `package.xml` are catkin
- 6 ROS 1 XML launch files in `pragmabot/launch/`
- Upstream targets Ubuntu 20.04 + ROS Noetic

Everything else is ROS 2: `franka_ros2` (Humble), `zed_ros2_ws`, `pragmabot_bridge`, `pragmabot_interfaces`, `easy_handeye2`, `calibration/*.py`, `bags/extract_bag.py`, `GraspGen/scripts/capture_zed_frame.py`. **36 `.launch.py` files vs 6 `.launch`.**

## 12.2 The coupling is much shallower than the file count suggests

Of the **17** library modules in `pragmabot/src/pragmabot/`, only **3** import ROS:

| Module | Lines | Disposition |
|---|---|---|
| `scene_observer.py` | 92 | port to `rclpy` |
| `panda_skill_executor.py` | 93 | rewrite (also carries the schema bug, §11.3) |
| `graspgen_client.py` | 95 | drop — standalone GraspGen client covers it |

The other 14 are **pure Python, zero ROS**: `vlm_task_planner.py` (206), `memory_manager.py` (308), `vlm_success_detector.py` (130), `vlm_exp_summarizer.py` (100), `vlm_scene_describer.py` (95), `vlm_client.py` (93), `conversation_builder.py` (129), `claude_vlm_client.py` (120), `gemini_vlm_client.py` (125), `utils.py` (67), `geometry.py` (85), `grounded_sam.py` (76), `simple_config.py` (29), `__init__.py`.

> **The paper's entire contribution — planner, memory, RAG, success detection, summarisation — is ~1,400 lines of framework-free Python.** It ports to ROS 2 for free, untouched. That satisfies the "never modify upstream VLM modules" rule *by construction* rather than by exception.

Node-level work to go all-ROS 2:

| File | Lines | Action |
|---|---|---|
| `pragmabot_node.py` | 423 | port (`rospy` → `rclpy`, keep Gradio) |
| `scene_observer.py` | 92 | port |
| `panda_{pick,place,push}_server.py` | 553 | **delete** — superseded by `pragmabot_bridge` |
| `memory_manager_node.py` | 296 | optional (LTM inspection UI) |
| `image_{decompressor,republisher}_node.py` | 178 | optional (rosbag replay helpers) |

**Realistic port surface ≈ 515 lines**, plus the executor that must be written regardless.

## 12.3 Consequence — recommendation revised

Deliverable 2's Step 2.1 originally recommended a **ZMQ bridge** to avoid porting. That was correct only under the assumption that the port was large. It is not. **Revised recommendation: port to ROS 2 and drop ROS 1 entirely.**

- vs `ros1_bridge` — no second ROS distro, no bridge process, no message-type mapping.
- vs ZMQ — ZMQ stays right for **GraspGen** (genuinely incompatible CUDA env, ships its own server). It is wrong between planner and executor, where both sides are Python in one ROS graph and you would be hand-rolling goals/feedback/cancellation that ROS 2 actions already provide, plus `ros2 action` introspection.
- One `colcon` workspace builds `pragmabot`, `pragmabot_bridge`, `pragmabot_interfaces` together, which **removes the rsync workaround** forced by `/pragmabot` being mounted outside the colcon build path (§3.4).
- `ROS_DOMAIN_ID=7` already unifies discovery across container and host.

## 12.4 Duplicate-copy hazard (verified)

`pragmabot_bridge` and `pragmabot_interfaces` each exist in **two** places:
- `D:/pb/home/pragmabot/ros2_ws/src/…` (source of truth)
- `D:/pb/home/ros2_ws/franka_ros2/…` (rsync'd copy, inside the colcon build path)

**[VERIFIED HERE]** all five compared files are currently **byte-identical** (`bridge_node.py`, `grasp_transform.py`, `setup.py`, `package.xml`, `setup.cfg`). So the copies are in sync *today* — but this is the same hazard as the two `zed-ros2-wrapper` checkouts (§3.6): edit one, forget the rsync, and they diverge silently. **Consolidating into one ROS 2 workspace removes this entire failure mode**, which is a second, independent argument for the port.
