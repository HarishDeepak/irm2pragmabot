# PragmaBot × Franka FR3 — Project Narrative

*Career material: what this project is, what it actually took, and how to talk about it.*

**Ground rule for this whole document:** every claim here survives a follow-up question. Where something is built but not yet validated on hardware, it says so — because the strongest version of this project is the true one, and because an interviewer who catches one inflated claim discounts everything else you said. There is a short section at the end (§11) listing exactly what **not** to claim, and what to say instead.

---

## 1. What this project is

**One line:**
> Reproducing a 2026 IEEE RAL paper on VLM-driven robot task planning on a real Franka FR3 — which meant building, from scratch, the entire perception-to-motion execution layer the paper deliberately leaves unimplemented.

**Three lines:**
> PragmaBot (ETH Zürich RSL) lets a robot improve its task planning by failing, writing a natural-language critique of the failure, remembering it, and retrieving it when a similar situation recurs — with no model fine-tuning at any point. The published code implements the reasoning and memory layers and leaves action execution as a deliberate `NotImplementedError`. This project is the other half: open-vocabulary segmentation, point-cloud reconstruction, 6-DoF grasp synthesis, hand-eye calibration, and ROS 2 motion execution on a Franka FR3 with a stereo camera — plus the reproduction of the paper's memory claims on that platform.

**The framing that matters.** This is not "I ran someone's repo." The paper's contribution is a *brain*. It ships with no *body*, and explicitly says so. A robotics reproduction of this paper is, unavoidably, a systems-integration and manipulation project wearing a research-reproduction label. That is precisely why it is good material: it forces you across the full stack, from a stereo camera's noise physics up to retrieval-augmented reasoning.

**Context:** Praktikum zur intelligenten Robotermanipulation (Part II), TU Darmstadt, PEARL Lab / iRobMan, Apr–Sep 2026. Project 3, *"Memory representations for Robotic Task Planning."* Real-world track, on lab hardware.

---

## 2. Why this is a hard problem

### 2.1 The research problem

Large language and vision-language models are the default choice for robot task planning because they bring common sense for free: they know a plate goes under an apple, that you empty a container before lifting it. But that common sense is *human* common sense, learned from internet text and images. It encodes what is easy for a person with two hands, stereo vision, and decades of contact experience.

A robot has a specific arm, a specific gripper, a specific perception stack, and a specific set of things it silently fails at. **Nothing in pretraining tells the model any of that.** The paper's own example: shown a tennis ball half-occluded by a fan, a VLM confidently says "pick up the tennis ball," because for a human that is trivial. The robot's gripper collides with the fan.

This is the **embodiment gap**. The obvious fix — fine-tune the model on robot data — is expensive, needs data collection on hardware, and must be redone per platform.

PragmaBot's answer is to move the learning out of the weights and into the context: let the robot fail, have the VLM look at before/after images and judge what happened, have it write down *why* it failed in natural language, keep that note for the rest of the task, distil it when the task succeeds, and retrieve it by semantic similarity next time. The paper calls the critique a **"linguistic gradient"** — it plays the role a gradient plays in RL, but the update lands in the prompt.

### 2.2 The engineering problem underneath it

Here is the part that makes this a real project rather than a prompt-engineering exercise.

For the loop to close, *something* must turn `"pick up the red cup"` into joint motion, and then produce an image the model can judge. That requires, in order:

1. Turning a **noun phrase** into a **pixel mask** — open-vocabulary, since the object set is not known in advance.
2. Turning that mask plus a depth image into a **metrically correct 3D point cloud** — which means confronting the fact that stereo depth is systematically wrong exactly at object boundaries.
3. Turning a point cloud into **SE(3) grasp poses** — and getting the frame conventions right, because a grasp pose is meaningless without knowing whether its origin is the gripper base or the fingertips.
4. Transforming those poses from **camera frame into robot base frame** — which requires hand-eye calibration, and is unforgiving: a few degrees of rotational error becomes centimetres of position error at the workspace.
5. Planning and executing motion that **does not pass through a singularity**, does not collide with the table, and recovers when the gripper driver faults.

Each of those five steps has a failure mode that is silent — it produces plausible-looking wrong output rather than an error. That is what makes manipulation hard, and it is the thing this project is genuinely about.

The paper's own failure analysis confirms the point: of 19 first-attempt failures across 91 trials, **8 came from execution** — 5 from poor grasp generation, 3 from inaccurate depth — the single largest category, and the layer the paper does not contribute.

---

## 3. System architecture

```
  ┌───────────────────────── REASONING LAYER (from the paper) ─────────────────────────┐
  │                                                                                    │
  │  instruction ──▶ Scene Describer ──▶ scenario key = (instruction, scene)            │
  │                                              │                                     │
  │                                              ▼                                     │
  │                            RAG: top-k cosine over long-term memory                 │
  │                                              │                                     │
  │       short-term memory ────────────────────▶│                                     │
  │       (action, feedback) pairs               ▼                                     │
  │                                        Task Planner ──▶ skill + target + flags     │
  │                                              │                                     │
  │       Success Detector ◀─────────────────────┼──── before/after images             │
  │              │                               │                                     │
  │              └──▶ Experience Summarizer ──▶ long-term memory                       │
  └──────────────────────────────────────────────┼─────────────────────────────────────┘
                                                 │
                    ══════════ the layer this project builds ══════════
                                                 │
  ┌───────────────────────── EXECUTION LAYER (built here) ─────────────────────────────┐
  │                                                                                    │
  │  ZED2 RGB ──▶ Grounded-SAM-2  (GroundingDINO text→box, SAM2 box→mask) ──▶ mask     │
  │                                                                                    │
  │  ZED2 depth + mask ──▶ back-projection + de-noising ──▶ object point cloud         │
  │                        (erosion · MAD · AABB)                                      │
  │                                                                                    │
  │  point cloud ──▶ GraspGen (ZMQ server) ──▶ N × SE(3) grasps + confidences          │
  │                                                                                    │
  │  grasps ──▶ un-center · TF into fr3_link0 · select · standoff · width estimate     │
  │                                                                                    │
  │  ──▶ MoveIt 2: MoveGroup to standoff → Cartesian approach → grasp → lift           │
  │      franka_gripper: homing · grasp · move                                         │
  └────────────────────────────────────────────────────────────────────────────────────┘
```

**Hardware:** Franka FR3 (firmware 5.9.0, Franka Hand), ZED2 stereo camera statically mounted (eye-on-base), RTX 4080 workstation.

**Environments:** four mutually incompatible Python/CUDA environments running side by side — GraspGen (`torch==2.1.0`, Python 3.10), Grounded-SAM-2 (`torch>=2.3.1`, Python 3.10), the ROS 2 control container (ROS Humble), and the ROS 1 planner. Getting these to cooperate is a substantial part of the work, and §4.6 explains how.

---

## 4. The engineering concepts, in depth

This is the section to actually understand, because these are the things an interviewer will probe. Each subsection states the concept, why it is subtle, and what was done.

### 4.1 Open-vocabulary segmentation — turning a noun into pixels

**The concept.** Classical segmentation works over a fixed label set: train on 80 COCO classes, get 80 classes. That is useless when the planner may name any object. Open-vocabulary detection removes the fixed set: **GroundingDINO** takes free text and produces boxes by grounding language against image features; **SAM 2** turns a box prompt into a precise mask. Chained, they give you `text → mask`.

**Why it is subtle.**
- The text convention is load-bearing: GroundingDINO expects lowercase phrases each terminated by a period (`"red cube. e-stop button."`). Get it wrong and detection quality collapses for reasons that look like model failure.
- **The model will confidently detect the wrong thing.** In one real capture, prompting `"cube."` returned the actual cube at confidence 0.57 *and* the robot's red e-stop button at 0.48 — a red, roughly cubic object. A naive "take everything above threshold" policy would have unioned both into one mask and produced a point cloud spanning two objects a metre apart. The system takes the top-confidence match by default, with an explicit option to union.
- The planner never sees the raw user instruction. Spatial language ("the one on the left") is resolved upstream by the scene describer into a concrete noun, which is what the segmenter receives. This separation of concerns matters: the segmenter is not asked to do reasoning it cannot do.

**What was built.** A wrapper producing a boolean mask plus an annotated preview and a JSON record of every detection above threshold. It required a non-obvious fix: the upstream repository's `inference.py` self-imports as `grounding_dino.groundingdino...` — a repo-root-relative path, not the installed package name — so the repo root must be injected onto `sys.path` before import or the module fails to resolve. That class of packaging bug is invisible until you hit it.

### 4.2 Back-projection — from a mask to metric 3D

**The concept.** The pinhole camera model inverted. For each masked pixel $(u,v)$ with depth $z$:

$$x = \frac{(u - c_x)\,z}{f_x}, \qquad y = \frac{(v - c_y)\,z}{f_y}, \qquad z = \text{depth}$$

Camera intrinsics $(f_x, f_y, c_x, c_y)$ come from the camera's own calibration.

**Why it is subtle.** This is the step where a small error becomes a physically wrong object. Real intrinsics matter — an early version of this project carried `fx = fy = 700` as a placeholder; the real ZED2 values are `528.604` and `521.536` across two capture sessions. Using the placeholder would have scaled every reconstructed object by ~33%, and every grasp with it.

### 4.3 Stereo "flying pixels" — the most interesting problem in the project

**The symptom.** The reconstructed cloud did not look like the object it came from. A cube whose true depth extent is 4–5 cm produced a point cloud spanning **16.7 cm** in z. On a second scene, a cup spanning ~7 cm produced **17.7 cm**.

**The wrong diagnosis** — and the one most people would reach for — is "the segmentation mask is bad." It was not.

**The actual diagnosis.** Split the mask into two regions: the eroded interior, and the boundary ring that erosion removes. Compare the depth statistics of each.

- Interior pixels: z standard deviation **1.7 cm** — a real, compact object.
- Boundary pixels: **~4× more likely** to be a far-outlier (>4 cm from the median) than interior pixels — 41% vs 10% on the cube scene.
- Independently re-measured on the cup scene: **11.9% vs 1.5%**, roughly 8×.

The 2D mask was accurate. **The depth values at silhouette pixels were wrong.** This is a well-understood stereo artefact: at a depth discontinuity, the block matcher cannot decide whether a boundary pixel belongs to the near surface or the far background, so it returns something in between — producing a comet-tail of points trailing off the object's edge into empty space.

**The fix — three filters, each justified separately.**

1. **Morphological erosion** (default 3 px) shrinks the mask inward before back-projection, discarding exactly the boundary ring where the artefact lives. Implemented as a pure-numpy 3×3 cross-kernel erosion, deliberately avoiding OpenCV and SciPy — because this script must run inside GraspGen's virtual environment, which has neither, and adding them risks perturbing a working CUDA dependency set.

2. **Median-absolute-deviation depth filter** (default 3.5 scaled MAD) drops points far from the median depth. **The choice of MAD over mean/standard deviation is the point.** The artefact is *one-sided* — flying pixels trail toward the background, never toward the camera. A mean/std filter is dragged by the very outliers it is meant to remove; a median/MAD filter is not. The `1.4826` scaling factor makes MAD comparable to a standard deviation for normally distributed data.

3. **Robust axis-aligned bounding-box crop.** This one exists for a distinct reason worth understanding: **the MAD filter is depth-only.** If the mask leaks sideways onto the table, those points have a perfectly normal depth — they survive the depth filter untouched, while stretching the cloud laterally and dragging its centroid off the object. And that matters specifically because **GraspGen re-centres its input on the cloud mean before inference — so a centroid pulled off-object shifts every grasp it generates.** The box corners come from percentiles rather than min/max, because the min/max *are* the stray points; a box built from them would enclose them by construction.

   The padding is expressed as a **fraction of the object's own span**, not an absolute distance — because 2 cm of margin is reasonable on a 30 cm box and larger than the object itself on a 4.5 cm cube, where it would silently turn the filter into a no-op.

**The measured result.** On the cube scene: 1947 → 1449 points, z-extent **16.7 cm → 4.5 cm**, and GraspGen's own confidence rose from **0.77–0.95 to 0.94–0.98**. That last number is the one that matters — the cleaner input measurably improved the downstream model's confidence in its own output. It was not a cosmetic improvement to a visualisation.

Independently reproduced on the second scene: 17.7 cm → 7.3 cm, with the full pipeline reproducing the shipped artifact's dimensions exactly (9.5 × 10.7 × 7.3 cm).

**Why this is a genuine contribution rather than housekeeping.** The paper's own failure analysis attributes 8 of 19 first-failures to execution — 3 of them explicitly to *"inaccurate depth values from the stereo camera"* — and separately notes that remaining annotation failures are *"mostly due to inaccurate 3D point clouds."* This work attacks the published system's largest measured weakness, with a quantified root cause and a quantified improvement.

**The instrumentation is part of the contribution.** The tool has a `--diagnose` mode that reports what *each* filter would remove **independently**, so an operator can see which one is doing the work and which is a no-op — including an explicit hint when a filter drops nothing ("either the cloud is already clean, or your box is too loose"). It warns when the centroid moves more than 1 cm, *with the GraspGen re-centring reason attached*. And it prints a scale check telling the operator that if the extents disagree with a ruler, the fault is in the mask or the depth and **"no amount of tuning will fix it here."** That last line is a deliberate refusal to let parameter-twiddling mask an upstream fault — which is a real engineering judgement, not a comment.

### 4.4 6-DoF grasp synthesis and frame conventions

**The concept.** A learned model takes an object point cloud and proposes SE(3) gripper poses with confidence scores. GraspGen (NVIDIA) ships a pretrained model for the Franka Panda gripper — the correct match for the FR3's Franka Hand — and runs as a ZMQ server (msgpack over a REP socket) so it can live in its own CUDA environment.

**Why it is subtle — three traps, all silent.**

**Trap 1: the origin is not where you think.** GraspGen's convention is: approach axis is the grasp frame's **+Z**, finger-closing axis is **+X**, and the **origin is the gripper base link — not the fingertips, not the TCP.** If you treat the returned pose as a fingertip pose, every grasp is wrong by the hand-to-fingertip distance (~10 cm) in a direction that varies per grasp. Nothing errors; the robot simply reaches into empty space or into the table.

> A verification worth knowing how to describe: take each grasp origin and advance it 0.75 × the gripper depth along its own +Z. If the convention is being handled correctly, that point lands on the object. Across six grasps it landed 5.8–7.2 cm from the cloud centroid, inside the bounding box. Grasp *origins* sitting outside the point cloud is therefore expected — and would look like a bug to someone who had not checked.

**Trap 2: the client silently re-centres the cloud.** GraspGen's client subtracts the point cloud's mean before sending it to the server, so returned grasps are relative to a *recentred* cloud. If you do not add the centroid back, every grasp is offset by the object's own position in the camera frame — which, for an object 60 cm from the camera, is a 60 cm error. The pipeline was extended to persist the subtracted centroid alongside the grasps precisely so the consumer can undo it, and the consumer does so as its first operation.

**Trap 3: scale.** The model expects an *object-scale* cloud of roughly 2000 points. A full unfiltered ZED frame (~100k points spanning metres) attempts a 40 GiB CUDA allocation and dies. A depth-cropped but still room-scale cloud crashes the discriminator's outlier-removal step — a k-NN distance matrix over a non-compact cluster removes every point, producing a reshape error deep inside the library. Neither failure says "your input is the wrong scale." Segmentation-driven cropping is what makes the scale correct automatically, rather than by hand-tuned bounding boxes.

### 4.5 Hand-eye calibration and the transform chain

**The concept.** Grasps are computed in the camera's optical frame. The robot plans in its base frame. You need $T^{\text{base}}_{\text{camera}}$.

Two configurations exist: **eye-in-hand** (camera on the wrist, moves with the robot — what the paper used, an elbow-mounted ZED X Mini) and **eye-on-base** (camera fixed in the world — this project's setup). Eye-on-base is a one-time calibration, but it is unforgiving: a static error never averages out, and every grasp for the rest of the project inherits it.

**Method used.** ArUco-marker-based eye-on-base calibration via `easy_handeye2`, solving for `fr3_link0 → zed_camera_link`, with the arm jogged by hand under a gravity-compensation controller to collect marker observations across a spread of poses. The published transform is then verified independently by echoing the TF and by switching RViz's fixed frame to the robot base and confirming the point cloud lands on the robot.

> **Attribution, stated precisely:** `easy_handeye2` is a third-party ROS 2 package, used unmodified — a recursive search of its source for FR3/ZED-specific strings returns zero hits; it is driven entirely through launch arguments. The work here is **calibration execution, integration, and verification**, not authorship of the calibration algorithm. Say it that way. Claiming otherwise is both false and unnecessary — performing and validating a hand-eye calibration on real hardware is a real skill.

**A second method was designed and abandoned**, and the reasoning is worth carrying: pose-estimating a textured cube with FoundationPose and composing $T^{\text{base}}_{\text{cam}} = T^{\text{base}}_{\text{cube}} \cdot (T^{\text{cam}}_{\text{cube}})^{-1}$ directly, rather than solving the classical AX = XB hand-eye problem. It was dropped once the ArUco path worked. One finding from it is permanently useful: **a plain-coloured cube has 24 rotational symmetries, and a pose estimator returns *a* valid pose rather than an error** — possibly the wrong symmetric equivalent. One face must be uniquely marked before cube-based pose estimation is usable at all. That is the kind of thing that produces a calibration that is confidently, silently wrong.

**The transform chain in practice** is `fr3_link0 → zed_camera_link` (from calibration) `→ zed_camera_center → zed_left_camera_frame → zed_left_camera_frame_optical` (from the camera driver). tf2 composes all of it in a single lookup. A related trap: the ZED's optical frame is `..._left_camera_frame_optical`; the older `..._left_camera_optical_frame` word order is deprecated and resolves to nothing.

**A design decision here is worth stating in interviews.** The camera driver was configured to *stop* publishing its own TF and odometry, because the FR3 setup owns its TF tree via `robot_state_publisher` plus the calibrated transform. Two sources publishing into one TF tree is a classic, genuinely hard-to-debug failure — the tree silently reparents and transforms start returning plausible but wrong answers. Turning it off is the correct call, and being able to explain *why* demonstrates you understand TF rather than just using it.

### 4.6 Grasp selection, and a frame subtlety worth its own discussion

Given N candidate grasps, which do you execute?

**The naive answer** is highest confidence. But confidence measures grasp quality in isolation, not reachability or safety in the robot's workspace.

**A better answer** — and the one used — is: prefer the most top-down approach, because on a tabletop a top-down grasp is least likely to collide with the table or approach through the object.

**The subtlety, and this is the part worth articulating.** "Most top-down" is meaningless in the camera frame, because the camera's own tilt is arbitrary. It is only meaningful in a **gravity-aligned frame**. So the selection must happen *after* transforming all candidates into the robot base frame — the approach axis of each grasp is computed as $R \cdot [0,0,1]^\top$ and the one with the most negative z-component wins.

This is a small piece of code and a real conceptual point. Notably, an alternative suggested during the project was to filter grasps by approach angle inside the grasp client — but that filter operates in camera frame, and its own help text admits uncertainty about which rotation column is the approach axis. Doing the comparison in the robot frame after a single TF lookup is more robust and easier to reason about. **Being able to explain why one of two reasonable-looking approaches is actually correct is exactly what "engineering judgement" means in an interview.**

**What is not yet implemented, and should be said:** the paper selects grasps by maximising the product of a geometric confidence and a *semantic* location score — the VLM chooses *where* on the object to grasp, and the grasp nearest that point wins. The current selection is purely kinematic. This is a known, scoped gap, not an oversight.

### 4.7 Gripper width estimation — a small function with a real idea in it

**The problem.** To close a parallel-jaw gripper on an object you must command a target width. Too wide and it does not grip; too narrow and it crushes or stalls. The obvious approach is to measure the object once and hard-code a number.

**Why that is wrong.** A non-uniform object — a cup — is a very different width at the rim than at the body. *The grasp pose determines which one the fingers actually land on.* A single hand-measured "object diameter" is therefore wrong for most grasps on most non-trivial objects.

**The approach.** Transform the object's point cloud into the grasp's own local frame, keep only the points within the gripper's fingertip contact band (local $z \in [\text{depth}/2, \text{depth}]$, where `depth` is read from the gripper's own config rather than guessed), and take a robust **percentile** spread along the local X axis — the finger-closing direction. Percentiles rather than min/max, because min/max are exactly the noise.

**The evidence that this was worth doing.** Measured across six valid grasps on a *single* object: **12.1, 18.2, 18.8, 21.3, 33.1, 33.2 mm.** Nearly a 3× spread. A single measured number would have been wrong for most of them.

**The honesty in it.** The function raises an exception rather than guessing when fewer than 10 points fall in the contact band, and its documentation states plainly that it relies on a single-view silhouette-width assumption and should not be trusted to the millimetre — recommending that the gripper's tolerance parameter be kept generous. Writing down the limits of your own estimator is a habit worth demonstrating.

### 4.8 Motion execution, singularities, and driver faults

**Pre-grasp standoff.** You do not fly the gripper straight to a grasp pose — you approach a **standoff** pose offset backwards along the grasp's own approach axis, then move in a straight line. This gives the planner freedom to find any collision-free path to the standoff, while the final centimetres are a controlled, predictable straight-line approach.

**The singularity guard — the piece of this section most worth understanding.** A Cartesian path is computed by inverse kinematics at small steps along a straight line. Near a **kinematic singularity**, a tiny Cartesian motion demands an enormous joint motion — the Jacobian loses rank, and joint velocities blow up. On a real 7-DoF arm this manifests as a violent, fast, dangerous motion.

MoveIt's Cartesian planner exposes a `revolute_jump_threshold`: if any single step requires a joint change larger than the threshold, it **truncates the path** and returns the fraction it managed. The critical design decision is what to do with a fraction below 1.0. **Executing a truncated path means executing a motion that stops somewhere unplanned.** This implementation aborts instead, and its error message explains the diagnosis to whoever reads the log — that a fraction well below 1.0, as opposed to exactly 0.0 from a service failure, usually means a large single-joint jump truncated the path, which is a sign the straight-line approach passes near a singularity for that grasp pose.

The threshold is documented as a conservative starting point rather than an empirically tuned value. Saying that out loud is better than pretending it was tuned.

**Retreat direction.** After grasping, the lift is along the *robot base frame's* +Z, not backwards along the grasp's own approach axis. Retracing the approach risks dragging the object back through the table it was resting on; a vertical lift does not. Small decision, real reasoning.

**Gripper fault recovery — an example of correctly diagnosing across a system boundary.** The Franka Hand became unresponsive after grasps, with the robot's own web interface reporting *"End Effector: Not connected."* The obvious hypotheses — network problem, Docker networking, ROS discovery — were all wrong. The actual cause: **the Franka Hand driver requires a `Homing` call after connecting, and after any grasp or move fault, before it will respond reliably again.** The fix is to home at the start of every pick, and to re-home and retry once if a grasp fails, rather than leaving the gripper wedged for the next attempt.

The valuable part is not the fix; it is that a symptom which looked like an infrastructure problem was correctly localised to a device driver state machine — and then written into the code's documentation so the next person does not repeat the investigation.

**A related one:** the arm's control node aborted and MoveIt never came up. Root cause was `libfranka: Connection to FCI refused` — the Franka Control Interface had not been activated in the robot's web interface. Not a code or container problem at all. Knowing that the class of "it doesn't work" includes "the robot is not in the right mode" is the difference between an hour and a day.

### 4.9 Environment architecture — the decision, and the reasoning

Four environments must coexist:

| Component | Python | torch | Why it cannot share |
|---|---|---|---|
| GraspGen | 3.10 | `2.1.0` + cu121 | pinned; compiled CUDA extensions |
| Grounded-SAM-2 | 3.10 | `>=2.3.1` + cu121 | needs a newer torch than GraspGen tolerates |
| ROS 2 control | container | — | ROS Humble + real-time constraints |
| ROS 1 planner | 3.8 | — | upstream stack |

The dependency conflict is genuine and unresolvable: GraspGen pins `torch==2.1.0`; Grounded-SAM-2 requires `>=2.3.1`. They cannot occupy one environment.

**The decision that is worth defending in an interview:** an earlier plan put each GPU tool in its own Docker container. That plan was dropped in favour of running everything host-native with per-tool virtual environments, keeping only robot control containerised.

**The reasoning:** the isolation actually required was at the *Python environment* level, not the OS level. `uv venv` provides exactly that. The container boundary added nothing — and it actively *created* a failure: a shared-memory/DDS permissions problem between root and non-root processes that cost a full day and **existed only because of the container boundary.** Removing the boundary removed the entire bug class rather than patching the symptom.

That is a defensible architectural decision with a concrete cost-benefit case, and it is a better interview answer than "we used Docker because that's best practice." The corollary is also worth saying: robot *control* stayed containerised, because there the container earns its place — a pinned ROS Humble stack with real-time scheduling privileges and a specific `libfranka` version.

**The cross-boundary pattern.** With processes in separate environments, they need to talk. GraspGen already ships a ZMQ server, so grasp synthesis is an RPC across a process boundary rather than an import. That same pattern is the lowest-risk option for connecting the ROS 1 planner to the ROS 2 executor — a decision informed by a pattern already proven to work in this system rather than chosen from a list.

### 4.10 The memory layer — what the paper contributes, and where it can be extended

Worth understanding deeply, because it is the paper's actual subject and the project's title.

**Two timescales.** Short-term memory is a per-episode list of (action, feedback) pairs, injected into every planning prompt, reset when the task ends. Long-term memory is persistent across tasks: one distilled lesson per completed task, keyed by `(instruction, initial scene description)`.

**Why the key is text.** Because it makes retrieval *semantic*. Two scenes with entirely different objects but the same structure — "the target is blocked by something next to it" — embed close together and retrieve each other's lessons. That is the whole generalisation mechanism, and it is why experience from "push the can away, then take the apple" transfers to "pick up the milk carton" with an apple leaning against it.

**The result worth carrying into any RAG work.** Retrieving five relevant memories beat providing all one hundred: **89% vs 74%** first-action accuracy, with random retrieval at **17%**. More context made the system *worse* — and cost 7.5× the prompt tokens. The full-memory condition *always* contains the correct lesson and still loses, because irrelevant context degrades attention.

> **The transferable statement:** in a retrieval system, relevance filtering is an **accuracy** mechanism, not merely a cost optimisation. That is a genuinely useful thing to believe, and it is measured, not asserted.

**The most interesting open problem in the paper**, and the natural target for the memory-representation extension: in 5 of 19 first-failures, the system **retrieved the correct memory and then ignored it** — judging the current scene sufficiently different to dismiss the lesson. Retrieval succeeded; *integration* failed. No amount of better embedding fixes that. It motivates representing a lesson as a **structured precondition the planner must explicitly discharge**, rather than as advisory prose it can talk itself out of.

The implementation detail that makes this tractable: retrieved memories are handed to the planner as serialised JSON. A structured or ontological representation can be substituted **at that boundary alone**, without modifying the planner — which preserves the project's rule of never editing the upstream reasoning modules.

---

## 5. Hard problems in this class of work, and how they are approached

Generalising beyond this project — this is the material for "what makes robotics hard?"

**1. Silent failure is the default.** Almost every error in a perception-to-motion pipeline produces plausible wrong output rather than an exception. A wrong intrinsic scales the object. A wrong frame convention offsets every grasp by a constant. A wrong TF puts the object somewhere else entirely. *Approach:* verify at every boundary with a property you can check independently — rotation matrices orthonormal with determinant 1, reconstructed extents matching a ruler, grasp fingertips landing inside the object's bounding box, TF echoed and visually confirmed in RViz.

**2. Coordinate frames are where projects go to die.** Camera optical vs camera link, gripper base vs TCP, robot base vs world, and each with its own axis convention. *Approach:* write the frame into the name of every variable, do transforms in one place rather than scattered, and prefer a single composed lookup over manual chaining.

**3. Sensor noise is structured, not Gaussian.** The flying-pixel artefact is one-sided, spatially localised at silhouettes, and correlated with scene geometry. Treating it as zero-mean noise and applying a mean/std filter makes it worse. *Approach:* characterise the noise before filtering it — partition the data by a hypothesis (interior vs boundary) and measure whether the hypothesis explains the difference.

**4. The integration surface is larger than any component.** Four environments, two ROS versions, three GPU models, one physical robot. Most of the difficulty is not in any one part. *Approach:* isolate at the smallest sufficient level, define explicit contracts at process boundaries, and prefer RPC over shared imports when environments conflict.

**5. Hardware has modes, and it will not tell you.** FCI not activated. Gripper needing a homing cycle. A brake not released. *Approach:* when a symptom looks like a software/network problem, check the device's own state first — it is cheap and it is frequently the answer.

**6. You cannot iterate quickly on a real robot.** Access is scarce, shared, and slow; a bad command can damage equipment. *Approach:* build an offline path deliberately. Record raw sensor data to bags, replay it, and structure the system so most of it can be exercised without hardware. Reserve robot time for what genuinely needs the robot.

**7. Reproducing a paper is not running its code.** Papers omit things — sometimes deliberately, as here. The published numbers depend on hardware, model snapshots, and protocols you cannot recreate. *Approach:* separate the paper's *claims* from its *numbers*. Reproduce the claims on your platform, with your own baseline, and state clearly what was not reproduced and why.

**8. Foundation models are confidently wrong.** The detector proposed an e-stop button as a cube at 0.48 confidence. The planner will assert a plan the robot cannot execute. *Approach:* never let a single model output drive an irreversible action unchecked. Gate on confidence, cross-check against geometry, and keep a verification step that can veto.

---

## 6. Skills demonstrated

**Robotics and control**
- ROS 2 (Humble) node development: action clients, service clients, parameters, TF2 lookups and composition
- MoveIt 2 motion planning: pose-goal construction with position/orientation constraints, Cartesian path computation, trajectory execution
- Kinematic reasoning: singularity detection via joint-jump thresholds, approach/retreat strategy, workspace safety
- Gripper control and driver-level fault diagnosis and recovery
- Hand-eye calibration (eye-on-base, ArUco): execution, verification, and understanding of the eye-in-hand/eye-on-base trade-off
- Real hardware: Franka FR3, FCI, safety modes, real-time scheduling configuration

**Computer vision and 3D perception**
- Open-vocabulary detection and segmentation (GroundingDINO + SAM 2)
- Pinhole back-projection, camera intrinsics, depth registration
- **Stereo depth artefact diagnosis and correction** — hypothesis-driven root-cause analysis with quantified before/after
- Robust statistics applied to sensor data: median/MAD over mean/std for one-sided noise, percentile-based bounding volumes
- Point-cloud processing: morphological operations, outlier rejection, scale-aware cropping, downsampling
- 6-DoF grasp synthesis and SE(3) pose conventions

**Machine learning systems**
- VLM integration with **structured output** (Pydantic schemas) across three different provider APIs
- Multi-backend abstraction handling genuinely different API contracts — system-prompt placement, role naming, three different image-payload encodings
- RAG: embedding-based retrieval, cosine similarity, top-$k$ selection, and a measured understanding of why selective retrieval beats full-context
- Understanding of embedding-dimension incompatibility as a real system constraint

**Software and systems engineering**
- Multi-environment dependency management under hard conflicts (`torch 2.1.0` vs `>=2.3.1`), CUDA architecture targeting, build isolation
- Docker and `docker compose`: privileged containers, host networking, IPC, real-time limits, device passthrough — **and the judgement to remove containers where they were not earning their place**
- Inter-process communication: ZMQ/msgpack RPC across environment boundaries
- CLI tool design with diagnostic modes and actionable error messages
- Data engineering: rosbag recording and extraction, reproducible artefact pipelines with seeded sampling

**Research-level**
- Close reading of a recent paper, including reconstructing implementation details from figures that the prose omits
- Distinguishing a paper's *claims* from its *numbers*, and defining what reproduction should mean
- Identifying a published system's weakest measured component and targeting it
- Critical evaluation: recognising sample-size limits, protocol differences between headline results, human-in-the-loop caveats, and shared-model verification risk
- Honest scoping: separating verified from unverified, and documenting the difference

**Engineering judgement — the thread through all of it**
- Choosing a licence-clean, container-compatible grasp model over a machine-locked alternative
- Removing an architectural layer whose only demonstrated effect was creating a bug
- Selecting grasps in a gravity-aligned frame rather than the camera frame, and being able to say why
- Refusing to let parameter tuning mask an upstream fault
- Writing down what is *not* verified

---

## 7. Full project scope and ambition

**The finished system.** A person says *"put the red cup on the plate."* The robot looks, describes what it sees, recalls similar past situations, plans a first action, executes it, looks again to judge whether it worked, and — if it failed — reasons about why, adapts, and tries something different. When it finally succeeds, it writes down what it learned so the next similar task is right the first time. No model is ever retrained.

**Delivered so far**
- Full perception→grasp pipeline, verified on real captured data
- Original depth de-noising with quantified improvement to downstream grasp confidence
- ROS 2 pick execution implemented end to end in code, including singularity guarding and gripper fault recovery
- Hand-eye calibration performed and verified on hardware
- Three interchangeable VLM backends
- A reproducible offline dataset (raw bags plus extracted RGB/depth/intrinsics/masks/clouds/grasps) enabling development without the robot

**Remaining**
- Connecting the reasoning layer to the FR3 executor across the ROS 1/ROS 2 boundary
- First end-to-end pick on hardware
- Place and push
- The memory experiments — STM ablation, LTM/RAG ablation, retrieval ablation
- Two research extensions: local-VLM inference for latency, and ontology-based memory representations

**The ambition beyond the course.** Two threads are genuinely open research, not coursework. First, **memory representation**: the paper's failure analysis shows the model retrieving a correct lesson and then reasoning its way out of using it — which argues for structured, checkable preconditions over free-text advice. Second, **local inference**: at ~10 s per planning call, a cloud VLM in a closed perception-action loop is a latency bottleneck; moving it on-device changes what the system can do.

---

## 8. CV bullet points

Use the shortest set that fits. All are defensible as written.

### One-liner
- Built the full perception-to-motion execution layer (open-vocabulary segmentation → 6-DoF grasp synthesis → ROS 2/MoveIt 2 execution on a Franka FR3) for a reproduction of an IEEE RAL 2026 VLM task-planning paper whose published code omits action execution entirely.

### Short (3 bullets)
- Implemented the complete execution stack for a Franka FR3 reproduction of PragmaBot (IEEE RAL 2026): Grounded-SAM-2 segmentation, point-cloud reconstruction, NVIDIA GraspGen 6-DoF grasp synthesis, ArUco hand-eye calibration, and ROS 2/MoveIt 2 pick execution — the layer the published code leaves unimplemented.
- Diagnosed and fixed a stereo depth artefact corrupting grasp inputs: identified silhouette "flying pixels" as the cause via an interior-vs-boundary outlier analysis, and cut reconstructed object depth-extent from 16.7 cm to 4.5 cm, raising the grasp model's own confidence from 0.77–0.95 to 0.94–0.98.
- Integrated four mutually incompatible GPU/ROS environments (`torch 2.1.0` vs `>=2.3.1`, ROS 1 and ROS 2) using per-tool virtual environments and ZMQ inter-process RPC, after establishing that a container-per-tool architecture added a class of DDS/shared-memory bugs without providing isolation the venvs did not already give.

### Medium (5–6 bullets)
- Reproducing *PragmaBot* (IEEE RAL 2026, ETH Zürich RSL) on a Franka FR3 for a TU Darmstadt research project lab: a vision-language model plans manipulation tasks, visually verifies its own actions, and improves through natural-language self-reflection stored in short- and long-term memory, with no fine-tuning.
- Built the entire execution layer the published code leaves as `NotImplementedError`: open-vocabulary segmentation (GroundingDINO + SAM 2), mask-and-depth back-projection to object point clouds, 6-DoF grasp synthesis via NVIDIA GraspGen, and ROS 2 pick execution against MoveIt 2 and the Franka gripper.
- Diagnosed a stereo-camera "flying pixel" artefact corrupting every grasp input — proved the mask was correct and the boundary *depth* was wrong by comparing far-outlier rates between eroded-interior and boundary pixels (1.5% vs 11.9%) — then corrected it with erosion, a median-absolute-deviation depth filter chosen for the noise's one-sidedness, and a scale-aware bounding-box crop, reducing object depth-extent from 16.7 cm to 4.5 cm and raising downstream grasp confidence to 0.94–0.98.
- Implemented robust pick execution: pre-grasp standoff along the grasp's own approach axis, straight-line Cartesian approach with singularity detection via joint-jump thresholding (aborting rather than executing a truncated path), base-frame vertical retreat, and automatic recovery from a Franka Hand driver fault that required a homing cycle — a symptom that presented as a network/container problem.
- Performed and verified ArUco-based eye-on-base hand-eye calibration (`fr3_link0` ↔ ZED2), and configured the camera driver to stop publishing its own TF/odometry so a single calibrated transform owns the tree.
- Built a three-backend VLM abstraction (OpenAI / Anthropic / Google) with structured Pydantic output, handling differing system-prompt placement, role naming, and image-payload encodings — and identified embedding-dimension incompatibility across backends as a hard constraint on memory portability.

### Research-flavoured variant (for MSc/PhD applications)
- Analysed and reproduced *PragmaBot* (IEEE RAL 2026), which adapts verbal reinforcement learning — improving a frozen VLM through in-context self-reflection rather than weight updates — to real-world robot task planning, with retrieval-augmented long-term memory.
- Identified that the paper's largest measured failure category (8 of 19 first-attempt failures) lies in the execution layer it does not contribute, and targeted that layer: a stereo depth-artefact correction that measurably improved the grasp model's own confidence on real captures (0.77–0.95 → 0.94–0.98).
- Reconstructed implementation details absent from the paper's prose by analysing its figures, including the exact prompt structures and the finding that its image-annotation module provides zero benefit on geometrically simple objects (100%/100% on box and mug) while being decisive on shape-critical ones (100% vs 20% on a skewer) — the empirical justification for gating it on-demand.
- Scoped two research extensions: structured/ontology-based memory representations, motivated by the paper's finding that the model retrieved a correct memory and then dismissed it in 5 of 19 failures; and local VLM inference against a ~10 s per-call latency baseline.

---

## 9. Cover-letter paragraphs

### A. Full-stack robotics emphasis
> My current project is a reproduction of *PragmaBot*, an IEEE RAL 2026 paper from ETH Zürich's Robotic Systems Lab, on a Franka FR3 at TU Darmstadt. The paper's contribution is a reasoning layer — a vision-language model that plans manipulation tasks, visually checks whether its own actions worked, and improves by writing natural-language reflections into short- and long-term memory without any fine-tuning. Its published code deliberately leaves action execution unimplemented. Building that layer has been the project: open-vocabulary segmentation, reconstructing object point clouds from mask and depth, 6-DoF grasp synthesis, hand-eye calibration, and ROS 2 motion execution against MoveIt 2. What I have valued most is that almost every failure in this stack is silent — a wrong frame convention or a corrupted depth edge produces a plausible-looking pose that puts the gripper somewhere wrong — so the work is really about building verification into every boundary.

### B. Debugging and root-cause emphasis
> The most instructive problem I have solved recently looked like a segmentation failure and was not. Object point clouds reconstructed from our stereo camera were badly elongated — a cube with a 4 cm depth extent was reconstructing at 16.7 cm — and the natural conclusion was that the segmentation mask was leaking onto the background. Instead of tuning thresholds, I partitioned the mask into its eroded interior and its boundary ring and compared depth statistics between them. The interior was a compact object with 1.7 cm depth variation; the boundary pixels were several times more likely to be far outliers. The mask was correct; the *depth values at silhouette pixels* were wrong — a stereo matcher interpolating between the near object and the far background. Knowing the artefact was one-sided told me to use a median-absolute-deviation filter rather than mean and standard deviation, which the outliers would have dragged along with them. The correction cut depth extent to 4.5 cm and raised the downstream grasp model's own confidence from 0.77–0.95 to 0.94–0.98 — which was the result I actually cared about, because it meant the fix improved the input rather than just the picture.

### C. ML-systems / VLM emphasis
> I work on the boundary between foundation models and physical systems. My current project reproduces a paper in which a frozen vision-language model improves at robot task planning purely through in-context learning — it self-critiques its failures in natural language, stores those reflections, and retrieves the relevant ones by semantic similarity when it meets a similar situation. One result from it has changed how I build retrieval systems generally: giving the planner its entire hundred-entry memory performed *worse* than retrieving the five most relevant entries — 74% against 89% first-action accuracy — while also costing 7.5 times the prompt tokens. The full-memory condition always contained the right lesson and still lost, because the irrelevant material degraded the model's attention. Relevance filtering turns out to be an accuracy mechanism, not a cost optimisation. On the implementation side I built a three-provider backend abstraction with structured schema output, which surfaced a constraint that is easy to miss: because each provider's embeddings have different dimensionality, the memory store is not portable across backends at all.

### D. Systems-architecture emphasis
> A decision I am glad I made on my current project was to remove Docker rather than add it. The system runs four environments that genuinely cannot be merged — two GPU models with conflicting pinned PyTorch versions, a ROS 2 control stack, and a ROS 1 reasoning layer — and the original plan was a container per tool. In practice the isolation we actually needed was at the Python-environment level, which virtual environments already provided, and the container boundary introduced a shared-memory and DDS permissions problem between root and non-root processes that cost a full day to trace. I moved everything host-native with per-tool virtual environments and kept only robot control containerised, where a pinned ROS distribution with real-time scheduling privileges genuinely earns the boundary. Cross-environment communication happens over a ZMQ RPC boundary instead. Removing the layer eliminated an entire bug class rather than patching one symptom, and it is the kind of trade-off I now look for.

### E. Research-track emphasis (MSc thesis / PhD)
> Reproducing a recent paper taught me more about research than reading a hundred would. *PragmaBot* (IEEE RAL 2026) shows that a robot can improve its task planning through natural-language self-reflection and retrieval over past experience, with no weight updates. Working through it closely, two things stood out. First, the paper's own failure analysis attributes its single largest category of failures — 8 of 19 — to the execution layer it does not contribute, which means the reported success rates are partly a function of manipulation quality rather than of the memory mechanism alone; that made improving the perception stack a legitimate contribution rather than setup work. Second, its most interesting failure is one it cannot fix within its own design: in 5 of 19 cases the system retrieved the correct past lesson and then reasoned its way out of applying it, judging the current scene sufficiently different. Retrieval succeeded and integration failed. That failure is what motivates the direction I want to pursue — representing experience as structured preconditions a planner must explicitly discharge, rather than as advisory prose it is free to dismiss.

---

## 10. Interview talking points

### Q: "Walk me through this project."
Give the layered answer, ~90 seconds:
> It's a reproduction of a 2026 RAL paper from ETH's Robotic Systems Lab on a real Franka FR3. The paper's idea is that a vision-language model can get better at planning manipulation tasks without being fine-tuned — it plans an action, executes it, looks at before-and-after images to judge whether it worked, and when it fails it writes a natural-language critique of its own failure. That critique goes into a short-term memory that shapes the rest of the task, and when the task succeeds the whole episode is distilled into a long-term memory retrieved by similarity later.
>
> The important thing for me is that the published code implements all the reasoning and deliberately leaves action execution as a `NotImplementedError`. So reproducing it meant building the entire body: turning a noun phrase into a pixel mask with open-vocabulary segmentation, turning that mask plus depth into a metrically correct point cloud, generating 6-DoF grasps, calibrating the camera to the robot base, and executing motion through MoveIt 2 with the Franka gripper.
>
> The part I'd most want to talk about is a stereo depth artefact I traced and fixed, because it turned out to attack the exact weakness the paper's own failure analysis identifies.

### Q: "What was the hardest technical problem?"
The flying-pixel diagnosis (§4.3). Structure it as: symptom → the obvious wrong hypothesis → the experiment that discriminated between hypotheses → why the fix follows from the diagnosis (one-sided noise ⇒ MAD not std) → the measured result, ending on the *downstream confidence* number rather than the geometry number.

The key sentence: **"The mask was right; the depth at the mask's edge was wrong."**

### Q: "How do you know your fix actually worked?"
> Three ways. The geometry got right — the reconstructed depth extent went from 16.7 cm to about 4.5 cm on an object whose true depth is 4–5 cm, so it now matches a ruler. The root cause was confirmed independently by comparing far-outlier rates between interior and boundary pixels of the same mask, which differed by several times over. And most importantly, the grasp model's own confidence in its output rose from 0.77–0.95 to 0.94–0.98 — that one matters because it's a downstream model that doesn't know anything about my filter, so it's independent evidence that the input got better rather than just prettier.

### Q: "Why MAD instead of standard deviation?"
> Because the noise is one-sided. Flying pixels always trail *away* from the camera toward the background, never toward it. A mean-and-standard-deviation filter estimates its own centre and spread from data that includes those outliers, so the outliers drag the mean toward themselves and inflate the standard deviation — the filter is corrupted by exactly what it's supposed to remove. The median and MAD have a breakdown point of 50%, so they're unaffected until outliers dominate. I scale MAD by 1.4826 so the threshold is interpretable on the same scale as a standard deviation would be for normal data.

### Q: "You have three filters. Isn't that over-engineering?"
> They handle genuinely different failure modes, and the tool can prove it — there's a diagnostic mode that reports what each filter would remove independently, so you can see which one is doing the work on a given scene. Erosion removes the boundary ring. The MAD filter is depth-only, so it can't touch mask leakage sideways onto the table — those points have a perfectly normal depth. That's what the bounding-box crop is for, and it matters specifically because the grasp model re-centres its input on the cloud mean, so a centroid pulled sideways off the object shifts every grasp it generates. On one of my scenes erosion does essentially all the work and MAD removes nothing — which is exactly why the diagnostic mode exists rather than me assuming.

### Q: "What's the trickiest bug you've hit?"
Offer the Franka Hand homing fault (§4.8):
> The gripper would go unresponsive after a grasp, and the robot's own web interface said "End Effector: Not connected." Everything about that says network or Docker. It wasn't. The Franka Hand driver needs a homing call after connecting, and after any grasp or move fault, before it responds reliably again. The lesson I took is that when a symptom looks like infrastructure, check the device's own state machine first — it's cheap and it's often the answer. I put the fix in the code as an automatic home at the start of every pick plus one re-home-and-retry on failure, and wrote the diagnosis into the docstring so nobody re-derives it.

### Q: "Why GraspGen and not AnyGrasp, which the paper used?"
> Two reasons, and one of them is that the authors themselves recommend it. The paper's experiments used AnyGrasp, but the released repository's README points users at GraspGen. Independently, AnyGrasp's licence is machine-locked, which breaks in a containerised or multi-machine setup, and GraspGen ships open weights with a pretrained model for the Franka gripper, which is exactly our hardware, plus a client-server mode that lets it live in its own CUDA environment. So it was the licence-clean and architecturally cleaner choice, and it happened to match the authors' own guidance.

### Q: "How do you pick which grasp to execute?"
This is a good one to answer *with the subtlety*, because the naive answer is obvious and the real answer shows understanding:
> Right now, the most top-down one — on a tabletop that's least likely to collide with the table or approach through the object. The subtlety is *which frame you decide that in*. "Top-down" is meaningless in the camera frame, because the camera's tilt is arbitrary; it only means something in a gravity-aligned frame. So I transform all the candidates into the robot base frame first with a single TF lookup, then compare each grasp's approach axis against vertical. There was an alternative suggestion to filter by approach angle inside the grasp client, but that operates in camera frame and I'd have to guess which rotation column is the approach axis, so the robot-frame version is both more correct and easier to reason about.
>
> What I haven't implemented yet — and it's the paper's approach — is combining geometric confidence with a *semantic* score, where the VLM picks where on the object to grasp and grasps near that point score higher. Mine is purely kinematic. That's a known gap.

### Q: "Tell me about a design decision you'd defend."
Removing Docker (§4.9). The shape: what the original plan was → why the isolation requirement was actually at a different level → the concrete cost (a full day lost to a bug that only existed because of the boundary) → what was kept containerised and why that one earns it.

### Q: "What would you do differently?"
Be specific and non-defensive:
> I'd have connected the reasoning layer to the executor earlier. I built the perception and motion stack thoroughly and verified it on real captures, but the interface between the planner and the robot is still the missing piece, and it's the thing that gates the actual end-to-end demonstration. I also found, reading the code carefully, that the older ROS 1 executor reads fields that don't exist on the planner's output schema — it would raise an `AttributeError` on its first real call. It was never caught because the system runs in a replay mode that skips execution entirely, so that code path had literally never executed. The lesson is that a code path guarded by a config flag nobody flips is untested code, and I should have written a schema round-trip test at the boundary.

### Q: "How would you evaluate whether the memory actually helps?"
> Three experiments, and the useful thing is that none of them needs the robot. The system has a replay mode that skips execution and calls the success detector directly, and the paper's retrieval ablation measures first-planned-action accuracy without executing anything at all. So: first, short-term memory on versus off on scenarios with a built-in first-attempt failure — does reflection convert repeated identical failures into an adapted plan. Second, empty long-term memory versus populated, single-trial, including structurally similar but unseen scenarios so you're measuring transfer rather than memorisation. Third, the retrieval ablation — top-k versus entire memory versus random. I'd report trial counts explicitly, because the original paper runs 5–10 trials per task, which means a single trial is worth 10 to 20 percentage points and no result there should be quoted without n.

### Q: "What are the limitations of this paper?"
Show you read critically without being dismissive:
> A few. The trial counts are small — 5 to 10 per task, no confidence intervals — so the direction is convincing but nothing is statistically established. The long-term memory has 100 entries and only 4 came from autonomous experience; the other 96 were seeded instructional examples, so the mechanism is demonstrated but the memory that produces the headline number is largely curated. The two headline numbers aren't the same measurement either — the 84% allows two attempts, the 80% is single-trial against a different baseline. A human operator may reset the scene after a destructive failure, so those aren't fully autonomous episode rates. And the same model both plans and grades, which is what makes it genuine self-reflection but also means correlated blind spots, and they only measure detector error on picking.
>
> None of that makes it a bad paper — it's a real system on real hardware, which is expensive, and they're transparent about most of it. It just changes what you can claim it establishes.

### Q: "What's the most useful thing you learned that transfers beyond robotics?"
> That in a retrieval system, filtering for relevance is an accuracy mechanism and not just a cost optimisation. The paper measures it directly: giving the planner all hundred memories scored 74% on first-action accuracy, while retrieving the five most relevant scored 89%. The full-memory condition *always* contains the right lesson — it can't miss it — and it still loses, because the irrelevant material degrades the model's attention. And it costs 7.5 times the tokens. The instinct to just put everything in the context window is both more expensive and less accurate, and I'd now push back on it with numbers.

### Q: "Have you run this on the real robot?"
Answer this one straight — it is the question most likely to catch someone out:
> The perception and grasp pipeline, yes — it's verified against real ZED captures on the actual FR3 setup, and the hand-eye calibration was done and validated on the hardware. The pick execution is written and its components are smoke-tested, but it has not yet run end to end on the robot; I've been working from recorded data because I don't currently have lab access. The next hardware session starts with confirming the TF chain and resolving which link MoveIt treats as the end effector, because GraspGen's poses are relative to the gripper base rather than the fingertips and getting that wrong offsets every grasp by about ten centimetres.

That answer is stronger than a vague "yes." It shows you know exactly what is verified, what is not, and what the first risk is.

---

## 11. Claim discipline — what not to say

The difference between a strong project and an over-claimed one is a handful of sentences. Keep these straight.

| ❌ Do not say | ✅ Say instead |
|---|---|
| "I built a robot that learns from its mistakes." | "I built the execution layer for a system that learns from its mistakes; the reasoning layer is from the paper." |
| "I ran the full pipeline on the FR3." | "Perception and grasp generation are verified on real captures; pick execution is implemented but not yet run end to end on hardware." |
| "I implemented hand-eye calibration." | "I performed and verified an ArUco eye-on-base calibration using `easy_handeye2`" — the package is third-party and unmodified. |
| "I reproduced the paper's results." | "I'm reproducing the paper's claims on our platform; matching their numbers isn't achievable with different hardware, a different grasp model, and a different VLM snapshot." |
| "I improved the paper's success rate." | "I improved the input quality to the grasp model — measured by its own confidence — which targets the paper's largest measured failure category." |
| "I wrote GraspGen / Grounded-SAM-2 integration from scratch." | "I integrated them, and contributed a grasp-persistence extension upstream-style plus the point-cloud conditioning that makes their output usable." |
| "The memory system works on our robot." | "The memory layer runs; it has not yet been evaluated on our platform — that's the next phase, and most of it doesn't require the robot." |

**Two numbers to always attach a caveat to:**
- *16.7 cm → 4.5 cm* — say which scene. A second scene gave 17.7 cm → 7.3 cm. Both are real; quoting only the best one is the kind of thing that unravels.
- *0.77–0.95 → 0.94–0.98* — this is the grasp model's **self-reported confidence**, not a measured grasp success rate on hardware. Say "the model's own confidence," every time. It is still a meaningful, independent signal, and stating it precisely costs you nothing.

**The general principle.** This project is strong because of the depth of the debugging and the quality of the reasoning, not because of a completion percentage. An interviewer who hears "I diagnosed a stereo artefact by partitioning the mask and comparing outlier rates, and I know exactly what I haven't validated yet" will rate you higher than one who hears an unqualified "it works." Precision reads as competence.
