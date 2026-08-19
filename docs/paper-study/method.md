# Method — PragmaBot

Component breakdown, algorithm flow, prompt structure, and the implementation details that decide whether a reproduction works.

---

## 1. System architecture

```
                            ┌──────────────────────────┐
   user instruction I ─────▶│  VLM Scene Describer  D  │
        o₀ (first RGB) ────▶└────────────┬─────────────┘
                                         │ initial scene description
                                         ▼
                            K′ = (I, D(o₀))   ── the retrieval key
                                         │
                                         ▼
                            ┌──────────────────────────┐
                            │  RAG over LTM  (top-k    │◀──── M  (long-term memory)
                            │  cosine similarity)      │      key → distilled lesson
                            └────────────┬─────────────┘
                                         │ {(Kᵢ, Eᵢ)}
                                         ▼
   oₜ (current RGB) ───────▶┌──────────────────────────┐
   m  (short-term memory) ─▶│  VLM Task Planner  P     │
                            └────────────┬─────────────┘
                                         │ aₜ = NextBestAction
                                         ▼
                            ┌──────────────────────────┐
                            │  Skill execution         │  ◀── pick / place / push
                            │  *** NotImplementedError │      + on-demand annotation
                            └────────────┬─────────────┘
                                         │ oₜ₊₁
                                         ▼
                            ┌──────────────────────────┐
                            │  VLM Success Detector R  │  (oₜ, aₜ, oₜ₊₁)
                            └────────────┬─────────────┘
                                         │ rₜ₊₁ = (successful?, complete?, scene delta)
                          ┌──────────────┴───────────────┐
                     not complete                    complete
                          │                              │
                          ▼                              ▼
                 m ← m ∪ (aₜ, rₜ₊₁)          ┌──────────────────────────┐
                 loop back to P              │ VLM Experience Summarizer│
                                             └────────────┬─────────────┘
                                                          │ S(m)
                                                          ▼
                                              M ← M ∪ {(K′, S(m))}
```

One VLM instance fills four roles. The only non-VLM components are the text embedding model and the skill layer.

## 2. Algorithm 1, annotated

```
Given: instruction I, initial observation o₀
Internal: long-term memory M

1: K′ ← (I, D(o₀))                                    build retrieval key ONCE per task
2: {(Kᵢ,Eᵢ)}ᵏ⁻¹ᵢ₌₀ ← arg top-k cos(E(K), E(K′))       retrieve ONCE per task
3: t ← 0, m ← ∅                                        STM starts empty every task
4: repeat
5:   aₜ ← P(I, oₜ, m, {(Kᵢ,Eᵢ)})                       plan (STM + LTM both in prompt)
6:   execute aₜ, receive oₜ₊₁                          <-- the integration point
7:   rₜ₊₁ ← R(oₜ, aₜ, oₜ₊₁)                            visual before/after comparison
8:   m ← m ∪ (aₜ, rₜ₊₁)                                grow STM
9:   t ← t + 1
10: until rₜ.completed                                 termination comes from R, not P
11: M ← M ∪ {(K′, S(m))}                               distil episode into LTM
```

Details that are easy to get wrong:

- **Lines 1–2 run once per task, not per step.** The scene description and the retrieval both use the *initial* observation. Re-retrieving every step would be a different (and more expensive) algorithm.
- **Line 10: the planner does not declare completion.** The success detector's `is_task_completed` terminates the loop. There is no `done` skill.
- **Line 11 executes only on success.** Failed episodes are not written to LTM.
- **`m` is reset at line 3 every task.** STM never crosses task boundaries; that is exactly what LTM is for.

## 3. Component specifications

### 3.1 Task Planner P

Prompt (Fig. 3), in order:
```
Instruction: <Instruction>
Robot's current observation: <IMG>
Here is the action history: <STM>
List of planning rules and constraints.
Here are the past experiences: <LTM>
Choose the next best action.
```

**The ordering is load-bearing.** STM sits right after the observation; the constraints sit in the middle; retrieved LTM sits *last*, immediately before the instruction to decide. Retrieved experience is placed closest to the decision point.

Structured output schema (from the released implementation):

```python
class RobotSkill(str, Enum):
    PUSH = "push"; PICK = "pick"; PLACE = "place"      # note: no "done"

class PushDirection(str, Enum):
    LEFT = "left"; RIGHT = "right"

class NextBestAction(BaseModel):
    scene_description: str
    applicable_knowledge: Optional[str]                 # which retrieved lesson applies
    chain_of_thought_reasoning: str                     # MUST precede the decision
    chosen_action: str                                  # free text the detector will grade
    chosen_skill: RobotSkill
    target_object: str
    should_grasp_at_specific_section: Optional[bool]    # on-demand annotation, pick
    placement_object: Optional[str]                     # place ON, not next to
    should_place_at_specific_section: Optional[bool]    # on-demand annotation, place
    push_direction: Optional[PushDirection]
```

Three design choices worth copying:

1. **Reasoning fields come before decision fields.** With structured decoding the model emits them in order, so it literally cannot commit before reasoning.
2. **`applicable_knowledge` forces explicit use of retrieval.** The model must name which past scenario is similar and what lesson transfers. This makes RAG use *auditable* — and it is exactly where the paper's "VLM overrides experience" failure becomes visible in the log.
3. **`chosen_action` is free text that becomes the success detector's input.** The two VLM roles communicate in natural language, not through a shared symbolic representation.

Planning constraints are stated in the prompt, not enforced in code — e.g. "when the target object is tiny or flat, which is hard to grasp, you cannot use PICK," and an explicit procedure: *propose an action, check each constraint one by one, discard and re-propose if violated, repeat.*

### 3.2 Success Detector R

```
Instruction: <Instruction>
Action the robot just attempted: <Action>
Observation before the action: <IMG>
Observation after the action: <IMG>
List of success/failure criteria and reasoning rules.
Output your structured evaluation.
```

```python
class SuccessEvaluation(BaseModel):
    scene_description: str
    is_action_successful: bool
    is_task_completed: bool
```

Two binary signals plus a semantic description of what changed. Measured reliability on picking: **6.67% false negative, 5% false positive**. Most false positives: object still on the table but visually enclosed by the gripper — a viewpoint artefact.

### 3.3 Experience Summarizer S

```
Instruction: <Instruction>
Scene: <Initial scene description>
Summarize the robot's experiences: <STM>
```

Compresses a whole episode into one reusable lesson. Runs only on success.

### 3.4 Memory

**STM** — `m = {(a_τ, r_{τ+1})}` — appended each step, injected whole into the planner prompt, reset per task.

**LTM** — key–value, persisted as two CSVs merged on the `scenario` column:
- `ltm.csv` — `time`, `scenario`, `experience` (human-readable)
- `ltm_<embedding_model>.csv` — `scenario`, `embedding` (base64)

Scenario key format:
```python
f"Instruction: {instruction}\nScene: {initial_scene_description}"
```

Retrieved entries are handed to the planner as JSON strings: `{"scenario": ..., "experience": ...}`.

> **Implementation trap:** because the embeddings file is named after the embedding model and joined on `scenario`, **LTM is not portable across embedding models.** Different backends produce different dimensionalities (3072-D OpenAI, 768-D Gemini, 384-D a local MiniLM fallback). Switching backends requires regenerating embeddings or maintaining parallel files. Any cross-backend comparison must control for this.

### 3.5 Retrieval

$$\{(K_i,E_i)\}_{i=0}^{k-1} \leftarrow \arg\text{top-}k \; \frac{\mathcal{E}(K)^\top \mathcal{E}(K')}{\lVert\mathcal{E}(K)\rVert\lVert\mathcal{E}(K')\rVert}$$

Default $k = 5$. The random-retrieval ablation is implemented as a full shuffle with similarities forced to zero.

## 4. The skill layer and annotation

```
VLM picks skill + parameters, and whether annotation is needed
        │
        ▼
Grounded SAM  (GroundingDINO text→box, SAM box→mask)  →  object mask
        │
        ├── annotation NOT requested ──▶ act on the mask directly
        │
        └── annotation requested
                 │
                 ├── pick  : FPS within the object mask     → numbered candidates
                 ├── place : FPS on the segmented mask      → numbered candidates
                 └── push  : candidate goal-region masks    → numbered candidates
                                    │
                                    ▼
                        VLM selects a numbered option
```

For grasping:
1. Segmented point cloud → AnyGrasp → grasp hypotheses with $s_{\text{conf}} \in [0,1]$
2. IK feasibility filter (Pinocchio) → feasible set $G$
3. Final selection:
$$g^* = \arg\max_{g \in G} \; s_{\text{conf}}(g)\cdot s_{\text{loc}}(g)$$
where $s_{\text{loc}}$ derives from normalised Euclidean distance from the grasp to the VLM-chosen location.

**When annotation actually matters** (measured, 7 objects): box 100/100, mug 100/100, banana 80/80 — *no gain*. Drumstick 80/40, ice cream 80/40, brush 80/20, skewer **100/20** — large gain. The benefit is entirely on objects with a functionally correct grasp *section*. This is why the module is gated on a model-emitted boolean rather than always on: it costs ~5.15 s per call.

## 5. Pseudocode for a faithful reimplementation

```python
def run_task(instruction, robot, vlm, memory, cfg):
    o = robot.observe()
    scene_desc = vlm.describe_scene(instruction, o)
    key = f"Instruction: {instruction}\nScene: {scene_desc}"

    ltm = memory.retrieve(key, top_k=cfg.retrieval_top_k) if cfg.activate_ltm else []
    stm = []

    while True:
        action = vlm.plan(instruction, o, stm, ltm)      # NextBestAction
        if cfg.activate_stm:
            stm.append({"action": action})

        o_before = o
        if cfg.rosbag_replay:
            pass                                          # skip execution entirely
        else:
            robot.execute(action)                         # <-- YOUR CODE
        o = robot.observe()

        evaluation = vlm.detect_success(
            instruction, action.chosen_action, o_before, o)
        if cfg.activate_stm:
            stm.append({"evaluation": evaluation})

        if evaluation.is_task_completed:
            break
        if not evaluation.is_action_successful:
            stm.append({"additional_info":
                        "The human operator might have reset the scene."})

    if cfg.save_to_ltm:
        memory.save(key, vlm.summarize(instruction, scene_desc, stm))
```

Note `rosbag_replay`: when true, execution is skipped and the success detector is called on two observations taken without any robot motion. **This is how the entire memory pipeline can be developed and tested without a robot.**

## 6. Implementation pitfalls

| Pitfall | Consequence | Guard |
|---|---|---|
| Reasoning fields placed after decision fields in the schema | Model rationalises instead of reasoning; STM quality collapses | Keep `chain_of_thought_reasoning` before `chosen_*` |
| Re-running retrieval every step | Different algorithm, higher cost, unstable context | Retrieve once per task from $o_0$ |
| Mixing embedding models in one LTM | Silent dimension mismatch or meaningless similarities | Model-specific embeddings file; regenerate on switch |
| Writing failed episodes to LTM | Memory fills with strategies that did not work | Save only on `is_task_completed` |
| Letting the planner declare completion | Planner is optimistic; loop terminates early | Termination comes from the detector |
| Dumping all memory into context | −15 points accuracy, 7.5× tokens | Top-$k$ retrieval |
| Always-on annotation | +5.15 s per call for zero gain on simple objects | Gate on the model's own boolean |
| Assuming an executor field name | Silent empty goals or `AttributeError` | Read the actual schema, not the docs |

## 7. Hyperparameter sensitivity

- **`retrieval_top_k`** — the most consequential. $k = 5$ default; $k = \infty$ (all) drops accuracy 89% → 74%; random $k=5$ → 17%. So both *what* you retrieve and *how much* matter, and the failure modes differ: random gives irrelevance, all gives distraction.
- **VLM size** — `gpt-4o-mini` is more conservative, which *helps* without memory and gains less from it. Roughly 3 s faster per call.
- **Success-detector strictness** — not ablated in the paper, but structurally critical: the detector gates both replanning and LTM writes.
- **Annotation gating** — model-decided; not swept.

## 8. Reproduction risks

**High**
- **Action execution is not provided.** `NotImplementedError` in the released node. Everything below the planner — segmentation, grasp synthesis, motion planning, gripper control — must be built for your own robot. Reported success rates are inseparable from an execution stack the paper only partially describes.
- **Different embodiment changes the lessons.** LTM entries encode *this robot's* limitations. A fixed-base FR3 with a static camera will learn different lessons than an ANYmal with an elbow-mounted camera. Transferring their LTM verbatim is not meaningful.
- **AnyGrasp licensing.** Machine-locked; problematic in containers. The released README recommends **GraspGen** instead — a substitution the authors themselves endorse.

**Medium**
- **VLM version drift.** `gpt-4o` behaviour changes across snapshots; results are not pinned to a frozen model.
- **Camera geometry.** Eye-in-hand (theirs) vs eye-on-base (a typical arm setup) changes occlusion patterns, the meaning of "left/right" in prompts, and calibration requirements.
- **Human scene resets** are part of the protocol and must be reported if reproduced.

**Low**
- Prompt templates are published only in skeleton form in Fig. 3; full examples are on the project webpage.
- Small trial counts mean reproduction variance will be high; ±1 success out of 7 trials is 14 percentage points.
