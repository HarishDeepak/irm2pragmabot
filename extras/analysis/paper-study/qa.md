# Q&A — PragmaBot

15 questions: 5 basic, 5 intermediate, 5 advanced. Answers hidden — try to answer before expanding.

---

## Basic

### 1. What problem is PragmaBot solving, in one sentence?

<details>
<summary>Answer</summary>

The **embodiment gap**: a VLM trained on internet data holds human intuitions about what is easy, and those intuitions do not match a specific robot's actual gripper, perception, and failure modes. PragmaBot aligns the frozen VLM to the robot's real capabilities using its own experience, rather than fine-tuning.

The paper's motivating example: a VLM will confidently instruct "pick up the tennis ball" even when it is half-occluded by a fan, because for a human that is trivial. The robot fails.

</details>

---

### 2. What is the difference between STM and LTM here?

<details>
<summary>Answer</summary>

| | STM | LTM |
|---|---|---|
| Scope | one task/episode | across all tasks, forever |
| Contents | `(action, feedback)` pairs as they happen | distilled lessons, one per completed task |
| Lifetime | **reset when the task ends** | persistent on disk (CSV) |
| Access | injected whole into every planning prompt | retrieved top-$k$ by similarity, once per task |
| Purpose | adapt *within* a task after failure | start a *new* task already knowing something |

STM is a scratchpad; LTM is your notes between exams.

</details>

---

### 3. Is any model being trained or fine-tuned?

<details>
<summary>Answer</summary>

**No.** Not a single parameter is updated anywhere in the system. The VLM is frozen, the embedding model is frozen. All "learning" happens by adding text to the model's context window.

This is the point of **verbal reinforcement learning** — the improvement mechanism is in-context, so it is instant, cheap, inspectable, and requires no training data or GPUs. It also means the "learning" is a text file: copy it to another robot with a different gripper and the lessons may be actively wrong.

</details>

---

### 4. What are the three skills, and what is the fourth thing the planner can output?

<details>
<summary>Answer</summary>

The skills are **pick**, **place**, and **push**.

There is **no fourth option** — and this is a common misreading. `RobotSkill` in the released code is exactly `{push, pick, place}`; there is no `done`. **Task completion is decided by the success detector**, which returns `is_task_completed`, not by the planner. The loop terminates on the detector's signal.

</details>

---

### 5. What are the two headline numbers, and are they measuring the same thing?

<details>
<summary>Answer</summary>

- **35% → 84%** — effect of short-term memory / self-reflection, 4 tasks (Table II)
- **22% → 80%** — effect of long-term memory + RAG, 12 scenarios (Table III)

**They are not comparable.** Table II allows **two attempts** per trial and measures whether the task eventually succeeds. Table III is **single-trial** — did it get it right the first time. Different baselines too: CaP-V for Table II, COME for Table III.

Quoting "35→84 and 22→80" as if they were one progression is wrong.

</details>

---

## Intermediate

### 6. What exactly is the LTM retrieval key, and why is it built that way?

<details>
<summary>Answer</summary>

```python
f"Instruction: {instruction}\nScene: {initial_scene_description}"
```

Instruction plus a **VLM-generated natural-language description of the initial scene**, embedded into a dense vector.

It is built this way because it makes retrieval **semantic rather than geometric**. Two scenes with completely different objects but the same *structure* — "target is blocked by a nearby object" — produce similar embeddings and retrieve each other's lessons. This is the entire generalization mechanism: experience from "push the can away, then grab the apple" transfers to "pick up the milk carton (apple leaning on it)" without any explicit task-similarity model.

Note the description is generated **once**, from $o_0$, not per step.

</details>

---

### 7. Why does providing the entire LTM perform *worse* than retrieving five entries?

<details>
<summary>Answer</summary>

Measured: **89% (RAG top-k)** vs **74% (entire LTM)** vs **17% (random k=5)**.

The full-LTM condition *always* contains the correct lesson — it cannot miss. It still loses, because it also carries ~99 irrelevant entries, and long, noisy contexts degrade LLM focus (the paper cites [38], [39]). The irrelevant material actively distracts.

Two independent costs, then:
1. **Accuracy** — 15 points worse.
2. **Money and latency** — 7.5× the prompt tokens, excluding images.

The generalizable lesson: **relevance filtering in a RAG system is an accuracy mechanism, not just a cost optimization.**

</details>

---

### 8. What is the "linguistic gradient" and how far does the RL analogy actually hold?

<details>
<summary>Answer</summary>

When an action fails, the planner writes a natural-language critique — "previous attempts to pick up the apple directly have failed... I will push the can to the right" — which enters STM and shapes the next decision. The paper calls this a *linguistic gradient*: it plays the role a gradient plays in RL, but the update lands in the context, not the weights.

**Where the analogy holds:** frozen policy ↔ planner; reward ↔ success detector; replay buffer ↔ LTM; policy improvement ↔ better prompt.

**Where it breaks:**
- No value function and **no credit assignment over time** — reflection acts on the immediately preceding failure, not on a discounted return over a trajectory.
- No exploration policy; no notion of on/off-policy.
- The "update" is not a step in any metric space, so nothing converges in any formal sense.

It is a useful and honest metaphor, not a formal equivalence.

</details>

---

### 9. Why is the image annotation module *on-demand* rather than always on?

<details>
<summary>Answer</summary>

Because for many objects it buys nothing and costs 5.15 s.

Reading Fig. 8's per-object bars: box **100 vs 100**, mug **100 vs 100**, banana **80 vs 80** — annotation changes nothing. But skewer **100 vs 20**, brush **80 vs 20**, drumstick **80 vs 40**, ice cream **80 vs 40**.

The gain is entirely on objects where a *specific section* is the correct grasp target — the stick of a skewer, the handle of a brush. For a simple convex object, geometry alone already picks a fine grasp.

So the VLM emits its own boolean (`should_grasp_at_specific_section` / `should_place_at_specific_section`) deciding whether it needs the help. A small piece of metacognition that converts a fixed latency cost into a conditional one.

</details>

---

### 10. Which single component's reliability is everything else built on, and how good is it?

<details>
<summary>Answer</summary>

The **VLM success detector**. The paper states outright that the emergent adaptive behaviours "would not be possible without the high accuracy of the VLM's success detection."

Measured on picking: **6.67% false negative (4/60)**, **5% false positive (3/60)**.

It is load-bearing because it gates *both* feedback loops: it decides whether the planner must reflect and replan, and it decides whether an episode is written into LTM. If it were, say, 25% wrong, STM would fill with noise, reflection would act on false premises, and LTM would accumulate lessons from episodes that never actually succeeded.

Most false positives happen when the object is still on the table but *looks* enclosed by the gripper — a pure viewpoint artefact, which suggests a second camera angle would help.

</details>

---

## Advanced

### 11. The success detector and the planner are the same model. What is the risk, and how well does the paper address it?

<details>
<summary>Answer</summary>

The risk is **correlated blind spots**. If the model misperceives a scene when planning, it may misperceive it identically when grading — so the error is invisible to the loop and can be written into LTM as a "successful" strategy.

The paper does *not* really address this. It measures detector error on picking only (5% FP / 6.67% FN) and frames single-model use as a **feature**, listing "unified visual feedback" in Table I as a property distinguishing it from prior work — the argument being that one model both planning and verifying is what makes it genuine *self*-reflection rather than one model grading another.

Both things are true simultaneously: it is what makes the reflection self-contained, and it is a systematic risk that is only partially measured. An independent verifier (different model, or a second viewpoint) would break the correlation but weaken the "self-reflection" framing.

</details>

---

### 12. 19 of 91 trials failed on the first attempt. Where do the failures come from, and what does that imply for anyone reproducing this?

<details>
<summary>Answer</summary>

```
Failure (19)
├── Execution (8)  ── poor grasp generation (5), inaccurate depth (3)
├── VLM (7)        ── overrides experience (5), false detection (1), wrong mask (1)
└── RAG (4)        ── retrieval failed (4)
```

**The largest bucket — 8 of 19 — is execution**, i.e. perception and grasping. And that is exactly the layer the paper does *not* contribute; it is left as `NotImplementedError` in the released code.

Two implications:

1. **Execution quality is an uncontrolled confound in the headline numbers.** A better manipulation stack would raise the reported success rates without any change to the memory mechanism. So the memory contribution is somewhat entangled with skill quality.
2. **For a reproducer this is good news.** Improving the perception/grasp layer is a legitimate, measurable contribution on top of the paper, not merely "setup work." §V-E even notes that annotation failures are "mostly due to inaccurate 3D point clouds."

</details>

---

### 13. What is "VLM overrides experience," why is it the most interesting failure, and what would fix it?

<details>
<summary>Answer</summary>

5 of 19 first-failures. The system **retrieved the correct memory** and then decided the current scene didn't match closely enough, and ignored it. Example: in "put tennis ball in box (mug obstructs)," it held a memory about clearing a similar occlusion but judged the mug's obstruction negligible.

It is the most interesting failure because **retrieval succeeded and integration failed.** No amount of better embedding or larger $k$ fixes it. The model has no calibrated sense of how much to trust retrieved experience versus its own visual judgement — the legal analogy is *distinguishing* a precedent away.

Plausible fixes, none of which the paper implements:
- **Structured memory.** Represent a lesson as a precondition/constraint the planner must explicitly discharge, rather than advisory prose it can talk itself out of. (This is a strong motivation for ontology-based memory representations.)
- **Confidence-weighted retrieval.** Attach a track record to each memory and require justification to override a high-confidence one.
- **Force explicit refutation.** The schema already has `applicable_knowledge`; requiring the model to state *why* it is discarding a retrieved lesson makes the override auditable and probably rarer.

Note that the existing `applicable_knowledge` field makes these failures **loggable today** — you can see the moment the model reasons its way out of using a memory.

</details>

---

### 14. The LTM has 100 entries. How many came from the robot's own autonomous experience, and why does that matter?

<details>
<summary>Answer</summary>

**Four.** The four experiences generated by the STM experiments in §V-B. The other **96 are "limited instructional experiences from simpler tasks"** — i.e. seeded by the authors.

Why it matters: the abstract's framing is "learning by experiencing the real world," and Table I claims "learning by experience" as the distinguishing capability. Both are legitimate — the *mechanism* is demonstrated end to end, and the 4 autonomous entries did feed the transfer results. But the *memory that produces the 80%* is overwhelmingly curated, not autonomously accumulated.

This is a scale limitation the authors are transparent about (§VI: "our current memory database is limited by the cost of real-world data collection"), and it is not fraud — it is the honest cost of real-robot experiments. But it does mean the paper has **not** demonstrated that a robot can bootstrap a useful memory from scratch, only that a useful memory helps and that autonomous entries can be added to one.

</details>

---

### 15. You are reproducing this on a fixed-base arm with a static camera, and their LTM is available. Should you use their LTM? What does "reproducing the paper" mean here?

<details>
<summary>Answer</summary>

**No, you should not use their LTM as-is**, for two independent reasons:

1. **Embodiment.** LTM entries encode *that* robot's limitations — a Robotiq 2F-140 on a 6-DoF arm on a quadruped, with an elbow-mounted camera. A lesson like "this object is too small to push with the gripper" is a fact about that gripper. Your FR3 with a Franka Hand has different limits, so some lessons will be wrong, and wrong memories are worse than no memories (see Q13 — the model over-trusting a bad memory).
2. **Mechanics.** Embeddings are stored per embedding-model in `ltm_<model>.csv` and joined on the scenario key. Using a different embedding backend means the embeddings are unusable and must be regenerated.

Also **eye-in-hand vs eye-on-base** changes the meaning of spatial language in the scene descriptions ("directions with respect to the camera"), so their scene descriptions do not even mean the same thing in your setup.

**What "reproducing the paper" should mean:**

- ✅ **Reproduce the claims:** show on *your* robot with *your* skills that (a) STM self-reflection raises success over a no-memory baseline, and (b) RAG retrieval beats both random retrieval and full-context prompting. These are the paper's actual scientific assertions and they are platform-independent.
- ❌ **Do not aim to reproduce the numbers:** 86%, 100%, 89% depend on their hardware, their AnyGrasp setup, their scenes, and a `gpt-4o` snapshot that has since drifted. With 5–10 trials per task, a single trial is worth 10–20 percentage points anyway.

The honest reproduction target is *the shape of the effect*, measured on your own platform, with your own baseline.

</details>
