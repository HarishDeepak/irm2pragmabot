# Mental Model — PragmaBot

How to file this paper in your head, and what it assumes you already know.

---

## 1. What type of problem is this?

**It is a systems paper about memory, wearing robotics clothes.**

The robot is the testbed, not the contribution. Strip the hardware away and you have: an agent with a frozen policy, a verifier, an episodic buffer, a persistent store, and a retrieval rule. The research question is *where to put the learning when you cannot touch the weights* — and the answer is "in the context, structured into two timescales."

Classify it as:
- **Not** a manipulation paper. No new controller, no new grasp model, no new perception. It *consumes* AnyGrasp, Grounded SAM, Pinocchio.
- **Not** a learning paper in the gradient sense. Zero parameter updates anywhere.
- **Yes** an agent-architecture paper: memory design, retrieval design, self-verification.
- **Yes** an empirical systems paper: the contribution is a working closed loop on real hardware plus ablations showing which parts carry the weight.

## 2. The one-sentence version

*Let a frozen VLM fail on a real robot, make it write down why, keep the note for the rest of the task, distil it when the task ends, and retrieve it by similarity the next time something like it happens.*

## 3. Prior knowledge assumed

**Necessary**
- **LLM/VLM prompting** — chain-of-thought, structured/constrained output, why field order matters under autoregressive decoding.
- **RAG** — embeddings, cosine similarity, top-$k$ retrieval, why chunk relevance beats chunk quantity.
- **RL vocabulary** — policy, reward, episode, replay. Only the vocabulary; no equations needed. The paper leans on the analogy hard.
- **Robot task planning** — the split between a high-level symbolic/semantic planner and low-level parameterised skills.

**Helpful**
- **Reflexion** [17] — the direct ancestor. PragmaBot is Reflexion + vision + real hardware + cross-episode retrieval.
- **Open-vocabulary segmentation** — GroundingDINO + SAM, text prompt to mask.
- **6-DoF grasp synthesis** — that a network can propose SE(3) grasps from a point cloud with confidences.
- **Stereo depth failure modes** — helps you understand why 3/19 failures are "inaccurate depth."

**Not required**
- Legged locomotion. The ANYmal is a mobile base here; nothing depends on it being legged.
- Any optimisation or control theory. There is no math beyond cosine similarity and an argmax.

## 4. Where it sits in the research map

```
LLMs for robot planning
│
├── Grounding via learned affordances
│     SayCan [6]  — needs a trained value function; expensive
│
├── Grounding via code generation
│     CaP [5]  — LLM writes policy code; no feedback loop
│
├── Grounding via closed-loop feedback
│     Inner Monologue [8], COME [14], ReplanVLM [15]
│     └── reacts to failure, but forgets it afterwards
│
├── Grounding via human correction
│     DROC [18], BUMBLE [20]
│     └── works, but needs dense human supervision
│
├── Verbal RL (self-generated feedback, no weight updates)
│     Reflexion [17]  — simulation only, ground-truth or hand-written observations
│     │
│     └──▶ PRAGMABOT — real robot, VLM-generated visual feedback,
│                       plus RAG over persistent cross-task memory
│
└── Retrieval-augmented agents
      RAP [24]  — RAG over experience, but simulated and no self-reflection
```

The paper's slot is the **intersection**: it is the first to combine *self-generated visual feedback*, *persistent cross-task memory*, and *retrieval* on a **physical** robot.

## 5. The three ideas to actually remember

1. **Learning without weight updates.** If the model is frozen and expensive to tune, the context window is your parameter space. Two timescales — within-episode and across-episode — are the natural structure.
2. **Retrieval is an accuracy mechanism, not a cost optimisation.** 89% (top-5) vs 74% (everything) vs 17% (random). More context made it worse. This is the most transferable finding in the paper.
3. **Geometry proposes, semantics disposes.** A grasp network gives you metrically valid poses with no idea what the object is for; a VLM knows what it is for but cannot produce SE(3). Multiply the two scores.

## 6. Analogies that hold

- **STM ≈ a scratchpad during one exam question; LTM ≈ the notes you keep between exams.** The scratchpad is thrown away; the notes accumulate.
- **The success detector ≈ a unit test written by the same person who wrote the code.** Cheap, effective, and shares the author's blind spots. The paper's 5% false-positive rate is precisely that risk, measured.
- **RAG here ≈ a case-law lookup.** You do not re-derive from first principles; you find the most similar precedent and argue from it. And, exactly as in law, the interesting failure is *distinguishing* the precedent away — the paper's "VLM overrides experience," 5 of 19 failures.

## 7. Analogies that break

- **It is not "the robot learns."** Nothing in the robot changes. A text file grows. Move the file to another robot with a different gripper and the lessons may be actively wrong.
- **It is not really reinforcement learning.** No value function, no credit assignment over time, no exploration policy. The "gradient" is a metaphor, and a one-step one — reflection acts on the immediately preceding failure, not on a discounted return.
- **It is not lifelong learning at scale (yet).** 100 entries, 96 of them hand-seeded, with top-$k$ retrieval the authors themselves say will stop working as memory grows.

## 8. How to categorise it when you cite it

> PragmaBot demonstrates that verbal reinforcement learning transfers from simulated language agents to a physical manipulator when the feedback signal is supplied by a VLM comparing before/after images, and that selectively retrieving prior experience outperforms both random retrieval and full-context prompting.

That sentence is defensible. What is **not** defensible from this paper alone: that it constitutes lifelong learning, that the 80% figure reflects fully autonomous operation, or that the memory mechanism is responsible for all of the improvement (execution quality is an uncontrolled confound at 8/19 failures).

## 9. If you are reproducing it

Understand the division of labour before writing any code:

| Layer | Who provides it |
|---|---|
| Planner, memory, retrieval, success detection, summarisation | **The paper and its released code** |
| Segmentation, grasp synthesis, motion planning, gripper control, calibration | **You** |

The released repository leaves execution as `NotImplementedError` deliberately. So "reproducing the paper" splits cleanly in two:

- **Reproducing the *claims*** = showing STM improves success and RAG beats naive prompting **on your own robot with your own skills**. This is the scientific reproduction and it is what the paper is about.
- **Reproducing the *numbers*** = not possible without their exact hardware, their AnyGrasp setup, their scenes, and their `gpt-4o` snapshot. Do not set this as the goal.

The mental trap to avoid: treating the manipulation stack as "setup work" before the real project. On this paper's own failure analysis, execution quality is the single largest source of failure — which makes the skill layer a legitimate site of contribution, not a prerequisite chore.
