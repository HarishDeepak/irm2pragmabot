# Summary — A Pragmatist Robot: Learning to Plan Tasks by Experiencing the Real World

**Venue:** IEEE Robotics and Automation Letters (RAL), 2026 · 8 pages
**Authors:** Kaixian Qu, Guowei Lan, René Zurbrügg, Changan Chen, Christopher E. Mower, Haitham Bou-Ammar, Marco Hutter
**Affiliations:** Robotic Systems Lab, ETH Zürich · ETH AI Center · Huawei Noah's Ark Lab, London · UCL Centre for AI
**Project page:** https://pragmabot.github.io/ · **Code:** https://github.com/leggedrobotics/pragmabot

---

## Background context

Large language models became the default approach to robot task planning: give the model an instruction, get back a sequence of skills. The approach works because LLMs carry broad common-sense knowledge — they know a plate goes under an apple, that you open a container before reaching inside.

But that knowledge is *human* knowledge, learned from internet text. It encodes what is easy for a person with two hands, binocular vision, and a lifetime of contact experience. A robot has a specific arm, a specific gripper, a specific perception stack, and a specific set of things it reliably fails at. Nothing in pretraining tells the model any of that.

Vision-language models narrowed part of the gap by accepting images instead of text-only state descriptions. They did not close the *embodiment* gap. A VLM will look at a tennis ball half-hidden behind a fan and confidently say "pick up the tennis ball," because for a human that is trivially easy.

## Problem statement

**How can a robot align a frozen VLM with its own embodiment, skill set, and limitations — without fine-tuning it?**

Fine-tuning is rejected on cost: it needs data collection on hardware, compute, and re-training per platform. The paper instead adapts **verbal reinforcement learning** (Reflexion, [17]): the agent improves by writing natural-language critiques of its own failures into its context. The learning signal lands in the prompt, not the weights.

Reflexion had two gaps this paper targets:
1. It was evaluated almost entirely in **simulation**.
2. Its observations came from **simulator ground truth or hand-written scene descriptors**, not real perception.

PragmaBot puts verbal RL on a physical robot and closes the loop with a VLM that looks at actual camera images.

## Main contributions

1. **PragmaBot** — verbal RL adapted to real-world robotic task planning, combining short-term memory (within-task self-reflection) and long-term memory (across-task experience), demonstrated to produce planning aligned to the robot's actual capabilities.
2. **RAG inside the verbal RL loop** — selectively retrieving task-relevant past experiences rather than dumping all memory into context. Shown to substantially beat naive prompting.
3. **An on-demand image annotation module** — the VLM decides *itself* whether it needs help with spatial grounding; if so, candidate locations are overlaid as numbered masks for it to choose from. Improves grasping and pushing accuracy across diverse skills.

## Method in brief

One VLM plays four roles: scene describer, task planner, success detector, experience summarizer.

```
1. Describe initial scene            → build retrieval key (instruction + scene description)
2. RAG over long-term memory         → top-k similar past experiences (cosine similarity)
3. Plan next action                  → skill (pick/place/push/done) + parameters
4. Execute                           → *** left as NotImplementedError in the release ***
5. Detect success visually           → compare before/after images
6. Append (action, feedback) to STM  → on failure, self-reflect and replan
7. On completion, summarize STM      → store as a long-term memory entry
```

The short-term memory is the within-task adaptation mechanism; the long-term memory plus RAG is the across-task generalization mechanism. Neither changes a single model weight.

## Key results

| Claim | Evidence |
|---|---|
| STM self-reflection raises task success | **35% → 84%** mean over 4 tasks vs CaP-V baseline (Table II; 5–10 trials each, two attempts allowed) |
| LTM + RAG raises **single-trial** success | **22% → 80%** mean over 12 scenarios vs COME (Table III; 8 of 12 previously unseen) |
| Selective retrieval beats dumping all memory | first-action accuracy **89% (RAG)** vs **74% (entire LTM)** vs **17% (random k=5)** |
| Full-LTM context is also expensive | **7.5×** prompt-length increase, excluding images |
| Image annotation improves grasping | Higher success on complex shapes (drumstick, skewer); lower push distance error. Cost: **+5.15 s** per gpt-4o call |
| Visual success detection is reliable enough to build on | **6.67% false negative** (4/60), **5% false positive** (3/60) on picking |

## Quantitative setup

- **Robot:** ANYmal quadruped + 6-DoF arm, Robotiq 2F-140 gripper, ZED X Mini mounted on the elbow (eye-in-hand)
- **VLM:** `gpt-4o`; **embeddings:** `text-embedding-3-large`; smaller-model ablation with `gpt-4o-mini`
- **LTM:** 100 entries — 4 from autonomous experience plus 96 "limited instructional experiences from simpler tasks"; frozen during evaluation
- **Skills:** pick, place, push; grasping via AnyGrasp, IK feasibility filtering via Pinocchio
- **Human in the loop:** an operator may reset the scene after a failure that alters the environment

## Failure analysis (19 first-failures / 91 trials)

| Source | Count | Breakdown |
|---|---|---|
| **Execution** | 8 | poor grasp generation (5), inaccurate depth (3) |
| **VLM reasoning** | 7 | overrides retrieved experience (5), false detection (1), wrong mask selection (1) |
| **RAG** | 4 | retrieval failed (4) |

The largest single bucket is **execution** — perception and grasping, the layer the paper does not itself contribute.

## Stated limitations

1. Vision-only; no tactile or auditory feedback.
2. Memory database is small, limited by the cost of real-world data collection. Scaling raises unanswered questions about pruning and forgetting.
3. Top-$k$ retrieval will degrade as memory grows; maximum marginal relevance (MMR) suggested as future work.
4. Cross-robot memory sharing is open when morphologies differ.
