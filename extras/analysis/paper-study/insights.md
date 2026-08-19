# Insights — PragmaBot

The most important file in this folder. What the paper is *really* doing, why it works, what it costs, and where it is fragile.

---

## 1. The core idea, plainly

Take a frozen VLM that is confidently wrong about what a robot can do. Instead of retraining it, **let it fail, make it write down why, and put that note back in its prompt.** Do this within a task (short-term memory) and across tasks (long-term memory, retrieved by similarity).

That's it. No weight updates, no gradients, no dataset. The learning is text.

## 2. The conceptual shift: gradients made of language

The paper's own framing is the sharpest way to understand it:

> Self-reflection is a **"linguistic gradient."** It plays the role gradient computation plays in reinforcement learning — but the update lands in the context window, not in the parameters.

Map it onto RL and every piece lines up:

| RL concept | PragmaBot analogue |
|---|---|
| Policy $\pi_\theta$ | VLM task planner $\Pi_\theta$ (θ frozen) |
| Reward signal | VLM success detector comparing before/after images |
| Gradient step | Natural-language self-reflection appended to STM |
| Replay buffer / experience | Long-term memory of distilled lessons |
| Policy improvement | Better in-context prompt, same weights |
| Episode | Task, terminated when the detector says complete |

The analogy is genuinely load-bearing rather than decorative, and it explains the design: STM is the *online* update within an episode; LTM is what persists across episodes; RAG is how you decide which past experience is relevant to the current state.

## 3. Why it works

**Because failure is informative and language is a good medium for it.** When a grasp fails because a can is in the way, the useful lesson is not a scalar reward — it is the sentence "the apple is next to a cylindrical container, which might be causing interference; push the can away first." That sentence is directly consumable by the same model that made the error, and it transfers to *any* structurally similar situation.

**Because the retrieval key is semantic, not geometric.** The LTM index is `(instruction, initial scene description)` — both natural language. Two physically different scenes with the same *structure* ("target is blocked by a nearby object") embed near each other. That is exactly why experience from "push the can away, then grab the apple" transfers to "pick up the milk carton" (reposition the apple first). Generalization comes for free from the embedding space, without any explicit notion of task similarity.

**Because chain-of-thought precedes commitment.** Reflection is produced *before* the action in the same output. The model cannot rationalize a decision it has already made; it must reason first. This ordering is a small implementation detail with an outsized effect.

**Because the success detector is good enough.** ~5–7% error on picking. The paper says outright that the emergent adaptive behaviour "would not be possible without the high accuracy of the VLM's success detection." Everything downstream is built on that signal — if it were 25% wrong, the STM would fill with noise and reflection would actively hurt.

## 4. The result that should change how you build LLM systems

**Retrieving 5 relevant memories beats providing all 100.**

| Setting | First-action accuracy |
|---|---|
| Random $k=5$ | 17% |
| **Entire LTM in context** | **74%** |
| **RAG top-$k$** | **89%** |

More context made the system *worse*. Fifteen points worse. This is consistent with known long-context degradation ([38], [39]), but here it is measured on a robotics task with real consequences — and the full-context option also costs **7.5× the prompt length**.

The takeaway generalizes well beyond this paper: **relevance filtering is not a cost optimization, it is an accuracy mechanism.** People reach for "just put everything in the context window" as the simple default; on this evidence that is both more expensive and less accurate.

## 5. The second-order insight: geometry proposes, semantics disposes

Pure geometric grasp planning picks the *wrong part* of an object. AnyGrasp "often favors larger surfaces due to its reliance on geometric cues" — it will grab the meat on a skewer rather than the stick, the ice cream rather than the cone.

The fix is a division of labour:

$$g^* = \arg\max_{g \in G} \; s_{\text{conf}}(g)\cdot s_{\text{loc}}(g)$$

Geometry generates feasible candidates with confidences; the VLM picks *where* semantically; the product decides. Neither alone is sufficient — a VLM cannot produce a metrically valid SE(3) grasp, and a grasp network has no idea what a skewer is *for*.

The **on-demand** part is underrated: the VLM emits a boolean for whether it needs annotation at all. Simple objects (an apple) skip it and save 5.15 s. The model is trusted to know when it needs help — a small piece of metacognition that turns a fixed cost into a conditional one.

## 6. Trade-offs

| You gain | You pay |
|---|---|
| No fine-tuning, no dataset, no training compute | Per-step API latency and money; +5.15 s per annotated call |
| Human-readable, auditable memory | Memory quality depends on VLM summarization quality |
| Learning transfers across tasks immediately | Retrieval is top-$k$ cosine — will not scale |
| Works with any frozen VLM | Locked to whatever that VLM is good and bad at |
| Skills stay modular and swappable | Ceiling set by the skill layer, which the paper doesn't contribute |

## 7. Limitations — the authors' and mine

**Stated by the authors:** vision-only; small memory limited by data-collection cost; top-$k$ won't scale (MMR suggested); cross-morphology memory sharing unresolved.

**Additional, from a critical reading:**

- **Small samples, no statistics.** 5–10 trials per task, no confidence intervals or significance tests. "Move screw: 0% → 86%" rests on ≤10 trials. Directionally convincing; not statistically established.
- **The LTM is 96% hand-seeded.** Only 4 of 100 entries came from autonomous experience. The mechanism is demonstrated, but "learning by experiencing" at the scale shown is mostly curated instruction. The paper is transparent about this; the abstract invites over-reading it.
- **Protocols differ between headline numbers.** Table II allows **two attempts**; Table III is **single-trial**. The 84% and 80% are not comparable quantities.
- **Human scene resets.** After a destructive failure a human may reset the scene. Reported rates are therefore not fully autonomous episode success.
- **The grader shares weights with the planner.** Same model detects success and plans. Correlated blind spots are plausible and only partially measured (picking only).
- **"Unseen" means structurally similar.** Generalization is demonstrated within a distribution of tabletop rearrangements, by the paper's own description.
- **Execution quality is a confound.** 8 of 19 first-failures are grasping/depth. A better manipulation stack would raise the headline numbers *without touching the memory contribution*. This cuts both ways: it also means a better skill layer is a legitimate contribution on top of this paper.

## 8. Comparison to prior work

| Method | Self-reflect | Learn by experience | Interactive replan | Creative tool use |
|---|---|---|---|---|
| CaP [5] | ✗ | ✗ | ✗ | ✗ |
| SayCan [6] | ✗ | ✗ | ✗ | ✗ |
| Inner Monologue [8] | ✗ | ✗ | ✗ | ✗ |
| RoboTool [23] | ✗ | ✗ | ✗ | ✓ |
| DROC [18] | ✗ | ✓ | ✗ | ✗ |
| REFLECT [21] | ✓ | ✗ | ✓ | ✗ |
| COME [14] | ✓ | ✗ | ✗ | ✗ |
| ReplanVLM [15] | ✓ | ✗ | ✓ | ✗ |
| BUMBLE [20] | ✓ | ✗ | ✓ | ✗ |
| **PragmaBot** | **✓** | **✓** | **✓** | **✓** |

The genuinely distinguishing column is **learning by experience** — retaining and reusing lessons across tasks without human annotation. SayCan needs a trained affordance network; DROC and BUMBLE need dense human corrections. PragmaBot's feedback comes from its own eyes.

The other distinguishing property is **unified visual feedback**: one model both plans and verifies. That is what makes it *self*-reflection rather than one model grading another — and simultaneously the source of the correlated-blind-spot risk.

## 9. Emergent behaviour worth noticing

Two behaviours were not designed in and appeared from reflection alone:

- **Tool use.** Told to move a tiny candy and unable to push it with the gripper (insufficient contact), the robot chose to pick up a **sponge** and push with that. Nobody wrote a tool-use skill.
- **Precondition discovery.** Told to collect a bowl with an apple inside, it lifted the bowl, the apple fell out, and it reflected: "I should first move the apple to the table before picking up the bowl." It learned an ordering constraint from a physical consequence.

After cracking an egg while grasping, it learned to *push* fragile objects rather than grasp them. These are the paper's most persuasive qualitative results — they are the behaviours you would have to hand-code in a classical planner.

## 10. The most interesting failure

**"VLM overrides experience" — 5 of 19 first-failures.**

The system retrieved the *correct* memory and then decided the current scene didn't match closely enough, and ignored it. In "put tennis ball in box (mug obstructs)," it had a memory about clearing a similar occlusion but judged the mug's obstruction negligible.

This is a real open problem with no clean fix in the paper: **the model has no calibrated sense of how much to trust retrieved experience versus its own visual judgement.** Retrieval succeeded; integration failed. Any follow-on work on memory representations runs straight into this — and it is a strong motivation for structured (e.g. ontology-based) memory, where a precondition could be represented as a constraint rather than as advisory prose the model is free to talk itself out of.

## 11. Practical implications

- **For LLM systems generally:** retrieve selectively; more context is not free and is not always better. Make chain-of-thought precede the decision. Let the model declare when it needs extra grounding.
- **For robotics:** the skill layer sets the ceiling. Memory cannot rescue a bad grasp. If you reproduce this paper, the fastest route to better numbers is often better perception, not better prompting.
- **For reproduction specifically:** the released code deliberately omits step 4 (execution). The published results are inseparable from an execution stack the paper does not contribute and only partly describes. Reproducing "the paper" therefore means reproducing the *memory* claims; the manipulation stack is yours to build and yours to be judged on.
