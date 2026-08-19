# PragmaBot — Study Folder

**A Pragmatist Robot: Learning to Plan Tasks by Experiencing the Real World**
Qu, Lan, Zurbrügg, Chen, Mower, Bou-Ammar, Hutter · IEEE RAL 2026 · arXiv [2507.16713](https://arxiv.org/abs/2507.16713)
ETH Zürich RSL · ETH AI Center · Huawei Noah's Ark Lab London · UCL Centre for AI
[Project page](https://pragmabot.github.io/) · [Code](https://github.com/leggedrobotics/pragmabot)

**Tags:** `task planning` · `verbal reinforcement learning`

---

## What this paper is about

A robot learns to plan tasks better **without any model being trained**. A vision-language model plans an action, looks at before/after images to judge whether it worked, and — when it fails — writes a natural-language critique of its own failure into a short-term memory that shapes its next decision. When the task finally succeeds, the whole episode is distilled into one lesson and filed in a long-term memory, indexed by a description of the situation. Faced with a new task later, the robot retrieves the most similar past situations and plans with those lessons in context. The learning signal is text; the weights never move.

## Difficulty

**Intermediate.** The method involves no difficult mathematics — the only equations are a cosine similarity, an argmax, and a product of two scores. What makes it non-trivial is systems judgement: knowing why memory is split across two timescales, why retrieving five entries beats providing a hundred, and why the success detector is the component everything else rests on.

Harder if you have never worked with structured LLM output or RAG. Easier if you have; then the robotics is mostly context.

## Key takeaways

1. **Learning without weight updates.** When a model is frozen and expensive to tune, the context window is your parameter space. Structure it across two timescales — within-episode (STM) and across-episode (LTM).
2. **Selective retrieval is an accuracy mechanism, not a cost optimisation.** Top-$k$ RAG **89%** vs entire memory **74%** vs random **17%**. More context made it worse *and* cost 7.5× the tokens.
3. **Geometry proposes, semantics disposes.** A grasp network produces metrically valid poses with no idea what an object is for; a VLM knows what it is for but cannot produce SE(3). Multiply the two scores.
4. **The verifier is the foundation.** ~5–7% detector error is what makes reflection trustworthy. It gates both replanning and what gets written to memory.
5. **The paper does not contribute the skill layer** — and 8 of its 19 first-failures come from exactly that layer. Execution quality is the ceiling.

## How to navigate this folder

| File | Read it for |
|---|---|
| **`summary.md`** | Context, problem, contributions, all results tables, failure analysis. Start here. |
| **`insights.md`** | **Most important.** Why the method works, the RL analogy and its limits, trade-offs, critical reading, emergent behaviours. |
| **`method.md`** | Architecture diagram, annotated algorithm, prompt templates, real output schemas, pitfalls, reproduction risks. |
| **`mental-model.md`** | How to classify this paper, prerequisites, where it sits in the literature, what "reproducing it" should mean. |
| **`qa.md`** | 15 self-test questions (5 basic / 5 intermediate / 5 advanced) with hidden answers. |
| **`code/memory_retrieval_demo.py`** | Runnable demo reproducing the central retrieval ablation. numpy only. |
| `paper.txt` / `paper.pdf` | Full text and original. |
| `page_01.png` … `page_08.png` | Rendered pages — **read these for the figures**; caption text alone loses the content of Figs. 4, 7 and 8. |
| `images/` | 63 raw extracted image objects. |
| `meta.json` | Parser metadata. |

## Suggested study time

- **Fast pass (45 min):** `summary.md` → `insights.md` §§1–5 → look at pages 3, 5, 6, 7 for Figs. 2, 4, 6, 8.
- **Working understanding (2–3 h):** add `method.md`, run the code demo, then `qa.md` basic + intermediate.
- **Reproduction-ready (half a day):** add `mental-model.md`, `qa.md` advanced, `insights.md` §7 (limitations), and `method.md` §8 (reproduction risks).

## Run the demo

```bash
cd code
python memory_retrieval_demo.py                 # reproduces the rag > all > random ordering
python memory_retrieval_demo.py --distraction 0 # removes the long-context penalty; the gap closes
python memory_retrieval_demo.py --top-k 1 --trials 5000
```
Requires only `numpy`. Deterministic across runs.

## One thing to be careful about

The two headline numbers are **not** the same measurement. **35% → 84%** (STM) allows **two attempts** per trial; **22% → 80%** (LTM+RAG) is **single-trial**, against a different baseline. Quoting them as one progression is a misreading.
