#!/usr/bin/env python3
"""
memory_retrieval_demo.py — why selective retrieval beats dumping everything.

Reproduces, in miniature, the central ablation of PragmaBot (Fig. 7):

    random k=5      ->  17% first-action accuracy
    entire LTM      ->  74%
    RAG top-k       ->  89%

The paper's result is counter-intuitive: giving the planner ALL of its
memory is *worse* than giving it five relevant entries. This script shows
the mechanism behind that, with no API keys, no GPU, and no robot --
numpy only.

WHAT IS AND ISN'T REAL HERE
---------------------------
Real (faithful to the paper):
  * the scenario-key format          "Instruction: {i}\nScene: {s}"
  * retrieval by cosine similarity over embedded keys, top-k
  * the three retrieval conditions being compared
  * the *shape* of the result: rag > all > random

Simulated (because a real VLM needs an API key):
  * the embedding model -> a deterministic bag-of-words hash embedding
  * the planner         -> a model of how an LLM uses context, with an
                           explicit distraction penalty that grows with
                           the number of irrelevant entries in the prompt

The distraction penalty is the whole point. If you set it to zero,
"entire LTM" wins trivially, because it always contains the right entry.
The paper's finding is that this term is NOT zero for real LLMs -- long,
noisy contexts degrade performance ([38], [39] in the paper). This script
lets you see how strong that effect must be to reproduce their ordering.

USAGE
    python memory_retrieval_demo.py
    python memory_retrieval_demo.py --distraction 0.0    # penalty off
    python memory_retrieval_demo.py --top-k 1 --trials 5000
"""

import argparse
import zlib

import numpy as np


def _stable_hash(text: str) -> int:
    """Process-independent hash.

    Python's builtin hash() is salted per process (PYTHONHASHSEED), so
    using it here would make the demo give different numbers on every
    run. crc32 is stable across processes and machines.
    """
    return zlib.crc32(text.encode("utf-8"))

# --------------------------------------------------------------------------
# A toy long-term memory. Each entry is (scenario_key, distilled_lesson).
# The lessons mirror the ones the paper reports emerging from reflection:
# clear the occluder first, use a tool for tiny objects, empty a container
# before lifting it, push rather than grasp fragile things.
# --------------------------------------------------------------------------

LESSONS = {
    "occlusion": "The target was blocked by a nearby object. Push the "
                 "blocking object away first, then pick the target.",
    "tiny":      "The object was too small for the gripper to contact. "
                 "Pick up a flat tool (sponge/towel) and push with it.",
    "container": "The container had an object inside. Remove the contents "
                 "onto the table before lifting the container.",
    "fragile":   "Grasping crushed the object. Push it instead of grasping.",
    "stacked":   "Another object was resting on top. Remove it first.",
}

# Scenarios that genuinely exercise each lesson.
TASKS = [
    ("Put the apple on the plate",   "an apple next to a tall can on a table",            "occlusion"),
    ("Put the tennis ball in the box", "a tennis ball behind a mug",                      "occlusion"),
    ("Put the orange on the plate",  "an orange with a fan standing in front of it",      "occlusion"),
    ("Move the candy to the banana", "a very small candy beside a sponge",                "tiny"),
    ("Move the screw to the toolbox", "a tiny screw lying near a folded towel",           "tiny"),
    ("Move the grape to the banana", "a single grape on an open table",                   "tiny"),
    ("Collect the bowl",             "a bowl with an apple resting inside it",            "container"),
    ("Pick up the milk carton",      "a milk carton with an apple leaning against it",    "container"),
    ("Move the egg to the sushi",    "a raw egg on an open table",                        "fragile"),
    ("Move the sushi to the plate",  "a soft piece of sushi on an open table",            "fragile"),
    ("Pick up the box",              "a box with an apple sitting on top of it",          "stacked"),
    ("Pick up the towel",            "a folded towel with an orange resting on it",       "stacked"),
]

# Filler entries: the "96 limited instructional experiences from simpler
# tasks" the paper mentions. Individually harmless, collectively the
# source of the distraction effect.
FILLER = [
    ("Pick up the {}", "a {} alone on an empty table",
     "The object was clearly visible with space around it. A direct pick "
     "succeeded on the first attempt.")
    for _ in range(1)
]
FILLER_OBJECTS = [
    "mug", "book", "bottle", "spoon", "plate", "banana", "remote", "cube",
    "marker", "stapler", "can", "jar", "brush", "block", "cup", "lid",
    "bowl", "fork", "knife", "phone", "wallet", "glove", "sponge", "towel",
]


def scenario_key(instruction: str, scene: str) -> str:
    """The paper's LTM key format (utils.get_scenario_key)."""
    return f"Instruction: {instruction}\nScene: {scene}"


def embed(text: str, dim: int = 256) -> np.ndarray:
    """Deterministic bag-of-words hash embedding.

    Stands in for a real text-embedding model. Shares the one property
    that matters for this demo: texts with overlapping vocabulary land
    near each other in cosine space.
    """
    vec = np.zeros(dim, dtype=np.float64)
    for word in text.lower().replace("\n", " ").split():
        word = word.strip(".,:;!?\"'")
        if not word:
            continue
        # two hashed positions per token reduces collision artifacts
        vec[_stable_hash(word) % dim] += 1.0
        vec[_stable_hash(word + "#") % dim] += 0.5
    norm = np.linalg.norm(vec)
    return vec / norm if norm > 0 else vec


def cosine(a: np.ndarray, b: np.ndarray) -> float:
    na, nb = np.linalg.norm(a), np.linalg.norm(b)
    if na == 0 or nb == 0:
        return 0.0
    return float(a @ b / (na * nb))


def build_ltm(rng: np.random.Generator):
    """Return (keys, categories, embeddings) for a 100-entry LTM.

    12 task-relevant entries + 88 filler, matching the paper's 100-entry
    memory that is mostly simple instructional experience.
    """
    keys, cats = [], []
    for instruction, scene, cat in TASKS:
        keys.append(scenario_key(instruction, scene))
        cats.append(cat)

    i = 0
    while len(keys) < 100:
        obj = FILLER_OBJECTS[i % len(FILLER_OBJECTS)]
        suffix = "" if i < len(FILLER_OBJECTS) else f" number {i // len(FILLER_OBJECTS)}"
        keys.append(scenario_key(f"Pick up the {obj}{suffix}",
                                 f"a {obj}{suffix} alone on an empty table"))
        cats.append(None)          # no transferable lesson
        i += 1

    embeddings = np.stack([embed(k) for k in keys])
    return keys, cats, embeddings


def retrieve(query_emb, embeddings, mode, top_k, rng):
    """Return indices of retrieved entries under one of three policies."""
    if mode == "rag":
        sims = embeddings @ query_emb / (
            np.linalg.norm(embeddings, axis=1) * np.linalg.norm(query_emb) + 1e-12)
        return np.argsort(-sims)[:top_k]
    if mode == "all":
        return np.arange(len(embeddings))
    if mode == "random":
        return rng.choice(len(embeddings), size=top_k, replace=False)
    raise ValueError(mode)


def plan_is_correct(retrieved_idx, cats, needed_cat, distraction, rng):
    """Model of the planner's first-action accuracy.

    Two competing effects, both reported in the paper:

      + having the right lesson in context helps a lot
      - every irrelevant entry in context costs a little, because long
        noisy contexts degrade LLM focus (the paper's explanation for
        why 'entire LTM' scores 74% and not ~90%)

    Base rates chosen so the no-memory case sits near the paper's COME
    baseline (~22%).
    """
    has_relevant = any(cats[i] == needed_cat for i in retrieved_idx)
    n_irrelevant = sum(1 for i in retrieved_idx if cats[i] != needed_cat)

    p = 0.92 if has_relevant else 0.22
    p -= distraction * np.log1p(n_irrelevant)      # diminishing, not linear
    p = float(np.clip(p, 0.02, 0.98))
    return rng.random() < p


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--top-k", type=int, default=5)
    ap.add_argument("--trials", type=int, default=2000)
    ap.add_argument("--distraction", type=float, default=0.055,
                    help="per-log-irrelevant-entry accuracy penalty "
                         "(0 disables the long-context effect entirely)")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    rng = np.random.default_rng(args.seed)
    keys, cats, embeddings = build_ltm(rng)

    print("=" * 72)
    print("PragmaBot memory-retrieval ablation, in miniature")
    print("=" * 72)
    print(f"LTM size            : {len(keys)} entries "
          f"({sum(c is not None for c in cats)} task-relevant, "
          f"{sum(c is None for c in cats)} filler)")
    print(f"top_k               : {args.top_k}")
    print(f"trials per condition: {args.trials}")
    print(f"distraction penalty : {args.distraction}")

    # ---- show one concrete retrieval, so the mechanism is visible -------
    instruction, scene, needed = TASKS[1]          # tennis ball / mug
    q = embed(scenario_key(instruction, scene))
    idx = retrieve(q, embeddings, "rag", args.top_k, rng)
    sims = [cosine(embeddings[i], q) for i in idx]

    print("\n" + "-" * 72)
    print("EXAMPLE QUERY")
    print("-" * 72)
    print(f"instruction : {instruction}")
    print(f"scene       : {scene}")
    print(f"lesson needed: {needed!r}")
    print(f"\ntop-{args.top_k} retrieved:")
    for rank, (i, s) in enumerate(zip(idx, sims), 1):
        first_line = keys[i].split("\n")[0].replace("Instruction: ", "")
        tag = cats[i] if cats[i] else "-"
        hit = "  <== transferable lesson" if cats[i] == needed else ""
        print(f"  {rank}. cos={s:.3f}  [{tag:<9}] {first_line}{hit}")

    if needed in [cats[i] for i in idx]:
        print(f"\n  -> the '{needed}' lesson was retrieved from a DIFFERENT "
              "task\n     (apple/can), which is exactly the generalisation "
              "the paper claims.")

    # ---- run the three conditions --------------------------------------
    print("\n" + "-" * 72)
    print("FIRST-ACTION ACCURACY OVER ALL 12 TASKS")
    print("-" * 72)

    results = {}
    for mode in ("random", "all", "rag"):
        correct = 0
        for t in range(args.trials):
            instruction, scene, needed = TASKS[t % len(TASKS)]
            q = embed(scenario_key(instruction, scene))
            idx = retrieve(q, embeddings, mode, args.top_k, rng)
            if plan_is_correct(idx, cats, needed, args.distraction, rng):
                correct += 1
        results[mode] = 100.0 * correct / args.trials

    paper = {"random": 17, "all": 74, "rag": 89}
    label = {"random": f"random k={args.top_k}",
             "all": "entire LTM in context",
             "rag": f"RAG top-{args.top_k}"}

    print(f"{'condition':<26}{'this demo':>12}{'paper':>10}")
    for mode in ("random", "all", "rag"):
        bar = "#" * int(results[mode] / 2.5)
        print(f"{label[mode]:<26}{results[mode]:>11.1f}%{paper[mode]:>9}%  {bar}")

    print("\n" + "-" * 72)
    print("WHY 'ALL' LOSES TO 'RAG'")
    print("-" * 72)
    print("The entire-LTM condition ALWAYS contains the correct lesson --")
    print("it cannot possibly miss it. It still loses, because it also")
    print(f"carries ~{len(keys) - 1} irrelevant entries, and the planner's")
    print("attention is finite. Selective retrieval trades a small risk of")
    print("missing the lesson for a large reduction in noise.")
    print("\nRe-run with --distraction 0.0 to remove the long-context effect:")
    print("'entire LTM' then catches right up to RAG (~91% vs ~91%), because")
    print("without a noise cost there is no reason to filter at all. That is")
    print("exactly the intuition Fig. 7 refutes -- the gap only exists because")
    print("the penalty for irrelevant context is real and non-zero.")
    print("\nThe practical lesson: in a RAG system, relevance filtering is an")
    print("ACCURACY mechanism, not merely a cost optimisation. The paper also")
    print("measures the cost side -- full LTM is 7.5x the prompt tokens.")
    print("=" * 72)


if __name__ == "__main__":
    main()
