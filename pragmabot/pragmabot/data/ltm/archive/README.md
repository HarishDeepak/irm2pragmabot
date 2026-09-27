# Archived: `ltm_paper_scenarios_NOT_OURS.csv`

**These four LTM entries were not produced by this robot. Do not use them as
experimental data, and do not let them back into `../ltm.csv`.**

## What they are

Four rows describing scenarios from the PragmaBot paper itself (Fig. 5b):
egg/banana, plate/apple, tiny candies + sponge-as-tool, and apple/plate with
an obstructing salt container. All four are timestamped `2026-02-07` at
suspiciously round times (08:00:00, 12:00:00, 17:30:00, 23:00:00).

No corresponding robot run exists in this project. They appear to have been
hand-authored as placeholder/demo content.

## Why they were removed (2026-08-22)

1. **Research integrity.** The graded contribution of this project is the
   memory system. Reporting retrieval results computed over fabricated
   memories — without disclosure — would be reporting invented results.
   The course report is examined; this is not a risk worth taking.

2. **They crashed retrieval anyway.** There is no matching
   `ltm_<embedding-model>.csv` embeddings file, so `MemoryManager.load()`
   takes the `elif` branch (memory_manager.py:73-76), logs "Found LTM
   entries but no embeddings", and sets the whole `embedding` column to
   `None`. `retrieve_relevant_experiences` then calls `cosine_similarity`
   with `a=None`, and `np.linalg.norm(None)` raises
   `TypeError: unsupported operand type(s) for *: 'NoneType' and 'NoneType'`.
   Verified by direct reproduction, not assumed. So with
   `activate_ltm: true`, the very first retrieval died.

   Note `cosine_similarity` DOES guard the zero-norm case, and NaN passes
   through harmlessly — it is specifically `None` that is unguarded.

## What replaces them

`../ltm.csv` is now empty apart from its header, which gained a
**`provenance`** column. Every future entry must declare where it came from:

| provenance | meaning |
|---|---|
| `real_robot` | a genuine execution on the FR3 |
| `replay` | a genuine full VLM loop run against a rosbag (`rosbag_replay: true`) — real images, real reasoning, real summarisation; only the arm motion is absent |
| `authored` | hand-written prior. Permitted, but must be reported separately |

Recording provenance in the data (rather than in a footnote) means results
can be split by it. That turns a liability into a finding: "authored priors
retrieved at similar rates but produced lower task success than
replay-derived entries" is a real result about memory quality.

## The crash is DORMANT, not fixed

An empty `ltm.csv` is safe: `self.df["embedding"].apply(...)` over zero rows
never calls `cosine_similarity`, so nothing raises (verified).

**But the bug is still there.** It re-arms the moment `ltm.csv` has a row
whose embedding is missing — i.e. any time `ltm.csv` and
`ltm_<model>.csv` fall out of step. Two ways that happens in normal use:

- `save_experience()` writes a row but the embeddings write fails or is
  interrupted;
- the `text_embedding_model` config value changes, so
  `MemoryManager.__init__` looks for a *differently named* embeddings file,
  finds nothing, and every existing row gets `embedding = None`.

The second is a live hazard right now: `config.yaml` declares
`text_embedding_model: text-embedding-3-large` while the Claude path
actually produces 384-d `all-MiniLM-L6-v2` vectors, so the embeddings file
is named after a model that never computed them.

The root-cause fix is a `None` guard in `MemoryManager.cosine_similarity`
(it already guards the zero-norm case). **That file is on the project's
never-modify list** (CLAUDE.md), so it has deliberately been left alone.

If the `TypeError` above appears, the cause is a row without an embedding —
not a corrupt file. Either re-embed the LTM or clear it; do not "fix" it by
deleting the error message.

## Bootstrapping honestly

Replay-mode runs are the highest-value source given scarce robot time: they
need no robot, they exercise the entire graded code path, and every entry
they produce is genuine. Mark them `replay` and say so in the report.
