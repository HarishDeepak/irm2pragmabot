# Ablation experiments — recording protocol

Results sheets live here as CSV (open in any spreadsheet). One row per trial.
The auto-generated per-run log is `pragmabot/pragmabot/data/logs/<trial_id>.jsonl`
— copy the `trial_id` into the sheet so the two can be cross-referenced.

Reproduction target: **the shape of the effect on our own platform with our
own baseline**, not the paper's absolute numbers (different arm, camera,
grasp stack, VLM snapshot).

---

## General rules (all tests)

- **Success is judged by the human running the trial**, with their own eyes —
  not the VLM success detector (it has false positives). Write SUCCESS / FAIL
  in the sheet yourself.
- **One run = one trial = one attempt.** Start a *fresh* task each time (never
  continue a previous one — a stale task leaves failures in STM).
- **Fixed layout.** Draw the table layout on paper and photograph it before
  starting a scenario. Reset to that exact layout before every trial.
- `save_to_ltm: false` for every eval run, always — keeps the LTM set frozen
  and stops eval runs polluting it.
- After any `config.yaml` change, **restart `pragmabot_node.py`** (config and
  both LTM CSVs load once at startup).
- Fill `#steps` from the run (or read `finish` line of the `.jsonl`).
  A run that hits `max_steps` without completing = FAIL, steps = the cap.
- `#steps` is only meaningful for SUCCESS rows; for FAIL rows it is just the cap.

## How to compute the numbers

Per condition:
- `success_rate = successes / trials`  (5 trials → each worth 20 points)
- `mean_steps` = average `#steps` over the **SUCCESS** rows only

Report like: `STM off: 20% (1/5), 10.0 steps  |  STM on: 80% (4/5), 3.5 steps`

If the gap is obvious at 5 trials, that is enough for a first result. If it is
close (e.g. 40% vs 60%), rerun the condition at 10 trials.

---

## Test 1 — STM self-reflection ablation

Paper Table II: 35% -> 84%. Measures **within-task recovery**: when an action
fails, does feeding the failure reason back into the planner (STM) let it
replan and finish the same task, versus repeating the failed action until the
step cap.

**Config matrix** (`config.yaml`):

| condition | activate_stm | activate_ltm | save_to_ltm | trials |
|-----------|--------------|--------------|-------------|--------|
| baseline  | false        | false        | false       | 5      |
| stm       | true         | false        | false       | 5      |

Sheet: `test1_stm_ablation.csv`

### Scenario S1 — ambiguous-detection recovery (do this first)

The cleanest STM test: the failure is 100% deterministic and the fix is pure
planning (no flaky hardware).

- **Layout:** two green cubes *touching* each other near table centre; one blue
  bowl ~20 cm away. NOTHING else green on the table (no green pepper, no
  green-LED device — "green" is an overloaded detector prompt).
- **Instruction:** `pick up the green cube and put it in the blue bowl`
  (leave it as "the green cube" — ambiguous on purpose; either cube in the
  bowl counts as success).
- **Success:** a green cube resting inside the blue bowl, nothing else knocked
  off the table, within 10 steps, no human touching the scene mid-run.

Expected:
- **STM off:** step 1 `PICK "green cube"` -> `ambiguous detection ... margin <
  0.15` abort. Steps 2-10: planner has no memory of the abort, re-issues the
  same PICK every step -> `max_steps` FAIL.
- **STM on:** step 1 same abort. Step 2: abort reason is in the history ->
  planner adds a spatial prefix (`"left green cube"`) -> succeeds -> PLACE ->
  SUCCESS in ~3-4 steps.

**Check the first STM-off run:** if the scene describer flags "two identical
green cubes" and the planner adds the prefix *preemptively on step 1* and
succeeds, S1 gives no STM signal — switch to S2.

### Scenario S2 — obstruction (fallback / second scenario)

- **Layout:** target cube (e.g. yellow) with a box or sponge pushed up against
  it on the approach side; a bowl ~20 cm away; a spare empty container for the
  obstacle.
- **Instruction:** `pick up the yellow cube and put it in the blue bowl`
- Expected STM off: the first `PICK`/approach on the blocked cube fails, and
  the planner keeps retrying the blocked pick -> FAIL. STM on: reads the
  failure, clears the obstacle first, then picks -> SUCCESS.

---

## Test 2 — LTM + RAG ablation (later)

Paper Table III: 22% -> 80%, single-trial, on **unseen** scenarios.
`activate_stm: true` in BOTH arms; toggle only `activate_ltm`.
See ARMIN.md "RECOMMENDED STARTING POINT" for the seeded/unseen stacked-cube
scenarios. Sheet: `test2_ltm_ablation.csv` (create when starting Test 2).

> Confound to resolve before Test 2 is clean: `vlm_task_planner.py` currently
> carries a "clear a graspable obstacle" HARD CONSTRAINT and the `_holding`
> guard in `panda_skill_executor.py` — both encode lessons LTM is meant to
> supply. For a faithful LTM ablation, revert the planner prompt to
> `vlm_task_planner.py.upstream-bak` and disable the `_holding` guard for the
> LTM-off arm.
