# HANDOFF — read this first (2026-08-21)

For a fresh Claude Code session (or teammate) picking this project up cold.

## Where to work

```
cd D:\irm2pragmabot
code irm2.code-workspace
```

That's it — this is the **only** working copy now. It's a real git clone
(`origin` = `HarishDeepak/irm2pragmabot`, public), already has the GraspGen
checkpoints (~2.5 GB) and both venvs built (`GraspGen/.venv`), and the
`irm2.code-workspace` file here defines 7 sidebar folders (repo root +
6 subfolders) each with its own integrated-terminal profile pre-wired
(Control/docker, ZED/host, GraspGen venv, GroundedSAM venv, pragmabot
bridge) — click a folder, open a terminal, it's already `cd`'d and
sourced correctly. Don't hand-cd around; use the workspace.

Day-to-day git: see `GIT_ADVICE.md` in this same folder.

## What NOT to use

- **`D:\pb\home`** — old, pre-consolidation copy. Kept on disk only for
  its original bag file and history; not a git repo at top level (has
  4 separately-git-tracked nested repos inside it, all reconciled into
  this repo already — see `D:\pb\home\ARCHIVED_README.txt`). Don't edit
  files here; nothing you change here will go anywhere.
- **`D:\pb\IRM2_ARCHIVE_old`** — a different, older repo (remote
  `HarishDeepak/IRM2.git`, branch `feature/fr3-host-adaptation`), renamed
  from `D:\pb\IRM2` on 2026-08-21 because it predates this consolidated
  repo. Had uncommitted local changes (`readme.md` modified, `extras/`
  untracked) at archive time — nobody has confirmed those are worthless,
  just that this location isn't the active one. If something turns out
  to be missing from the new repo, check here before assuming it's gone.
- **`D:\pb\irm2-repo`** — deleted 2026-08-21. Was a byte-identical clone
  (same commit `6d1b896`) of this repo, just missing the venvs/checkpoints.
  Nothing lost.
- **`D:\pb\irm2.code-workspace.OLD_DO_NOT_USE`** — stale workspace file
  that pointed at `D:\pb\home\*`. Superseded by this folder's
  `irm2.code-workspace`. Renamed, not deleted, in case anything in its
  `settings` block is still wanted.

## Config that does NOT carry over automatically

`ros2_ws/franka_ros2/franka_bringup/config/franka.config.yaml` in this
repo has **public placeholder values** (this is a public GitHub repo —
don't put real robot IPs in it). The old `D:\pb\home` copy had the real
lab values: `robot_ip: "10.10.10.10"`, `namespace: "NS_1"` → `""`,
`load_gripper`/`use_rviz` both `"true"`. Set these locally (untracked,
or in a gitignored override) before running against the real arm.

## Pending decision: Armin's grasp-execution files

Armin (colleague) sent 4 new files + a written explanation on 2026-08-21,
sitting in `D:\Downloads` (NOT yet in this repo):
`grasp_pipeline (1).py`, `moveit_client (1).py`, `moveit_server (1).py`,
`Pragmabot summary.txt` (the "(1)"-suffixed ones have the real content —
the non-suffixed duplicates are 0-byte Telegram download artifacts, ignore
them). These implement exactly the piece this project's own pipeline
diagram calls the blocker: grasp pose (camera frame) → apply camera→robot
calibration → HTTP → MoveIt execution on the FR3.

**Assessed 2026-08-21: useful, worth merging, not over-engineered** — it
follows the repo's existing cross-venv subprocess pattern, and the
HTTP-instead-of-ros1_bridge choice is justified for a single request/
response type rather than dogmatic. Two real gaps before it's usable:
1. `grasp_pipeline.py::_run_graspgen()` — placeholder parsing; needs
   `~/GraspGen/.venv/bin/python ~/GraspGen/client-server/graspgen_client.py
   --help` output to fill in the real output schema.
2. `moveit_server.py` — untested against the real `move_group`/gripper
   actions (has a `PRAGMABOT_DRY_RUN=1` mode for a safe first pass).

**Not yet wired into this repo**: his summary describes editing
`config/config.yaml` (new `execution:` section) and
`pragmabot/pragmabot/nodes/pragmabot_node.py` (replacing a
`NotImplementedError` in `execute_action()`) — neither edit is present
in this repo yet (checked via grep 2026-08-21). Merging means applying
those two edits by hand, not just copying the 4 new files in. Correct
destinations per Armin's own reasoning: `moveit_client.py` →
`pragmabot/pragmabot/src/pragmabot/`, `moveit_server.py` →
`ros2_ws/franka_ros2/pragmabot_moveit_server/` (only that subtree is
Docker-bind-mounted into the container; a `pragmabot/` mount he
referenced doesn't exist on this machine).

## Also read

- `extras/analysis/05_HANDOFF.md` — the paper-reproduction / course-project
  handoff (report due 2026-09-14, course rules, what's verified vs. not
  on the *robot/paper-reproduction* side). Different scope than this file:
  that one's about the research project state, this one's about the repo/
  workspace plumbing. Note its Hard Rule #1 ("D:/pb/home is READ-ONLY")
  is from an earlier, narrower paper-writing task — superseded here by the
  explicit 2026-08-21 decision to archive `home` in place, but the spirit
  holds: don't develop in `home`, only in `D:\irm2pragmabot`.
- `GIT_ADVICE.md` — day-to-day git commands, branch-per-person workflow.
- `SETUP_LAPTOP.md` — environment build steps if setting up a new machine.
