# PragmaBot × Franka FR3 — Analysis Deliverables

Produced 2026-08-18/19. **Nothing under `D:/pb/home` was modified** — this workspace was read-only throughout. All output lives here, outside your project tree, so you can move it wherever you want.

---

## The documents

| File | What it is | Read it when |
|---|---|---|
| **`00_MASTER_REFERENCE.md`** (1035 lines) | **The common doc.** Paper + project + system + data + gotchas + open questions, in one place. | **Start here**, and return here instead of re-reading the paper or re-tracing the code. |
| `01_PAPER_ANALYSIS.md` | Deliverable 1. Study output index, what "recreated" must mean, what exists in your workspace, what doesn't. | Deciding scope. |
| `02_REPRODUCTION_PLAN.md` | Deliverable 2. Gap analysis (paper vs `pragmabot_bridge`), ordered `[HOME]`/`[LAB]` steps with done-conditions, biggest risk + fallback. | Deciding what to do next. |
| `04_ROS2_MIGRATION.md` | ROS version audit + the port plan (port 515 lines, delete 648, keep 1400). | Doing the ROS 2 migration. |
| `05_HANDOFF.md` | **Resume doc.** State, rules, verified findings, next action. | Any new session or teammate — **read first**. |
| `03_PROJECT_NARRATIVE.md` | Deliverable 3. Career material — architecture, deep engineering concepts, skills, CV bullets, cover-letter paragraphs, interview Q&A, claim discipline. | Writing your CV, applying, or preparing to be interviewed. |

Paper study materials (full `study` output) are in `../papers/pragmabot/`:
`README.md` · `summary.md` · `insights.md` · `method.md` · `mental-model.md` · `qa.md` · `code/memory_retrieval_demo.py` · `paper.txt` · `paper.pdf` · `page_01..08.png` · `images/` (63)

---

## The five things worth knowing immediately

1. **Report is due 2026-09-14 — 26 days.** Grading is 30% supervisor / 40% report / 30% presentation, so **70% is communication**. This should drive every scheduling decision.

2. **Most of the paper is reproducible without the robot.** `pragmabot_node.py:139-141` skips execution entirely under `rosbag_replay: true`, and the retrieval ablation executes nothing at all. **Five of six reproduction requirements need no lab access.** Start tonight.

3. **The critical gap is a missing connection, not missing code.** The planner is ROS 1 (`panda_skill_executor.py:1-2`), the executor is ROS 2 with no exposed interface (`bridge_node.py:476-507`), and the package meant to bridge them generates nothing (`pragmabot_interfaces/CMakeLists.txt:16-22`).

4. **There is a real bug waiting.** `panda_skill_executor.py:30` reads `action.skill`, which does not exist — the field is `chosen_skill` (`vlm_task_planner.py:107`). It raises `AttributeError`, and two more accessors fail *silently* to `""`. Never caught because that code path has never run. **`PROJECT_OVERVIEW.md` §6 documents the wrong schema** — do not write the new bridge against it.

5. **Your depth fix is a genuine contribution.** It targets the paper's largest measured failure category (8/19 execution failures, 3 explicitly depth). I reproduced it independently: 17.7 cm → 7.3 cm z-extent, boundary-vs-interior outlier rates 11.9% vs 1.5%, and the full three-filter pipeline reproduces your shipped `object_pcd.npy` extent exactly.

---

## Preflight: the `study` skill

It was broken; it is now fixed and was used to produce the study output.

**Live config:** `C:\Users\haris\.claude-account1` (confirmed via `CLAUDE_CONFIG_DIR`), **not** `C:\Users\haris\.claude`.

Four defects, all fixed in `C:\Users\haris\.claude-account1\skills\study\`:

1. `${CLAUDE_PLUGIN_ROOT}` is empty for user-installed skills, and SKILL.md additionally appended a duplicated `skills/study/` path segment. → all paths rewritten to the real absolute directory (`SKILL.md.bak` keeps the original).
2. No `package.json`, and the scripts are ESM. → added with `"type": "module"`.
3. `npm install pdf-parse` pulls v2, which has no default export; pinning v1.1.1 instead hits its debug-mode self-test and dies on a missing bundled sample PDF. → pinned `pdf-parse@1.1.1` **and** changed the import to `pdf-parse/lib/pdf-parse.js` to bypass the wrapper.
4. `python3` is not on PATH on this machine. → `python`. PyMuPDF was already present.

Verified working end to end: 8 pages, 49,020 characters, no truncation, 63 images extracted.

---

## Honest status of this analysis

**Read fully, line by line:** the paper (all 1,147 lines) *and* every page as a rendered image; `bridge_node.py`; `grasp_transform.py`; `panda_skill_executor.py`; `mask_to_pointcloud.py`; `vlm_task_planner.py` schema; `vlm_success_detector.py` schema; `memory_manager.py` through `:150`; the course PDF; `CLAUDE.md`; `memory.md`; `SETUP.md`; `readme.md`; `PROJECT_OVERVIEW.md`; `docker-compose.yml`.

**Verified by re-derivation, not assertion:** artifact/contract consistency; grasp rotation validity; the gripper-base convention; per-grasp width spread; the flying-pixel root cause; exact reproduction of the shipped point cloud; the executor schema bug; `easy_handeye2` being unmodified; the `/pragmabot` mount vs colcon build-path contradiction; `foundationpose/` being empty.

**Read structurally but not line by line:** `detect_object.py`, `calibrate_extrinsic.py`, `make_object_pointcloud.py`, `mesh_gen.py`, `capture_calib_frame.py`, `test_fps.py` (partial), `WORKSPACE_STRUCTURE.md`, upstream `vlm_*.py` bodies.

**Deliberately not used as evidence:** git history — you said the folders are lab copies whose VC state is not meaningful. Every structural claim comes from files on disk.

**Could not verify:** anything requiring the robot or the lab network; the hand-eye calibration result (no `.calib` file exists in this copy); whether `fr3_hand` or `fr3_hand_tcp` is the correct `eef_link`.
