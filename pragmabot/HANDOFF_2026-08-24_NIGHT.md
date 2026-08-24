# HANDOFF — 2026-08-24, night session (~21:30–23:00)

Continues `HANDOFF_2026-08-24_EVENING.md` (same day, same lab session). Read
that one first — this file only covers what changed after it.

Report due **2026-09-14**.

---

## 0. State in six lines

- **Grasp selection went from a soft geometry preference to a hard, fail-closed
  perpendicularity gate** — the pipeline now refuses to attempt a badly-tilted
  grasp instead of trying it and failing downstream. Verified against live
  data multiple times, including one measured case where "best confidence"
  targeted a point **7cm below the robot's own base**.
- **A cube vs. cup comparison ruled out a calibration/rotation bug.** The same
  pipeline gives a cube a 2.7°-tilt grasp candidate and gives two different
  cups nothing better than 59-63°. The math/TF/calibration chain is correct;
  the cups specifically don't present a good top-down grasp from this
  camera's angle right now.
- **Arm motion (standoff + Cartesian approach) is now clean and error-free**
  for a good grasp candidate — but the gripper close itself fails ("Grasp
  reported failure", blank error = closed outside the expected width band).
- **Unresolved, and the most important open item:** twice now, Terminal A
  logged a fully successful standoff + Cartesian approach with zero errors,
  and the operator watching the robot reports **no visible arm motion**. A
  joint-state watcher is running (see §6) to get ground truth on the next
  attempt — this was left mid-diagnosis when this handoff was requested.
- **`franka_gripper_node` keeps needing manual restarts**, and separately,
  Desk's "End Effector: Not connected" is showing up often — the second one
  has a known, much lighter fix than what's been used (see §5).
- **New code all uncommitted as of the start of this handoff** — bridge_node.py,
  grasp_transform.py, live_perception.py, LAB_README_ARCHIVE.md, new
  scripts/home_pose.py.

---

## 1. Code changes this session

### grasp_transform.py — `select_grasp_index` rewritten twice

**First pass:** added `max_tilt_deg`, multiplying confidence by a continuous
top-down alignment factor. **Second pass (current):** rewritten to a hard
gate — any candidate more than `max_tilt_deg` off perpendicular is excluded
entirely before confidence is even considered, and if NOTHING qualifies the
function returns **-1** rather than falling back to the least-bad tilted
option. This was an explicit, repeated user instruction ("only look for
perpendicular or close to perpendicular grasp and filter out all others"),
not a unilateral design choice.

`min_confidence` keeps the older best-effort behaviour (ignored rather than
emptying the tilt survivors) — only the tilt gate fails closed.

Verified with 4 synthetic cases (mixed tilts, none-qualify, gate-disabled,
confidence-among-survivors) — all correct. See the function's own docstring
for the full reasoning.

### bridge_node.py

| Change | Why |
|---|---|
| **Standoff move (Step 1 of `execute_pick`) tries `_compute_cartesian_path` first**, falls back to `_move_to_pose` (OMPL) only if the straight line isn't reachable | Was plain OMPL with no collision scene — the likely cause of the original "weird trajectory to reach" complaint. Reuses the already jump-guarded, velocity-gated Cartesian machinery Steps 2/4 already used. |
| **`max_grasp_tilt_deg` parameter, default `20.0`** | The tilt gate's threshold. Briefly defaulted to `0.0` (disabled) around 22:15-22:30 at the user's explicit request, to isolate a motion-planning question from grasp quality — reverted after live data confirmed every "best confidence" failure since disabling it was this gate's exact target (see §3). |
| **`grasp_index < 0` check added after grasp selection** | `select_grasp_index` can now return -1. Without this check, Python's negative indexing would have silently wrapped to the LAST grasp in the array — a real bug caught before it shipped. Aborts with `_fail_log`, message tells the planner to reorient/reposition the object. |
| **`grasp_topk` parameter, default `0`** | Added (default 6), then **reverted to 0** the same session — see §3, it caused a real failure. |

### live_perception.py

- `LivePerception.__init__` gained `grasp_topk: int = 0`. `_generate()`
  slices to top-K by confidence client-side if `grasp_topk > 0`. NOT wired
  to the GraspGen server's own `topk_num_grasps` — that parameter was tested
  and found to not hard-cap the response (asked for 6, got 36 back;
  appears to be a per-iteration cap, not global).

### New: scripts/home_pose.py

Standalone `--record`/`--go` utility. Saves the arm's current joint pose to
`~/.ros2/pragmabot/home_pose.json`, returns to it later via a **joint-space**
MoveGroup goal (not pose-space) — deliberately sidesteps the standoff move's
IK-ambiguity failure mode since the target joint values are already known
exactly. Not wired into the planner's skill vocabulary (pick/place/push only)
— it's a manual reset between attempts, not a fourth skill. Not yet actually
used this session (the user hadn't recorded a home pose before things got
busy) — recommend doing this at the START of the next session.

---

## 2. Terminal commands used tonight (add to LAB_README_ARCHIVE.md too)

**Terminal A (bridge)** — unchanged from the runbook:
```bash
export ROS_DOMAIN_ID=7
source /opt/ros/humble/setup.bash
source ~/pragmabot_bridge_ws/install/setup.bash
export PYTHONPATH=$PYTHONPATH:$HOME/GraspGen
ros2 run pragmabot_bridge bridge_node
```

**Terminal B (VLM planner + Gradio)** — unchanged:
```bash
export ROS_DOMAIN_ID=7
source /opt/ros/humble/setup.bash
source ~/pragmabot_bridge_ws/install/setup.bash
export PYTHONPATH=$PYTHONPATH:$HOME/irm2pragmabot/pragmabot/pragmabot/src:$HOME/GraspGen
cd ~/irm2pragmabot/pragmabot
python3 pragmabot/nodes/pragmabot_node.py
```

**Gripper node restart (inside `franka_ros2_humble` container)** — used
repeatedly tonight; see §5 for why it keeps being needed:
```bash
docker exec -it -e DISPLAY=$DISPLAY franka_ros2_humble bash
ros2 run franka_gripper franka_gripper_node --ros-args \
  -r __node:=franka_gripper \
  -p robot_ip:=10.10.10.10 \
  -p joint_names:="[fr3_finger_joint1, fr3_finger_joint2]" \
  --params-file /ros2_ws/install/franka_gripper/share/franka_gripper/config/franka_gripper_node.yaml
```

---

## 3. Diagnostic findings — grasp geometry investigation

This took most of the session. Sequence of checks, each done against live
data, not guessed:

1. **GraspGen's own axis convention checked against its source** (`~/GraspGen/docs/GRIPPER_DESCRIPTION.md`): "approach direction is the positive
   Z-axis" — matches what `grasp_transform.py` already assumed. No sign bug.
2. **Point cloud position sanity-checked** in `fr3_link0` — physically
   plausible (x=0.55-0.64m forward, near table height), not garbage.
3. **Detection confidence checked directly** (it's logged via plain Python
   `logging`, not ROS's logger, so it never appeared in Terminal A's output
   all night): "orange cup" (0.641) vs plain "cup" (0.657) on the same
   frame — nearly identical, both a normal range for this pipeline's past
   correct detections. Ruled out "wrong color prompt confuses the detector."
4. **Decisive test: cube vs. cup.** Same pipeline, same session, same camera:
   - Cup (two different physical cups tried): 0/100 grasps within 40° of
     vertical, minimum tilt across ALL candidates 59-63°.
   - Cube: minimum tilt **2.7°**, 8/100 within 20°, clean pick-worthy
     candidate at 0.927 confidence / 12.2° tilt.
   
   This is the key finding: if the TF/calibration chain had a rotation
   error, the cube would show the same garbage as the cups (a rotation bug
   doesn't care about object shape). It didn't. **The pipeline's geometry is
   correct.** The cups specifically aren't presenting a good top-down grasp
   from this camera's current angle — most likely their real physical
   pose/geometry from this viewpoint, not a bug. Not fully root-caused
   (didn't get to physically re-checking cup orientation before the session
   moved to cubes).
5. **`grasp_topk` experiment, self-correcting:** added a top-K-by-confidence
   cap (default 6) to shrink the candidate pool for a cleaner picture.
   Immediately caused a real failure: the SAME cube that produced a 3.5°-tilt
   pick moments earlier aborted with "no candidate within 20 deg" once
   capped to 6, because its one near-vertical candidate ranked outside the
   top-6 by confidence. Reverted to `grasp_topk=0` same session. Lesson,
   worth remembering: **fewer candidates only shrinks the tilt gate's odds
   of finding a valid one — there is no upside to capping the pool.**

### Visual grasp inspection tooling (ad hoc, not committed)

Built a viser-based visualizer (GraspGen's own `docs`-referenced tool,
`http://localhost:8080`) fed from `/tmp/pragmabot_live_cloud.npy` +
`/tmp/pragmabot_live_grasps.npz` (the same temp files `_resolve_grasps`
writes), so a live pick's actual candidate set can be eyeballed. Scripts
live in this session's Claude scratchpad, not the repo — worth formalizing
into `scripts/` if this keeps being useful:
- fetch live data fresh (bypass the bridge, call `LivePerception.grasps_for`
  directly) — this is what caught that the visualizer was showing 20+
  minute stale data at one point; it does NOT auto-refresh.
- select and visualize a small labelled subset (best-confidence / least-tilt
  / worst-confidence) instead of all 100, for readability.
- render in the CAMERA's frame, not the robot's — a grasp can look
  reasonable there while still being severely tilted relative to true
  vertical, since the ZED isn't mounted directly overhead. Don't judge
  "is this vertical enough" from that viewer; use the tilt numbers.

---

## 4. THE OPEN QUESTION — arm motion logged as success, not observed

Twice tonight (green cube attempts, ~22:34 and ~22:52), Terminal A logged:
- grasp selection succeeded (good confidence, good tilt)
- standoff move: no error
- Cartesian approach: no error
- only the gripper CLOSE failed afterward

...and the operator, watching the robot, reported **no visible arm motion**
both times. Checked and ruled out as an explanation:
- Controller fault: `ros2 control list_controllers` showed
  `fr3_arm_controller` active, no fault, both times checked.
- `_execute_trajectory`'s code path was read in full — it does call the real
  `/execute_trajectory` action and check the real result code, no silent
  bypass found.

**Working hypothesis, not confirmed:** the arm may be moving a genuinely
small amount if it was already resting near a similar pose from a prior
attempt (nothing returns it "home" between tries — this is exactly what
`home_pose.py` was built for, but wasn't used yet). Alternative not ruled
out: some MoveIt/controller-manager mismatch reporting execution success
without the command truly reaching hardware — this would be more serious
and needs the joint-state ground truth below to distinguish.

**Left mid-diagnosis:** a joint-state watcher script was started
(background, not committed — logs `/joint_states` deltas for `fr3_joint*` to
`$SCRATCHPAD/joint_watch.log`) right as this handoff was requested. Whoever
picks this up next: run one more pick attempt, then check that log (or
re-launch an equivalent watcher) — if it shows near-zero joint movement
during a "successful" attempt, that is the smoking gun for a real
execution-reporting bug and needs to be chased immediately, ahead of
everything else in this file. If it shows real movement, the "small
displacement" hypothesis is confirmed and the fix is just: use
`home_pose.py --go` between attempts so movement is always visible.

---

## 5. Gripper: two DIFFERENT problems, don't conflate them

1. **`franka_gripper_node` process dies entirely** (`Action servers: 0`).
   Robot-side, User-Stop-triggered, documented in the evening handoff. Only
   fix: restart the node (§2 command). Happened multiple times again
   tonight.
2. **Desk shows "End Effector: Not connected" while the node is still
   alive.** This is NOT a hardware/network fault — `bridge_node.py`'s own
   code comment (written in an earlier session) already documents this:
   it's what the Franka Hand driver does after ANY failed Grasp/Move action,
   until re-homed. The bridge already re-homes automatically on a failed
   grasp (one retry). The much lighter manual fix, if needed outside the
   bridge, is just:
   ```bash
   ros2 action send_goal /franka_gripper/homing franka_msgs/action/Homing "{}"
   ```
   or Desk's own Home button — **not** a full joint-lock + reinitialize,
   which the operator had been doing all session out of not knowing this.
   Worth confirming next session whether plain re-homing actually clears it
   every time, or whether some cases still need more.

---

## 6. Still open, in priority order

1. **The joint-state motion mystery (§4)** — highest priority, could
   invalidate every "success" logged tonight if it's a real bug.
2. **Calibration z-fix still not active.** 15.7mm error, computed and on
   disk since the evening session, needs `/handeye_publisher` restarted to
   take effect. Recommended as the next lever once §4 is resolved and a
   pick gets far enough to fail on contact accuracy specifically (the green
   cube attempts were getting close to this being the limiting factor
   before the "didn't move" question took priority).
3. **Root cause of the cup-specific bad grasps** — not found, only ruled
   out (not a calibration bug). Suspect the cups' real physical pose/angle
   relative to the camera; never got back to physically checking them.
4. **No MoveIt collision scene** — still nothing, unchanged from evening
   handoff.
5. **`should_grasp_at_specific_section` still silently ignored** by the
   bridge — unchanged from evening handoff.
6. **Commit tonight's changes** — see this session's commit alongside this
   handoff file.
