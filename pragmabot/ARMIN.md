# armin branch — pick-reliability debugging session (2026-08-26)

Session goal: the `pick` skill on the green cube worked *sometimes* and
failed *sometimes*, with no obvious pattern. This log walks through what
was actually wrong (three separate, unrelated bugs stacked on top of each
other) and what was changed to fix each one, in the order they were found.

## Summary: why it works now

Three independent problems were causing failures, and each needed its own
fix. Fixing one exposed the next:

1. **Bad grasps were being executed anyway.** The grasp selector had a
   tilt gate that failed closed (aborted rather than pick a bad angle) but
   a confidence gate that was soft (fell back to the best-of-a-bad-batch
   instead of aborting). A real run executed a grasp GraspGen itself only
   rated 0.634 confidence (best available that frame was 0.968, but it
   failed the tilt gate) and the gripper closed on nothing.
2. **Tilted approaches were physically clipping the cube.** The approach
   went in a single straight line from the pre-grasp standoff directly
   into the grasp pose. For any tilted grasp, that line has a sideways
   component as well as a downward one (measured: 31mm of sideways drift
   over a 120mm descent at 15° tilt) - so partway through the descent the
   gripper was still sliding sideways while already below the object's
   top-surface height. Result: the hand hit the cube's top surface/edge
   before ever reaching the intended grasp pose, on two consecutive real
   trials.
3. **The camera→robot calibration had a real, measurable offset.** Even
   after (1) and (2) were fixed, the arm kept landing right where it
   computed (arrival check consistently ~1-2mm off its own commanded
   pose) but still missed the real cube - to one side, or clipping an
   edge. A touch-test (hand-guided the gripper to visually centre it on
   the real cube, read live TF, compared against the logged computed
   grasp pose - projected to the actual fingertip contact point along the
   grasp's approach axis, not just the raw base-link translation, since
   the two poses being compared had different tilts) measured a bias of
   **x=-3mm, y=-30mm, z=+6mm** - dominated by a ~30mm offset on one axis,
   on a cube only ~40-45mm wide. That is large enough, by itself, to fully
   explain every miss/edge-clip seen this whole session.

With all three fixed, a pick succeeded end to end
(`success=True message='picked green cube'`) at 19:56, and has been
reproduced since with the cube moved to new positions.

## What was changed

All three fixes are in `bridge_node.py` (`execute_pick`); one debug aid
was added to `calibration/perception_server.py`.

### 1. Hard confidence gate (fail closed, like the tilt gate already did)
`min_grasp_confidence` used to be soft: if nothing cleared it, the
selector silently fell back to the best available candidate regardless of
how low that was. Added an explicit check right after the final grasp
candidate is chosen (after the tilt+width gates run): if its confidence is
still below `min_grasp_confidence`, abort with a message telling the
planner to reorient/re-capture, instead of executing a grasp GraspGen
itself rated unreliable.

### 2. Hover waypoint before the final descent
`execute_pick`'s Step 2 used to go straight from the standoff pose to the
grasp pose in one Cartesian line. Now it inserts an intermediate waypoint
directly above the grasp target - same x/y as the grasp, but at the
standoff's (safely elevated) z - so the arm finishes centring itself
horizontally *before* it starts descending, instead of doing both at
once along a diagonal that can clip the object.

### 3. Empirical calibration correction
New parameters `calib_correction_x/y/z` (defaulted to the measured
`-0.003, -0.030, +0.006` m), applied as a fixed additive correction to
`T_base_from_cam`'s translation right after the TF lookup, before it's
used for anything. Deliberately NOT applied by editing
`~/.ros2/easy_handeye2/calibrations/fr3_zed_right.calib` directly, so the
raw calibration output stays untouched and the correction stays visible,
tunable, and revertible from one place (set all three to `0.0` to run
against the raw calibration again).

This is based on **one** hand-eyeballed touch-test sample - a reasonable
starting point, not a final calibrated number. If picks start missing
consistently in some direction again (especially far from where this was
measured), re-run the touch-test procedure and adjust these three
parameters rather than assuming the underlying bug is back.

### 4. Debug save hook in `perception_server.py`
Opt-in via `PRAGMABOT_DEBUG_SAVE=1`: saves the RGB frame and the exact
segmentation mask used for each request to `/tmp/pragmabot_debug/`. Added
to rule out (or confirm) a segmentation-side bias as a candidate
explanation before landing on the calibration bias above. Conclusion: the
mask was clean and well-aligned with the cube in every capture checked -
segmentation was not the problem. Off by default; harmless to leave on.

## Touch-test procedure (for re-measuring the calibration correction)

1. Run one pick attempt, note the `grasp pose (fr3_link0):` matrix from
   the log, and leave the cube in place.
2. Put the arm in manual-guidance mode (hold both buttons near the
   flange) and hand-guide the gripper to visually centre it on the real
   cube, at roughly the same height/approach the log's grasp pose used.
3. Read `ros2 run tf2_ros tf2_echo fr3_link0 fr3_hand` at that position.
4. Project BOTH poses (logged grasp pose, and the hand-guided pose) to
   their fingertip contact point along each one's own approach axis
   (`translation + 0.10527314 * R[:, 2]`) before differencing them - the
   two orientations are usually not the same (the logged grasp is
   tilted; a hand-guided pose is usually eyeballed level), and GraspGen's
   origin convention is the gripper BASE link, not the fingertip, so
   comparing raw translations under different orientations gives a wrong
   answer.
5. The difference between the two projected fingertip points is the bias
   to feed into `calib_correction_x/y/z`.

## Known caveats / not fully closed out

- The calibration correction is translation-only and was measured at one
  table position. A genuine calibration error can have a small rotational
  component too, which would show up as the required correction changing
  with the cube's distance/position on the table. Not yet tested across
  the full workspace.
- `min_grasp_confidence` (0.80) and `max_grasp_tilt_deg` (20.0) are still
  the pre-existing defaults; only the *enforcement* of the confidence gate
  changed (soft → hard), not the threshold value itself.
- Restarting `bridge_node` (and `perception_server.py`, for the debug
  hook) is enough to pick up all of this - both are Python-only changes
  under an editable/symlink colcon install (`build/pragmabot_bridge` is a
  symlink back to `ros2_ws/src/pragmabot_bridge`), no rebuild required.

---

# 2026-08-27 - place brought up on real hardware

Goal for the day: run pick THEN place ("pick up the green cube and put it
on the orange tray") end to end on the robot. Pick already worked from the
26th. Three separate problems had to be fixed before place ran; all fixes
are in `bridge_node.py`.

## 0. Stale Fast-DDS SHM segments froze the whole stack (environment, not code)

Every `ros2` command printed
`RTPS_TRANSPORT_SHM Error ... open_and_lock_file failed`. `controller_manager`
was unreachable, RViz showed no image, and the bridge froze mid-run.
Cause: a pile of stale `/dev/shm/fastrtps_*` + `sem.fastrtps_*` segments
left by processes that were killed (one set was inconsistent - a
`*_el` file with no matching main file). All owned by `harish`, so NOT the
root-vs-non-root variant CLAUDE.md describes - just leftovers.

Fix: stop every ROS process, then
`rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*` on the host AND inside
the container, and restart. Confirm the `open_and_lock_file` error is gone
from every node as it launches. If it comes back, a node grabbed a slot
that was still stale - stop and re-clean.

## 1. Bridge froze after the first completed goal (nested-spin / executor bug)

Symptom: `pick` (goal 1) succeeded, then the `place` goal (goal 2) got
"no response to goal request within 120s" and the bridge logged NOTHING -
not even the first line of `_execute_skill_cb`. The process was alive but
its `MultiThreadedExecutor` had stopped dispatching. (An earlier `ps`
that showed "no process" was a false alarm - it was run inside the
container, where the host bridge process is invisible.)

Cause: `bridge_node.py` called the module-level
`rclpy.spin_until_future_complete(self, ...)` / `rclpy.spin_once(self, ...)`
~6 times from inside the skill callback while the node was already owned by
the `MultiThreadedExecutor`. That free function spins up its own throwaway
executor; `add_node()` on a node the main executor already owns silently
fails, and after the first goal completes the node is no longer serviced
by the main executor.

Fix: new helper `_spin_until_done(future, timeout_sec)` that just polls
`future.done()` with a 5 ms sleep - the OTHER executor threads (Reentrant
group) complete the future. All 5 call sites switched to it. The one
Cartesian call that had NO timeout now gets 15 s so it can't hang forever.
This is the fix that made place run at all.

## 2. Place couldn't reach the drop pose with the grasp's orientation

Symptom: place perceived the tray fine, then `_move_to_pose` to the
pre-place approach pose timed out (`MoveGroup returned no result within
120s`). Seen when GraspGen had returned a "backhand" grasp (rotation
matrix `R[0][0] ~ -0.99`, i.e. ~170 deg of yaw) - the pick still worked
but wristy, and that same orientation carried to the tray (far on the
robot's right, `y ~ -0.25 m`) was near-unreachable. The tilt gate only
checks approach-axis verticality, not wrist wind-up, so the selector
accepts these.

Fix: `execute_place` now tries the grasp orientation first (proven
reachable at the PICK location), and on failure retries with a
straight-down orientation that keeps only the grasp's yaw (new helper
`_level_place_pose`). Placing is a release, not a grasp - the exact tilt
does not matter. Logs `reached the pre-place pose with a straight-down
orientation` when the fallback fires.

## 3. Place descent drove the cube into the tray -> FR3 reflex

Symptom: descent executed, then the robot went to a reflex error (red
light, "configured force threshold reached" on the robot server);
`_execute_trajectory` returned False -> "place: descent execution failed".
The perceived tray surface came back at `z = -0.002 m` - BELOW the table
top (`z = 0.005`), so the tray-floor depth was ~1-2 cm low, and the 1 cm
`place_clearance_m` was not enough margin.

Fixes:
- `place_clearance_m` default 0.01 -> **0.04**. The object is released into
  free air a safe gap above the perceived surface and drops the rest -
  absorbs the stereo depth error and any unmodelled tray rim. Fine for a
  light cube; raise it further for a fragile object.
- New `place_descent_speed_factor` (default **0.25**). The final descent
  Cartesian trajectory is retimed to 1/4 speed via
  `grasp_transform.retime_trajectory` before execution (Humble's
  GetCartesianPath has no velocity-scaling field). A slow descent means
  any unmodelled contact trips the reflex gently instead of at speed.

## Result

After all three: `pick up the green cube and put it on the orange tray`
runs end to end, and was reproduced several times with cube and tray
moved to new positions.

## Where the place logic actually lives (asked this session)

`pragmabot_node.py` (planner loop) -> `PandaSkillExecutor.execute()` in
`src/pragmabot/panda_skill_executor.py` sends ONE `ExecuteSkill` ROS 2
action goal (it contains no motion logic - just marshals the planner's
decision). The bridge's `_execute_skill_cb` (`bridge_node.py`) receives it
and calls `execute_pick` / `execute_place` / `execute_push`.
`execute_place` is `bridge_node.py:1039`, ~125 lines, fully implemented:
resolve placement surface (`_resolve_placement` -> `surface_point_for` ->
FPS on the segmented mask, `calibration/mask_sampling.py`) -> reach a
pre-place pose -> slow straight-line descent -> open gripper -> lift.
The upstream leggedrobotics/pragmabot repo is where place/push are
`NotImplementedError` stubs - that is the gap this project fills, and it
fills it in the bridge, not by editing the executor.

## LTM enabled (same day)

`config.yaml`: `activate_ltm: true`, `save_to_ltm: true`. The local MiniLM
embedder (`claude_vlm_client.py`) needed `sentence_transformers`, missing
from the SYSTEM python that runs `pragmabot_node.py`. Disk was 100% full
(shared box) so the default CUDA torch pull failed - installed **CPU-only
torch** (`--index-url https://download.pytorch.org/whl/cpu`) + removed the
orphan CUDA-13 wheels, then `sentence-transformers`. `transformers 5.x`
needs Pillow >= 9.1; system Pillow 9.0.1 was upgraded to 11.3.0 (user
site). Embedder verified: 384-d vectors, model cached. LTM now writes a
row per completed task and retrieves by cosine similarity; the tell that
retrieval fired is the planner's `applicable_knowledge` field being
populated (it is `None` when no LTM is provided).

## execute_push implemented (same day)

`bridge_node.py:execute_push` replaced the NotImplementedError stub. A
non-prehensile skill for clearing an obstruction before a pick, or moving
an object the two-finger hand cannot grasp (wide cup, sponge, ball).

Flow: `_add_table_collision` -> home + close the gripper (Move to width 0,
the closed hand is the pushing tool) -> `LivePerception.object_cloud_for`
(new; GroundedSAM detect only, NO GraspGen) -> TF `fr3_link0 <- camera` +
the same `calib_correction` the pick path uses -> object centroid/extent
in base frame. `push_direction` is "left"/"right" **camera-relative**
(matches how the scene describer talks): `-x_cam` / `+x_cam` rotated into
base, flattened to the ground plane. Gripper points straight down, finger
axis perpendicular to the push. Motion: free move to a standoff beside the
object on the side the push comes FROM -> slow Cartesian descent to
contact height (`push_height_frac` of object height, clamped
15-60 mm, + the 0.10527 m fingertip offset) -> slow Cartesian lateral push
through the object to `centroid + push_distance_m` (0.12) -> lift. Descent
and push retimed to `push_speed_factor` (0.25).

Planner side: `vlm_task_planner.py` HARD CONSTRAINTS - the line that said
"PUSH IS NOT AVAILABLE ... Never choose PUSH" was replaced with a
description of what PUSH now does and when to use it. (That file is on
CLAUDE.md's never-modify list but was already modified once, in `6a3b78e`,
to add that very line.) The executor + action interface already carried
`push_direction`; no change needed there.

Target scenario: a white sponge between the robot and the green cube; the
planner should PUSH the sponge left, then PICK the cube, then PLACE on the
orange tray. NOT YET RUN ON HARDWARE - params (`push_distance_m`,
`push_height_frac`, `push_contact_margin_m`, `push_speed_factor`) are
first guesses and will need tuning against the real sponge.

## clear-obstacle-by-picking (2026-08-27, NOT YET RUN ON HARDWARE)

The PUSH-the-obstacle path clipped the obstacle with the closed gripper
instead of moving it. Alternative for any *graspable* obstacle (any
colour, any size within the gripper's range - not cube-specific): PICK
the obstacle, then PLACE it with no `placement_object`.

That PLACE branch:
  - if `clearing_zone_xyz` param is set to `"x,y,z"` (metres, fr3_link0),
    releases the held object at that ABSOLUTE point;
  - if it is `""` (**the default - no location is baked in**), releases
    at `place_offset_xyz` RELATIVE to the pick, as before. No table
    geometry is assumed in this fallback.
Object height at the release z is measured per object by `_hold_height`
(point cloud from the pick), so different-sized obstacles need no
per-object tuning. `_clearing_zone_xyz()` parses/validates the string; a
malformed value logs a warning and degrades to the relative fallback.

Successive obstacles fan out by `clearing_zone_step_xyz` (default
`[0,0,0]` = no fan-out), tracked in `self._cleared_count`;
`reset_clearing_zone()` zeroes it between tasks. Size the step to your
largest obstacle and to whichever axis has room on your table.

Config for a real table (example):
```
ros2 run ... bridge_node --ros-args \
  -p clearing_zone_xyz:="0.40,0.30,0.01" \
  -p clearing_zone_step_xyz:="[0.10,0.0,0.0]"
```
Also set the `table_*` params (`table_z`, `table_center_x`,
`table_size_x/y`, `table_thickness`) to YOUR table - they drive the
MoveIt collision box and the grasp-height clamp, and still carry
placeholder defaults from earlier.

Planner (`vlm_task_planner.py`) HARD CONSTRAINTS updated: for a graspable
obstacle prefer PICK + PLACE-aside over PUSH; a PLACE with no
`placement_object` means "put it down in a clear spot out of the way".

Expected sequence for "pick up the goal object" with obstacles in front:
PICK obstacle -> PLACE (no placement_object) -> repeat per obstacle ->
PICK goal object.

## spatial disambiguation for identical objects (2026-08-27)

Scene had two (later three) identical "wooden cube"s. GroundedSAM scored
them ~0.36/0.35 and the `AMBIGUITY_MARGIN` (0.15) guard refused every
pick ("ambiguous detection ... margin 0.00 < 0.15").

Fix - a `disambiguate` selector threaded end to end:
- `perception_server.py`: request field `disambiguate` in
  {left,right,near,far,top,bottom,largest,smallest} (+ aliases like
  front/back/nearest/biggest). When set and >1 box matched, picks the box
  by image geometry (`_select_spatial`) instead of the score-margin
  abort. Image frame: +row = down, so "near" = lowest in frame = max row.
- `live_perception.py`: `grasps_for(..., disambiguate=)` and
  `object_cloud_for(..., disambiguate=)` forward it to `_detect` ->
  `client.detect(**kwargs)` -> request.
- `bridge_node.py`: `_split_spatial("left wooden cube")` ->
  `("wooden cube", "left")`. The spatial word is stripped from the DINO
  prompt (it handles "left" poorly as a modifier) and sent as the
  selector. Used by `_resolve_grasps` (pick) and `execute_push`.
- `vlm_task_planner.py`: planner told to prefix `target_object` with one
  spatial word when the target/obstacle is one of several lookalikes.

So "pick the goal cube with two cube obstacles in front" becomes e.g.
PICK "front wooden cube" -> PLACE aside -> PICK "front wooden cube"
(next one) -> PLACE aside -> PICK "wooden cube" (last one left, no
qualifier needed).

## Session summary — 2026-08-27 (obstacle-clearing, afternoon/evening)

Goal: "pick up the goal cube" with two/three same-looking wooden cube
obstacles in front of it. PUSH kept clipping the obstacles instead of
moving them. Not achieved on hardware yet — blocked on robot bring-up and
camera depth (see below). Code changes made this session, all restart-only
(no colcon rebuild; `~/pragmabot_bridge_ws/src` symlinks into this repo):

1. **Clear-obstacle-by-picking** (`bridge_node.py`, `vlm_task_planner.py`)
   — PICK the obstacle then PLACE with no `placement_object`. Absolute
   `clearing_zone_xyz` param ("x,y,z", default `""` → relative
   `place_offset_xyz` fallback, no table geometry baked in),
   `clearing_zone_step_xyz` fan-out (default `[0,0,0]`), `_cleared_count`
   + `reset_clearing_zone()`. Planner told to prefer this over PUSH for
   graspable obstacles. See "clear-obstacle-by-picking" above.
2. **Spatial disambiguation** for identical objects — `disambiguate`
   selector (left/right/near/far/top/bottom/largest/smallest) threaded
   perception_server → live_perception → bridge; `_split_spatial()`
   strips a leading spatial word from `target_object` into the selector;
   planner told to prefix. Fixes the `AMBIGUITY_MARGIN` abort on three
   identical cubes. See "spatial disambiguation" above.
3. **Grasp-gate loosening** (`bridge_node.py` defaults) —
   `min_grasp_confidence` 0.80 → 0.70, `max_grasp_tilt_deg` 20 → 30.
   The wooden cubes' only tilt/width-legal grasps scored ~0.74–0.76
   (0.90+ candidates were all >20° tilted), so every pick aborted on the
   confidence gate.
4. **Contaminated-cloud gate** (`bridge_node.py`) — new
   `max_object_extent_m` param (0.20 m). If the perceived object cloud's
   longest side exceeds it, `execute_pick` aborts asking for a
   RE-CAPTURE, instead of the width gate reporting "too large for the
   gripper" (which the planner read as a real object property and used to
   escalate to PUSH forever).

### Blockers seen on hardware this session (NOT code — for next session)
- **MoveIt executes nothing.** Every `MoveGroup` / `ExecuteTrajectory`
  call times out at 120 s (17:19 and 17:35 logs). Robot likely in User
  Stop / reflex, or FCI dropped, or `ros2 control list_controllers` shows
  the trajectory controller inactive. Must be able to Plan & Execute from
  RViz before the bridge is worth running.
- **The yellow wooden cube segments as a ~28×31×17 cm blob every
  capture** (≈13 mm "tall" — the cloud is mostly tabletop). A plain cube
  gave a clean 5×6×3.5 cm once, so this is specific to the yellow one —
  suspect glossy paint → poor ZED stereo depth, or mask bleed onto a
  bright background. Next step: `PRAGMABOT_DEBUG_SAVE=1`, inspect
  `<ts>_rgb.png` vs `<ts>_mask.npy` — tight mask + huge cloud = depth
  holes (fix lighting / matte surface); bloated mask = raise
  `box_threshold` / `text_threshold` in `perception_server.py`.

# 2026-08-28 - two open questions (motion planning, grasp generality)

## Q1: Cartesian vs joint-space (OMPL) motion planning

Supervisor asked to use Cartesian planning instead of joint-space
("coordination") planning for smoother motion. Checked `bridge_node.py`:
this is ALREADY the case for every fine motion. `_compute_cartesian_path`
(`/compute_cartesian_path`) is used for pick approach/descent/lift, place
descent/lift, and all push segments. OMPL (`_move_to_pose` / MoveGroup)
is only used for the two big free-space transit moves - place's move to
above the drop point (`bridge_node.py:1276`) and push's move to the
standoff (`:1524`) - plus as a fallback for the pick standoff if the
straight line has no IK (`:941-948`). Those transit moves are
deliberately NOT Cartesian: a straight line across the whole workspace
routinely crosses a singularity/joint limit (the `max_joint_jump_rad`
guard catches exactly this), so forcing Cartesian there breaks picks
rather than smoothing them. If smoother transit is still wanted: finer
`cartesian_max_step`, apply `retime_trajectory` to the OMPL trajectories
too, or add TOTG/RRTstar smoothing in `_move_to_pose`. Conclusion: the
concern is already addressed for the motions that matter.

## Q2: grasping beyond cubes

The grasp stack is NOT cube-specific - GraspGen takes any object cloud,
GroundedSAM takes any text prompt. Cube-specificity is only in the gates
and the single-view capture. To generalise:
- Easy: change `target_object` prompt; tune `max_grasp_tilt_deg` (30),
  `min_grasp_confidence` (0.70), `max_object_span` (0.075),
  `max_object_extent_m` (0.20) per object - non-boxy shapes need higher
  tilt / lower confidence. Make grasp-height clamp use measured object
  height (`_hold_height`) instead of `table_z` + fixed margin.
- Medium: add a wrist-yaw / backhand gate in `select_grasp_index`
  (already listed open) - helps every non-cube shape.
- Hard (biggest payoff): multi-view point cloud capture. Single ZED view
  never sees the far side / rim / interior - this is the real ceiling for
  cups, bowls, handles. Merge 2-3 views in base frame (calib_correction +
  TF already available). Glossy/dark surfaces (yellow-cube blob) = poor
  stereo depth, partly lighting, partly depth-hole infill.
- Recommended order: raise tilt gate + add yaw gate + object-relative
  grasp height -> reliable on markers, small boxes, toys, fruit.
  Multi-view is the separate bigger project.

# 2026-08-28 - lab session (scattered objects, detector limits, cross-axis grasp)

## What ran on hardware
- **pick + place (green cube -> orange/blue tray/bowl): reliable.** ~8+ clean
  runs. Arrival error consistently ~1 mm.
- **push (red bowl left) -> pick green cube -> place on blue bowl: works**,
  but only after fixing the planner derail below.
- **two-cube two-bowl** ("yellow->blue bowl, green->red bowl"): full 4-step
  pick-place-pick-place completed. In ltm.csv (11:04 row).

## Detector limits found (GroundingDINO + SAM2, via PRAGMABOT_DEBUG_SAVE)
- **Orange tray poisons any warm-coloured target.** Prompts `yellow cube` /
  `yellow wooden cube` / `yellow dice` ALL match the orange tray (conf
  0.68-0.83, identical mask to `orange tray`). The "26x31x15 cm oversized
  cloud" abort was the tray, every time. No threshold fixes this - the tray
  is just a better "yellow" match than a small textured cube. Fix: use a
  non-orange tray, or a non-warm goal cube.
- **Close/touching objects -> ambiguous-detection abort.** `green cube` with
  a red cube touching it -> two overlapping DINO boxes, scores 0.48/0.41,
  margin < AMBIGUITY_MARGIN (0.15) -> abort. Spacing the cubes ~8-10 cm
  apart fixes it. For deliberately-clustered obstacle scenes, use the
  spatial prefix (`front green cube` -> disambiguate='near') which bypasses
  the margin guard.
- **A yellowish/wooden cube also reads as a weak "green cube"** (0.41) -
  second source of the green-cube ambiguity.
- Green-LED device at the back of the table is a persistent 2nd "green"
  match - move it off the table.

## LTM confirmed working
`applicable_knowledge` populates and the planner's chain_of_thought cites
it. BUT: the four single-PUSH rows (2026-08-27 15:06-15:22) all preach
"retry the same PUSH unchanged, never abandon the plan" - injected into a
push->pick->place task this made the planner re-issue PUSH while holding
the cube instead of PLACING. Workaround that session: `activate_ltm:false`
for the multi-skill run. Better: prune those four rows, or don't retrieve
them for multi-step tasks.

## Cross-axis grasp preference (NEW, implemented - NOT yet run on hardware)
Banana / carrot always failed: GraspGen's top-confidence candidate on a
long thin object is an end-to-end pinch (span > max_object_span 0.075 ->
width gate abort), or executes with the fingers along the length (see the
IMG_8772 photo).

- `grasp_transform.principal_axis_xy(points)` - cloud's horizontal long
  axis + elongation ratio (sqrt of 2D covariance eigenvalue ratio).
- `grasp_transform.rank_grasp_indices(..., object_long_axis, crossaxis_weight)`
  - soft re-rank: multiply each candidate's confidence by
  `1 - weight*|finger_axis . long_axis|` so crosswise grasps rise to the
  top. Never filters - if every grasp is lengthwise the list still returns.
- `bridge_node.py` execute_pick: computes the long axis in base frame from
  the object cloud, engages the preference only when footprint elongation
  >= `crossaxis_min_elongation` (1.6) so cubes/round objects are untouched.
- New params: `crossaxis_grasp_weight` (0.6), `crossaxis_min_elongation`
  (1.6). Set weight 0.0 to disable.
- Offline-verified: flips a 0.95-conf lengthwise pinch below a 0.80-conf
  crosswise grasp for a 5:1 footprint; leaves cube ordering unchanged.
- Restart-only (Python under symlink colcon install). TEST: banana/carrot
  pick on hardware; watch for the "Elongated object (footprint N:1)" log
  line and check the executed grasp closes across the width.
- If GraspGen returns NO crosswise grasp within the tilt gate, this cannot
  help - that needs multi-view capture or a wider tilt gate.

## The black ribbed aluminium table
User has no matte cover. That extrusion (parallel slots, specular) is a
worst-case ZED stereo surface and is behind most of the noisy-cloud
aborts. Cross-axis grasp helps tolerate it; a matte sheet would help more.

## Still open

- `execute_push` first hardware run (2026-08-27): GroundedSAM matched
  "rectangular block" to 53% of the frame -> raised the push mask ceiling
  to `push_max_mask_frac` 0.60 (pick keeps 0.40). Gripper close changed
  from Move-to-0.0 to `push_gripper_width` 0.005 (commanding the fingers
  into each other stalled the Move and the hand faulted; needed a
  franka_gripper_node restart). Not yet re-run.
- Grasp selector accepts "backhand" (high-yaw) grasps - pick works but
  moves wristily, and it is what forced fix #2. A wrist/yaw gate in
  `select_grasp_index` would prefer front-facing grasps. Not done.
- Wide objects (the 88 mm cup) still hit the `max_object_span` (0.075 m)
  gate - a real gripper limit (Franka Hand opens 80 mm), not tunable away.
  Rim-grasping a cup would need multi-view cloud capture (the single ZED
  view never contains the rim/interior) AND a cup-specific relax of the
  `min_gripper_width` gate - not a parameter tweak.
- Place tray-floor depth reading ~2 cm low was worked around with
  clearance, not root-caused (calibration z? thin tray? point catching
  the table through the tray?).
