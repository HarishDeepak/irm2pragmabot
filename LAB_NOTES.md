container franka:
echo $ROS_DOMAIN_ID
echo $DISPLAY
docker exec -it -e DISPLAY=$DISPLAY franka_ros2_humble bash
ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=10.10.10.10

in host:
echo $ROS_DOMAIN_ID
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2

we should add:
    export ROS_DOMAIN_ID=7
for every host termianl










harish@Alonnisos:~$ echo $DISPLAY
:1
harish@Alonnisos:~$ docker exec -it -e DISPLAY=$DISPLAY franka_ros2_humble bash

we use first container for franka:
user@Alonnisos:/ros2_ws$ ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=10.10.10.10


for camera first we go to the zed wrapper ros2:
harish@Alonnisos:~/zed_ros2_ws$ ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2

(no container is needed, but just remember to source install.... and 


--------------------------------------------------------
command for running bag file:


ros2 bag record -o scene_single_cup \
  /tf /tf_static \
  /zed/zed_node/rgb/color/rect/image/camera_info \
  /zed/zed_node/rgb/color/rect/image \
  /zed/zed_node/depth/depth_registered


python3 extract_bag.py \
  --bag ~/pragmabot/bags/scene_single_cup \
  --out ~/pragmabot/extracted/scene_single_cup \
  --rgb-topic   /zed/zed_node/rgb/color/rect/image \
  --depth-topic /zed/zed_node/depth/depth_registered \
  --info-topic  /zed/zed_node/rgb/color/rect/image/camera_info
  
~/groundedsam/.venv/bin/python ~/pragmabot/calibration/detect_object.py \
    --rgb ~/pragmabot/extracted/scene_single_cup/rgb.png \
    --prompt "cube."



etc info

- --prompt follows GroundingDINO convention: lowercase, each phrase ending in a period. You can search for multiple things at once, e.g. "cube. red button.".
- Output goes to <rgb's folder>/detections/ by default (override with --out-dir):
  - mask.npy — boolean array, same H×W as the input image, ready to pass straight into calibrate_extrinsic.py's mask argument
  - annotated.jpg — boxes + mask overlay, for a visual sanity check
  - detections.json — every match above threshold (class, bbox, confidence, RLE mask)
- --select best (default) keeps only the highest-confidence match in mask.npy; --select all unions every match above threshold into one mask.
- --box-threshold / --text-threshold (defaults 0.35/0.25) tune detection sensitivity if a prompt is missing or over-triggering.









# 1. detect + segment (Grounded-SAM-2's venv)
~/groundedsam/.venv/bin/python calibration/detect_object.py \
    --rgb extracted/scene_single_cup/rgb.png --prompt "cube."

# 2. mask + depth -> 3D object point cloud (numpy only, any venv)
python3 calibration/mask_to_pointcloud.py \
    --depth extracted/scene_single_cup/depth.npy \
    --intrinsics extracted/scene_single_cup/intrinsics.json \
    --mask extracted/scene_single_cup/detections/mask.npy \
    --out extracted/scene_single_cup/detections/object_pcd.npy

# 3. GraspGen (its own separate venv)
source ~/GraspGen/.venv/bin/activate
python3 ~/GraspGen/client-server/graspgen_server.py \
    --gripper_config ~/GraspGen/GraspGenModels/checkpoints/graspgen_franka_panda.yml &
python3 ~/GraspGen/client-server/graspgen_client.py \
    --pcd_file extracted/scene_single_cup/detections/object_pcd.npy
    
python3 ~/GraspGen/client-server/graspgen_client.py \
    --pcd_file extracted/scene_single_cup/detections/object_pcd.npy \
    --visualize
    
    
------------------------------------------------------------------------  -
  












Let me look at the actual depth values inside the mask to see what's going on — a 16.7cm z-spread for what should be a small cube is suspicious.

  Ran 2 shell commands

Confirmed the cause. Here's what's going on:

The point cloud is smeared because of stereo-depth "flying pixel" noise at the mask's silhouette edge — not because of a bad mask.

- The cube's actual body sits in a tight cluster around z≈0.61–0.64m (std 1.7cm on interior pixels — that's a real, compact cube).
- But ~16% of the SAM2 mask pixels near the boundary of the cube's silhouette get bogus depth values that trail off up to 17cm further away (z up to 0.777m). Edge pixels have 4× the "far outlier" rate of interior pixels (41% vs 10%).
- This is a well-known artifact of stereo depth cameras (ZED included): at any depth discontinuity — an object edge against the background — the stereo matcher can't cleanly resolve which surface a boundary pixel belongs to, so it interpolates between the near and far depth, producing a comet-tail of noisy points trailing off the true object edge. The 2D mask itself is accurate (that's a GroundedSAM/SAM2 problem, not present here); it's the depth values at those same pixel locations

Fix: erode the mask by a few pixels before back-projecting (drops exactly the boundary ring where this noise lives), and/or add an outlier filter (e.g. keep only points within N·std of the median z). Want me to add a --erode-px and/or a z-outlier filter to mask_to_pointcloud.py and regenerate the point cloud/grasps?




❯ do we have space to install it for ourself

  Listed 1 directory, ran 2 shell commands

Yes, barely — but there's a much better option I just found. Let me lay out the numbers.

Host venv/conda route: We have 30GB free (97% full, 938GB disk). A native FoundationPose env (PyTorch3D + nvdiffrast built from source, like jiayun/rickmer have) runs 5–11GB based on comparable CUDA/torch envs already on this machine (rickmer's envs range 4.5–11GB, jiayun's 3.6–6.4GB). It would fit, but on a disk that already hit 100% once this session, that's cutting it close and leaves almost no margin.

Docker route — much better: docker images shows this machine already has three pre-pulled FoundationPose images, including the official wenbowen123/foundationpose:latest (20.2GB) — sitting there at zero additional disk cost since they're already downloaded. Docker images are shared across all users on the host, not per-home-directory.

This directly conflicts with the CLAUDE.md note "Do NOT reuse the old foundationpose Docker container — abandoned along with the rest of the container plan." That note was about not resurrecting a specific dead, exited container (Exited (255)) — but the underlying image is still here and pullable into a fresh container, which is a different thing. Given the disk situation, I think this is worth revisiting rather than blindly following the old decision.

● User declined to answer questions
· Given disk is at 97% (30GB free) and this machine already has the official wenbowen123/foundationpose:latest image pulled (20.2GB, zero marginal cost to use), how do you want to set up FoundationPose? (Use the existing Docker image (Recommended) / Install natively on host anyway / Just check the numbers, decide later)

What would you like to clarify?

✻ Churned for 49s


==========================================================================
SESSION LOG — 2026-07-25 — FoundationPose calibration pipeline, end to end
==========================================================================

## Disk space
- Started at 25G free / 98% full. `uv cache clean` alone freed 5.2GiB
  (own cache only, no risk to other users' files — never touch anything
  outside our own home dir on this shared box).
- Ended around 21-22G free after weights/demo data downloads. Still tight
  — check `df -h /` before any further large installs.

## FoundationPose: native build attempted, then abandoned for Docker
- CLAUDE.md said python 3.9 — WRONG, live upstream environment.yml pins
  python 3.11. Fixed the note.
- Repo was already cloned at ~/foundationpose/FoundationPose (someone had
  done this before, half set up, with a stray `.venv_py39_unused` from an
  earlier wrong attempt — deleted).
- Native build (torch/pytorch3d/nvdiffrast/mycpp from source) was started
  in background, ~30-90min expected, but killed partway through in favor
  of Docker once we found 3 pre-pulled FoundationPose images already on
  this shared machine (zero extra disk cost to reuse).
- Of those 3 images, only one actually works on this GPU:
  - `wenbowen123/foundationpose:latest` — CUDA 11.3 nvcc, FAILS to
    compile nvdiffrast for the RTX 4080 (`compute_89`/Ada needs 11.8+)
  - local `foundationpose:latest` — also CUDA 11.3, same failure
  - **`200851842357.dkr.ecr.us-east-1.amazonaws.com/snapdock/community/
    vision/foundationpose:latest`** — CUDA 11.8, WORKS. This is the one
    to use.

## The running setup (persists across reboots of the terminal, not the machine)
- Named, persistent (non `--rm`) container: **`foundationpose_calib`**
  - Built from the CUDA 11.8 ECR image above
  - `--gpus all`, repo mounted: host `~/foundationpose/FoundationPose` ->
    container `/workspace/FoundationPose`
  - Conda env inside: `/opt/conda/envs/my` (torch 2.0.0+cu118, pytorch3d
    0.7.8, nvdiffrast — all prebuilt in the image)
  - `mycpp` (FoundationPose's own C++/pybind11 extension) was built once
    inside this container against that env — NOT reusable from the other
    two images, different CUDA/torch.
  - Restart if the container ever stops: `docker start foundationpose_calib`
    then `docker exec -it foundationpose_calib bash` to get a shell in it.
- Weights downloaded via `uvx gdown --folder ...` into
  `~/foundationpose/FoundationPose/weights/{2023-10-28-18-33-37,
  2024-01-11-20-02-45}/` (had to move them up one level out of a
  `no_diffusion/` subfolder gdown created).
- Demo data (mustard bottle) downloaded similarly into `demo_data/` —
  only `mustard0/` kept, the unused `kinect_driller_seq.zip` (943MB) was
  deleted immediately to save space.
- `run_demo.py --debug 0` runs clean end-to-end on the demo data
  (`--debug 1`'s default GUI crashes headless — no X display in
  `docker exec`, unrelated to the CUDA fix).

## Calibration pipeline — new/changed files in this repo
- **`calibration/capture_calib_frame.py`** (new) — live ZED capture on
  host, synced RGB+depth+intrinsics via `message_filters.
  ApproximateTimeSynchronizer`. Real topic names on this ZED driver
  (different from what CLAUDE.md/extract_bag.py assumed):
  - `/zed/zed_node/rgb/color/rect/image`
  - `/zed/zed_node/depth/depth_registered`
  - `/zed/zed_node/rgb/color/rect/camera_info`
  Run: `ROS_DOMAIN_ID=7 python3 calibration/capture_calib_frame.py --out extracted/<name>`
- **`calibration/mesh_gen.py`** (rewritten) — the old `--face-texture`
  approach was confirmed BROKEN (trimesh's default box UVs paint the
  same image on all 6 faces, not just one — defeats the whole point of
  breaking cube symmetry). Replaced with `--edge-stripe`: builds the
  cube as 6 independent faces (no shared vertices) with real per-face
  solid colors, one face getting a stripe along one edge (not centered —
  a centered/symmetric mark doesn't break the remaining 4-fold in-plane
  rotation ambiguity). Matches a real strip of tape stuck along one edge
  of the physical cube.
  Run: `python3 calibration/mesh_gen.py --side 0.061 --out cube.obj --edge-stripe --stripe-face +z --stripe-frac 0.3`
  (needs trimesh — not on host, ran via the container's python instead)
- **`calibration/calibrate_extrinsic.py`** (finished — was a stub before
  today) — full pipeline: captures N synced frames live (cube stays
  stationary), runs GroundedSAM (`detect_object.py`, its own venv, shelled
  out via subprocess) for each frame's mask, calls FoundationPose's
  `register()` inside `foundationpose_calib` via `docker exec` + file
  exchange (no ROS2/DDS ever crosses into the container — deliberate,
  same bug class as the SHM/DDS incident that killed the 3-container
  plan), averages poses with 2-sigma outlier rejection, composes with
  hand-measured `T_cube_in_base`, publishes TF, runs an IMU roll/pitch
  sanity check.
  Container-side counterpart: `~/foundationpose/FoundationPose/calib_io/
  register_once.py` (new) — loads mesh+rgb+depth+K+mask from files,
  calls `FoundationPose.register()`, writes the 4x4 pose back to a file.
  Real confirmed API (was previously an unverified guess in this file):
  ```
  est = FoundationPose(model_pts=mesh.vertices, model_normals=mesh.vertex_normals,
                        mesh=mesh, scorer=scorer, refiner=refiner,
                        debug_dir=..., debug=0, glctx=glctx)
  pose = est.register(K=K, rgb=rgb, depth=depth, ob_mask=mask, iteration=est_refine_iter)
  ```

## Host environment fix
- System `scipy` (apt, 1.8.0) was ABI-broken against a user-level pip
  `numpy` (2.2.6, `~/.local/lib/...`, shadows the older apt numpy).
  Fixed with `pip3 install --user --upgrade "scipy>=1.11"` — user-level
  only, doesn't touch system packages, safe/reversible.

## Calibration redo (2026-07-27)
- Root-cause checklist worked through before re-running: mesh units
  confirmed meters (`[0.061,0.061,0.061]` via trimesh in the
  `foundationpose_calib` container), registered/aligned depth topic
  confirmed (`/zed/zed_node/depth/depth_registered`), real ZED2
  intrinsics pulled live from `/zed/zed_node/rgb/color/rect/camera_info`
  at HD1080 (fx=fy=1048.424, cx=946.911, cy=557.599) — `fx=fy=700` was
  never actually in `config.yaml`, it's a separate placeholder in
  `panda_pick_server.py`/`panda_push_server.py`/`panda_place_server.py`,
  intentionally deferred to "Phase 7", not touched this session.
- ZED2 launched with `camera_model:=zed2
  param_overrides:="general.grab_resolution:=HD1080"` — `depth_mode` and
  `pos_tracking_enabled` are NOT top-level launch args in this wrapper
  version (would be rejected by `ros2 launch`); they're
  `param_overrides` dotted keys (`depth.depth_mode`,
  `pos_tracking.pos_tracking_enabled`) and both already default to the
  right values on `local-fr3-setup` (`NEURAL_PLUS`, `false`) in
  `zed-ros2-wrapper/zed_wrapper/config/common_stereo.yaml`.
- Topic namespace confirmed live: `/zed/zed_node/...` (not `/zedxm`,
  which only exists in this repo's `config.yaml` from an unrelated/older
  setup).
- **Two real bugs found and fixed in this repo's calibration code:**
  1. `calibrate_extrinsic.py`'s `run_foundationpose_register()` passed
     the HOST mesh path straight to `docker exec`, which can't see host
     paths — container only sees its mounted `calib_io/` at
     `/workspace/FoundationPose/calib_io`. Fixed: resolve `--mesh`
     relative to `FP_CALIB_IO_HOST` and translate to the container path
     before calling `register_once.py`.
  2. TF one-shot publish bug (previously known, not yet fixed): the
     script now `rclpy.spin()`s the `ExtrinsicPublisher` node forever
     instead of one `spin_once` + immediate shutdown, so `/tf_static`
     stays available to subscribers that connect after the script
     finishes running (RViz, `tf2_echo`, ...). Ctrl-C to stop.
  3. `register_once.py` (in `~/foundationpose/FoundationPose/calib_io/`,
     not this repo) OOM'd a 16GB GPU at 1920x1080 input —
     `scorer.predict()`'s crop-data warp costs memory proportional to
     the FULL frame resolution x 252 rotation hypotheses, not just the
     object crop. Fixed with a `--max-side 960` downscale (rescales K to
     match) before `register()`; uses NEAREST interpolation for
     depth/mask specifically to avoid reintroducing the "flying pixel"
     edge-blend artifact already fixed once in `mask_to_pointcloud.py`.
- `calibrate_extrinsic.py`'s `--cube-xyz-in-base` is now optional — if
  omitted, it runs the live capture + FoundationPose register/average
  loop and saves `T_cube_in_camera` to `--out-cube-pose` without
  publishing any TF, so the (expensive, GPU-bound) vision step doesn't
  have to be re-run just because the hand measurement isn't ready yet.
- 8/8 frames registered clean, zero outliers dropped (2-sigma rotation
  filter). Translation agreed to the mm across all 8: x≈0.0633,
  y≈0.0958, z≈0.4098m in the camera frame.
- Cube physically NOT square with the base this time (previous run's
  docstring recommendation to axis-align it wasn't followed) — decided
  to measure the actual yaw (FR3 TCP touch or protractor against a
  base-aligned reference) rather than assume `rpy=[0,0,0]`, per the
  same "don't eyeball it" lesson from the 2026-07-25 run below.
- **New TF bug found while publishing the result:** the script's default
  `--child-frame zed_left_camera_frame_optical` already has a parent
  inside the ZED wrapper's own internal static TF tree
  (`zed_camera_link -> ... -> zed_left_camera_frame_optical`) —
  publishing `fr3_link0 -> zed_left_camera_frame_optical` directly gives
  that frame two parents, an undefined/conflicting TF tree. Fix: compose
  out the wrapper's own fixed `zed_camera_link -> ...optical` offset
  (looked up via `tf2_echo`, static) and attach the external calibration
  to `zed_camera_link` instead — the actual root of the ZED's own tree.
  `calibrate_extrinsic.py` still needs this composition step added
  properly (currently done as a one-off outside the script); tracked as
  a follow-up, not yet ported into the script itself.
- Tape/mesh-scale note: `mesh_gen.py`'s existing `cube.obj` was built
  with `--stripe-frac 0.3` (30% of the top face's depth = ~1.8cm on the
  6.1cm cube) — re-taping to match this exactly (not eyeballing a
  slightly different %) matters because FoundationPose scores poses by
  rendering the mesh against the real RGBD frame; a stripe-size mismatch
  between the physical tape and the modeled mesh is a real, avoidable
  source of pose bias.

## Real calibration run (2026-07-25) — RESULT NOT YET FULLY TRUSTED
- Cube: 3D-printed, green, 6.1cm side (measured, not the old 6cm
  placeholder). Marked with off-white tape, one edge stripe on the top
  face, ~30% of that face's depth, NOT centered (centered/full-face
  marks don't break rotational symmetry — confirmed this mistake once
  during marking and corrected it).
- 8 frames captured, cube stationary. Translation extremely consistent
  across all 8 (agreeing to the mm: x~0.0597, y~0.0082, z~0.368m in
  camera frame) — vision pipeline itself is stable.
- `--cube-xyz-in-base 0.60 0.0 0.0305` used — **0.60m is an approximate
  single measurement along the base's forward axis, y assumed 0, z is
  half the cube's height ASSUMING fr3_link0's origin height equals the
  table surface height. Not independently verified. Revisit with a real
  3-axis caliper/tape measurement from the actual fr3_link0 origin
  (center of the round base mounting plate) before trusting this for
  anything beyond a rough visual sanity check.**
- Result: yaw ~-176.6° (~180°) matches known physical setup (camera and
  robot face each other) — good sign. IMU roll/pitch sanity check showed
  a large mismatch, but this check's own math likely isn't valid in a
  facing-each-other configuration (it implicitly assumes the camera's
  identity orientation roughly matches the base's forward axis) — so
  that mismatch is NOT strong evidence of an error, just an inconclusive
  check for this geometry. Real arbiter is the RViz2 point-cloud-vs-
  robot-mesh visual alignment check — still in progress at end of session.
- **Known bug found and fix given, not yet re-verified:** `calibrate_
  extrinsic.py`'s TF publish is one-shot (`StaticTransformBroadcaster`
  then the script exits) — `/tf_static`'s "latched" delivery only
  reaches subscribers that were already listening *before* the
  publisher process exits. Any fresh subscriber (new `tf2_echo`, a
  restarted RViz2) afterward sees "two unconnected trees." Fix used:
  run it as a persistent `ros2 run tf2_ros static_transform_publisher`
  process instead (left running in its own terminal), using the same
  computed x/y/z/qx/qy/qz/qw. `calibrate_extrinsic.py` itself should
  probably be changed to either spin forever or hand off to a persistent
  publisher automatically — not done yet.

## Still open / next steps
1. Get a real 3-axis measurement of the cube's position relative to
   `fr3_link0`'s actual origin (not the ~50cm/60cm rough estimate used
   today) — this is the biggest accuracy gap right now.
2. Finish the RViz2 visual alignment check (point cloud vs robot mesh,
   viewed from the side, focused on the table) with the persistent
   static_transform_publisher running.
3. Decide whether to fix `calibrate_extrinsic.py` to auto-publish
   persistently (e.g. spin forever, or launch a static_transform_publisher
   subprocess itself) instead of the current one-shot behavior.
4. Optional refinement discussed but not implemented: override the
   computed rotation's roll/pitch with the ZED's own IMU reading (IMU
   can't give yaw or translation, only roll/pitch from gravity) — only
   worth doing once IMU noise vs. vision noise is actually compared.
