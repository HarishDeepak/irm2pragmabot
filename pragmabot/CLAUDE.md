# IRM2 / pragmabot — Project Memory

## Architecture (host-only — the 3-container plan was abandoned)
Only Container 1 (control) stays containerized. Everything else runs
directly on Alonnisos, each GPU tool in its own isolated `uv`/conda venv —
isolation was always venv-level, never Docker-level, so dropping
Containers 2/3 lost nothing. The SHM/DDS root-vs-non-root bug that ate a
full day only existed because of a container boundary; host-only removes
the entire bug class, not just the symptom.

## Real confirmed paths (don't guess, don't recreate)
- **Control:** container `franka_ros2_humble`, source at
  `~/ros2_ws/franka_ros2` (docker-compose.yml here). Enter with:
  `docker exec -it -e DISPLAY=$DISPLAY franka_ros2_humble bash`
  Launch inside it:
  `ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=10.10.10.10`
  — if you get "package not found", run
  `source /ros2_ws/install/setup.bash` first (workspace overlay not
  sourced in a fresh shell); if `install/` is missing entirely, the
  container was recreated instead of restarted and needs
  `colcon build --symlink-install` again.
- **ZED:** host install at `~/zed_ros2_ws`, already tested working
  (RViz2-confirmed live feed). Fully independent of any container.
- **GraspGen:** `~/GraspGen` (has `GraspGenModels/` subfolder). Env is
  `uv venv --python 3.10 .venv` at `~/GraspGen/.venv`, `grasp_gen`
  installed editable. Activate: `source ~/GraspGen/.venv/bin/activate`.
  Confirmed working end-to-end against a **real ZED point cloud**
  (2026-07-24) — see "GraspGen real-data test" below. `GraspGenModels/`
  now only carries the `franka_panda` gripper checkpoints (Robotiq
  2F-140 and suction-cup checkpoints were removed as unneeded for this
  robot) and `sample_data/{meshes,real_object_pc}` (the 1.4GB upstream
  `real_scene_pc` demo dataset was also removed, unrelated to our ZED
  captures).
- **GroundedSAM (Grounded-SAM-2):** `~/groundedsam` — fully set up and
  tested working end to end. Env is `uv venv --python 3.10 .venv` at
  `~/groundedsam/.venv` (SEPARATE from GraspGen's venv — cannot share,
  torch>=2.3.1 vs GraspGen's torch==2.1.0). Repo cloned to
  `~/groundedsam/Grounded-SAM-2` (IDEA-Research/Grounded-SAM-2, not the
  original v1 Grounded-Segment-Anything that upstream PragmaBot's README
  links — this was a deliberate choice, confirmed with user 2026-07-24).
  Activate: `source ~/groundedsam/.venv/bin/activate`. Before any
  build/compile step: `export CUDA_HOME=/usr/local/cuda` and
  `export PATH=/usr/local/cuda/bin:$PATH` (nvcc isn't on PATH by
  default even though CUDA_HOME resolves) and
  `export TORCH_CUDA_ARCH_LIST="8.9"`. Installed via
  `uv pip install --no-build-isolation -e ".[notebooks]"` for SAM2 and
  `uv pip install --no-build-isolation -e grounding_dino` for
  GroundingDINO — **must** use `--no-build-isolation`, otherwise uv's
  build isolation pulls a fresh unrelated torch (seen: cu13.0) into a
  temp env and the CUDA-version-mismatch check fails against nvcc.
  `transformers` must be pinned `<5` (installed 4.57.6) — transformers
  5.x removed `BertModel.get_head_mask`, which GroundingDINO's
  `BertModelWarper` still calls; the demo throws
  `AttributeError: 'BertModel' object has no attribute 'get_head_mask'`
  on the unpinned version. Also needed at runtime, not auto-installed
  by either package's setup.py: `addict`, `yapf`, `timm`,
  `supervision`, `pycocotools`. Only the checkpoints
  `checkpoints/sam2.1_hiera_large.pt` and
  `gdino_checkpoints/groundingdino_swint_ogc.pth` were downloaded (the
  minimum pair `grounded_sam2_local_demo.py` needs), not the full set
  the repo's download scripts fetch by default — download the others
  on demand if a different demo script needs them. Verified working via
  `python grounded_sam2_local_demo.py` on the repo's own
  `notebooks/images/truck.jpg` — correct car+tire boxes and masks in
  `outputs/grounded_sam2_local_demo/`.
- **GroundedSAM wired into pragmabot:** `calibration/detect_object.py` —
  text-prompted 2D detection wrapper, run with
  `~/groundedsam/.venv/bin/python calibration/detect_object.py --rgb
  <path> --prompt "cube."`. Produces `mask.npy` (bool, same H×W as the
  input image), matching the `mask` contract
  `calibrate_extrinsic.py`'s `run_foundationpose_register()` expects.
  Internally must `sys.path.insert(0, GROUNDED_SAM2_ROOT)` before
  importing `groundingdino` — its `inference.py` does a self-import as
  `grounding_dino.groundingdino...` (repo-root-relative, not the
  installed package name `groundingdino`), which only resolves if the
  Grounded-SAM-2 repo root is on `sys.path`. Verified 2026-07-24 against
  `extracted/scene_single_cup/rgb.png` with prompt `"cube."` — correctly
  picked the real cube (conf 0.57) over a false-positive match on the
  e-stop button (conf 0.48); `--select best` (default) keeps only the
  top-confidence match in `mask.npy`, `--select all` unions everything
  above threshold.
- **FoundationPose:** `~/foundationpose/FoundationPose` — repo already
  cloned (2026-07-24), clean, on `main`. `.venv` at `~/foundationpose/.venv`
  already created with **python 3.11** (confirmed 2026-07-25, matches
  live `environment.yml`) but has no packages installed yet — env, not
  weights, is the next step. A `.venv_py39_unused` sits alongside it from
  an earlier, wrong python-3.9 attempt — do not use it, safe to delete.
  No `weights/` folder yet — model weights not downloaded. Do NOT reuse
  the old `foundationpose` Docker container (`Exited (255)`) — abandoned
  along with the rest of the container plan.
- **pragmabot bridge/planner:** `~/pragmabot` (this repo).
- **VS Code workspace:** `~/irm2.code-workspace` — multi-root, one
  terminal profile per environment above, opened via
  `code ~/irm2.code-workspace`.

## Shared machine — disk space
Alonnisos is a shared multi-user box; `/` (`/dev/nvme0n1p2`, 938G) has
hit 100% full at least once (2026-07-24, during Grounded-SAM-2 setup) —
other users' home dirs (e.g. `/home/rickmer` at 152G) account for most
usage, not our own tools. If a pip/uv install fails with "No space left
on device," check `df -h /` first; clearing our own `uv cache clean`
only frees a few GB and may not be enough. This is not ours to fix by
deleting other users' files — flag it and wait rather than guess at
cleanup.

## Robot identity (corrected — do not revisit)
Robot is a **Franka FR3** ("Athna", arm ID `10070378`, firmware 5.9.0, IP
`10.10.10.10`, Franka Hand gripper) — not a Panda. Never reference
`role_ros2`, Polymetis, or `rickmer-ros2_*` containers — that track is
abandoned; it solved a problem that doesn't apply to this robot.

## Live control interfaces (confirmed working on real hardware)
`/move_action`, `/execute_trajectory`, `/compute_cartesian_path`,
`/compute_ik`, `/franka_gripper/grasp`. Never
`/fr3_gripper/gripper_action` (dead stub, silently hangs).

## Hardware/version facts
- Alonnisos: driver 535.183.01, RTX 4080 (Ada, compute capability `8.9`).
- GraspGen: `torch==2.1.0`, `torchvision==0.16.0`, cu121, python 3.10.
- Grounded-SAM-2: `torch>=2.3.1`, `torchvision>=0.18.1`, cu121.
- FoundationPose: python 3.9, builds PyTorch3D + NVDiffRast from source.
- **Always** `export TORCH_CUDA_ARCH_LIST="8.9"` before compiling any
  custom CUDA op in this project (pointnet2_ops, ms_deform_attn,
  PyTorch3D, NVDiffRast).
- Real measured joint config (replaces old placeholder):
  `[-0.181, -0.420, 0.005, -2.260, -0.008, 1.806, 0.609]`
- DISPLAY-mismatch bug already baked into the VS Code terminal profile
  for the Control tab (`-e DISPLAY=$DISPLAY` on every `docker exec`) —
  don't need to remember this fix manually anymore.

## FoundationPose calibration — verified facts (don't re-research)
- **Cube symmetry blocker:** a plain-colored cube has 24 rotational
  symmetries — FoundationPose returns *a* valid pose, not an error, but
  possibly the wrong symmetric equivalent. One face must be uniquely
  marked (sticker/pattern/ArUco) before pose estimation is usable.
- Mesh: `.obj`+`.mtl`+texture PNG, units must be **meters**, no
  auto-scaling. Generate via `trimesh.creation.box(extents=[s,s,s])`.
- ZED2 optical frame is `<camera_name>_left_camera_frame_optical` — the
  older `_left_camera_optical_frame` naming is deprecated, don't use it.
- IMU accelerometer-at-rest gives **orientation only** (roll/pitch via
  gravity direction) — never translation, never yaw.
- Chosen calibration method is direct composition, not AX=XB hand-eye:
  `T_cam_in_base = T_cube_in_base @ inv(T_cube_in_camera)`.
- Calibration tooling lives in `pragmabot/calibration/`
  (`mesh_gen.py`, `calibrate_extrinsic.py`). The FoundationPose
  `register()` call inside `calibrate_extrinsic.py` is a flagged
  placeholder — confirm the real API against the actual repo before
  filling it in.

## GraspGen real-data test (2026-07-24, don't re-derive)
- **ZED topics only appear under `ROS_DOMAIN_ID=7`** — not the default
  domain. Confirm with `ps aux | grep zed` then read
  `/proc/<pid>/environ` if `ros2 topic list` comes back empty; don't
  assume the camera isn't running just because the default domain sees
  nothing.
- Raw point cloud topic: `/zed/zed_node/point_cloud/cloud_registered`
  (`sensor_msgs/PointCloud2`, XYZRGB, ~9Hz, frame_id
  `zed_left_camera_frame` — the non-optical camera-link frame: x-forward,
  y-left, z-up).
- Capture script: `~/GraspGen/scripts/capture_zed_frame.py` — one-shot
  rclpy subscriber, saves `xyz` as `.npy` (format
  `graspgen_client.py --pcd_file` already accepts natively).
- **The model expects an object-scale point cloud (~2000 pts, roughly
  object-sized extent), not a raw scene capture.** A full unfiltered ZED
  frame (~100k pts spanning meters) blows CUDA memory (tried to alloc
  40GiB); a depth-cropped but still room-scale cloud (~4000 pts spread
  over ~1m+) crashes the discriminator's outlier-removal step (`knn`
  distance matrix on a non-compact cluster removes every point → reshape
  error). Fix: crop to a tight bounding box around the dense
  near-camera cluster (~30cm cube) *before* downsampling to ~2000 pts —
  same scale as what `demo_object_mesh.py` samples from a mesh surface.
  (This was the state before GroundedSAM was wired in — see below for
  the real segmented-object pipeline that superseded the hand-picked
  crop.)
- Server: `graspgen_franka_panda.yml` gripper config (Robotiq's
  generator checkpoint doesn't exist / was removed — franka_panda is
  also the correct match for this robot's actual gripper anyway).
  Start: `python3 client-server/graspgen_server.py --gripper_config
  GraspGenModels/checkpoints/graspgen_franka_panda.yml` (from
  `~/GraspGen`, its own venv, ~5s to load). Then
  `python3 client-server/graspgen_client.py --pcd_file <xyz.npy>`.
- Result on one real captured+cropped frame: 50 grasps returned,
  confidence 0.84–0.92, rotation matrices properly orthonormal (det ≈
  1.0, no NaN/Inf), grasp positions landed inside the cropped input
  cloud's own bounding box in camera frame (x 0.29–0.61m forward, y
  ±0.24m, z -0.22–0.00m) — physically plausible, not a fluke/garbage
  output.
- **Real segmented-object pipeline (2026-07-24, supersedes the
  hand-picked crop above):** `calibration/mask_to_pointcloud.py` closes
  the gap — takes `detect_object.py`'s `mask.npy` + `depth.npy` +
  `intrinsics.json`, back-projects the masked pixels to camera-frame XYZ
  (standard pinhole: `x=(u-cx)*z/fx, y=(v-cy)*z/fy, z=depth`), filters
  invalid/out-of-range depth, downsamples to `--target-points` (default
  2000, matching the scale proven safe above). Output feeds directly
  into `graspgen_client.py --pcd_file` unchanged — the client already
  centers point clouds itself. Only needs numpy, so it runs in either
  venv (or system python3) — not tied to GraspGen's or GroundedSAM's
  venv. Verified end-to-end on `extracted/scene_single_cup/`: GroundedSAM
  mask (prompt `"cube."`) → 1947-point object cloud (x[-0.154,-0.094]
  y[-0.112,-0.028] z[0.610,0.777] m, correctly object-scale, no
  hand-picking) → GraspGen server → 50 grasps, confidence 0.77–0.95,
  valid orthonormal rotations. Full chain:
  ```
  ~/groundedsam/.venv/bin/python calibration/detect_object.py --rgb <rgb.png> --prompt "cube."
  python3 calibration/mask_to_pointcloud.py --depth <depth.npy> --intrinsics <intrinsics.json> \
      --mask <detections/mask.npy> --out <detections/object_pcd.npy>
  # separate venv:
  source ~/GraspGen/.venv/bin/activate
  python3 ~/GraspGen/client-server/graspgen_server.py --gripper_config ~/GraspGen/GraspGenModels/checkpoints/graspgen_franka_panda.yml &
  python3 ~/GraspGen/client-server/graspgen_client.py --pcd_file <detections/object_pcd.npy>
  ```
- **Stereo "flying pixel" edge noise (2026-07-24, fixed in
  `mask_to_pointcloud.py`):** the first version of the pipeline above
  produced a point cloud that didn't look like the cube it came from — a
  16.7cm z-spread for an object whose real depth extent is ~4-5cm. Root
  cause confirmed by comparing eroded-interior vs boundary-ring pixels of
  the same mask: interior pixels had z std=1.7cm (a real compact cube),
  but boundary pixels were 4x more likely to be a >4cm-far outlier
  (41% vs 10%) — classic stereo depth-camera artifact where a silhouette
  edge pixel gets a depth interpolated between the near object and far
  background, not a GroundedSAM/SAM2 mask problem (the 2D mask itself was
  fine). Fix, both on by default in `mask_to_pointcloud.py`:
  `--erode-px 3` (shrinks the mask inward before back-projecting, pure
  numpy cross-kernel erosion — no cv2/scipy, since GraspGen's venv has
  neither) and `--z-outlier-mad 3.5` (drops points beyond N scaled-MAD
  from the median depth; MAD chosen over mean/std because this artifact
  is one-sided and drags a mean/std filter along with it). Result on the
  same cube scene: 1947 → 1449 points, z-extent 16.7cm → 4.5cm, and
  GraspGen confidence jumped from 0.77–0.95 to 0.94–0.98 — the cleaner
  input measurably improved the model's own confidence, not just the
  visual shape.

## Hard rule inherited from upstream PragmaBot
Never modify: `vlm_client.py`, `vlm_task_planner.py`, `vlm_scene_describer.py`,
`vlm_success_detector.py`, `vlm_exp_summarizer.py`, `memory_manager.py`,
`scene_observer.py`, `conversation_builder.py`. Only permitted edit
pattern: import + instantiate the executor, replace `NotImplementedError`
sites in `pragmabot_node.py`.

## Behavioral rule
Flag uncertainty rather than guess, especially about compatibility or
versions. This project has already had two real incidents from assuming
instead of checking: a fabricated Docker image tag, and Panda/FR3
robot-identity confusion that triggered an unnecessary architecture
pivot. Ask before making an architecture decision not already settled
above.
