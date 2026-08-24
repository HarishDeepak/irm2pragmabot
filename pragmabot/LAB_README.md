# Lab README — raw command notes

> **Start here instead for anything operational:**
> - **`RUNBOOK.md`** — the full pipeline, in order, with the check after each step.
>   Use this to bring the robot up. It supersedes the loose commands below.
> - **`HANDOFF_2026-08-24_EVENING.md`** — current project state, what changed,
>   what is broken, what is open.
> - `extras/HANDOFF_2026-08-24.md` — the earlier handoff (calibration facts,
>   protected-module rule).
> - `LAB_README_ARCHIVE.md` — the longer earlier version of this file (376 lines),
>   kept for the FoundationPose calibration results.
>
> This file stays as the raw scratch log of commands actually typed at the lab.
> Three things in it are now out of date — corrected here rather than edited
> away, since the history is useful:
>
> 1. **`export ROS_DOMAIN_ID=7` is now automatic.** It is in `~/.bashrc`, along
>    with sourcing `/opt/ros/humble` and `~/zed_ros2_ws/install`. Every new host
>    terminal is ready. The "we should add ..." note below is done.
> 2. **Use `~/zed_ros2_ws`, never `~/irm2pragmabot/zed_ros2_ws`.** The copy inside
>    the repo is source-only with no `install/` and is never built. `ros2 launch`
>    resolves packages by name, not by working directory, so the folder you stand
>    in has never mattered — only what is sourced. Same trap applies to the
>    vendored `ros2_ws/franka_ros2`, `GraspGen/` and `groundedsam/` in the repo,
>    each of which now carries a `NOT_THE_BUILT_COPY.md`.
> 3. **The published calibration TF was wrong by 15.7 mm in Z** (2026-08-24).
>    The measurement was fine; the composition to `zed_camera_link` was done from
>    `tf2_echo`'s 3-decimal output. Recomputed at full precision and written to
>    `~/.ros2/easy_handeye2/calibrations/fr3_zed_right.calib`
>    (backup `.bak-precision-2119`). It only takes effect after the robot stack is
>    relaunched, since `/handeye_publisher` reads the file at startup.
>    Verify with:
>    ```
>    ros2 run tf2_ros tf2_echo fr3_link0 zed_left_camera_frame_optical
>    ```
>    Translation should read `[0.912, -0.064, 0.514]`. If z is `0.498`, the old
>    value is still live. Full detail in `HANDOFF_2026-08-24_EVENING.md` §4.
>
> Also note `docker exec` needs `-e DISPLAY=$DISPLAY` or RViz will not open — the
> image carries a stale `DISPLAY=:2` while the real socket is `:1`. The
> container's `~/.bashrc` now sets it too.

---

container franka:
echo $ROS_DOMAIN_ID
echo $DISPLAY
docker exec -it -e DISPLAY=$DISPLAY franka_ros2_humble bash
source install/setup.bash
ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=10.10.10.10

in host:
echo $ROS_DOMAIN_ID
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2
ros2 launch zed_wrapper zed_camera.launch.py   camera_model:=zed2   depth_mode:=NEURAL   pos_tracking_enabled:=false

we should add:
    export ROS_DOMAIN_ID=7
for every host termianl

for camera calib
in franka container
ros2 launch franka_bringup example.launch.py controller_names:=gravity_compensation_example_controller

in zed ws

t1
ros2 run aruco_detector aruco_detector --ros-args --remap image:=/zed/zed_node/rgb/color/rect/image --remap camera_info:=/zed/zed_node/rgb/color/rect/image/camera_info -p marker_size:='0.05' -p image_is_rectified:=true

t2
ros2 launch easy_handeye2 calibrate.launch.py name:=fr3_zed_right calibration_type:='eye_on_base' tracking_base_frame:='zed_camera_link' tracking_marker_frame:='marker_0' robot_base_frame:='fr3_link0' robot_effector_frame:='fr3_hand'

after calib file is saved

to publish the tf

ros2 launch easy_handeye2 publish.launch.py name:=fr3_zed_right

and in another terminal

check and verify if it is being published

ros2 run tf2_ros tf2_echo fr3_link0 zed_camera_link

and then in rviz change the fixed frame to fr3_link0

harish@Alonnisos:~$ echo $DISPLAY
:1
harish@Alonnisos:~$ docker exec -it -e DISPLAY=$DISPLAY franka_ros2_humble bash

we use first container for franka:
user@Alonnisos:/ros2_ws$ ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=10.10.10.10

for camera first we go to the zed wrapper ros2:
harish@Alonnisos:~/zed_ros2_ws$ ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2

(no container is needed, but just remember to source install.... and

---

command for running bag file:
export ROS_DOMAIN_ID=7
cd ~/pragmabot/bags

ros2 bag record -o scene_single_cup 
  /tf /tf_static 
  /zed/zed_node/rgb/color/rect/image/camera_info 
  /zed/zed_node/rgb/color/rect/image 
  /zed/zed_node/depth/depth_registered

python3 extract_bag.py 
  --bag ~/pragmabot/bags/scene_single_cup 
  --out ~/pragmabot/extracted/scene_single_cup 
  --rgb-topic   /zed/zed_node/rgb/color/rect/image 
  --depth-topic /zed/zed_node/depth/depth_registered 
  --info-topic  /zed/zed_node/rgb/color/rect/image/camera_info

~/groundedsam/.venv/bin/python ~/pragmabot/calibration/detect_object.py 
    --rgb ~/pragmabot/extracted/scene_single_cup/rgb.png 
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

~/groundedsam/.venv/bin/python calibration/detect_object.py 
    --rgb extracted/scene_single_cup/rgb.png --prompt "red cube."

# 2. mask + depth -> 3D object point cloud (numpy only, any venv)

python3 calibration/mask_to_pointcloud.py 
    --depth extracted/scene_single_cup/depth.npy 
    --intrinsics extracted/scene_single_cup/intrinsics.json 
    --mask extracted/scene_single_cup/detections/mask.npy 
    --out extracted/scene_single_cup/detections/object_pcd.npy

# 3. GraspGen (its own separate venv)

source ~/GraspGen/.venv/bin/activate
python3 ~/GraspGen/client-server/graspgen_server.py 
    --gripper_config ~/GraspGen/GraspGenModels/checkpoints/graspgen_franka_panda.yml

python3 ~/GraspGen/client-server/graspgen_client.py     --pcd_file extracted/redcup/detections/object_pcd.npy     --topk_num_grasps 40     --visualize

python3 ~/GraspGen/client-server/graspgen_client.py 
    --pcd_file extracted/scene_single_cup/detections/object_pcd.npy

python3 ~/GraspGen/client-server/graspgen_client.py 
    --pcd_file extracted/scene_single_cup/detections/object_pcd.npy 
    --visualize

---

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
