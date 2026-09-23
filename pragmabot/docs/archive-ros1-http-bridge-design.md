# Archived design — ROS1 + HTTP-bridge execution layer (superseded)

**Status: SUPERSEDED, not current.** This document is a preserved transcript
of an early design/implementation session (around 2026-08). It designs and
partially implements a `pragmabot_node.py` running under **ROS 1 (rospy)**,
talking to MoveIt in a **separate ROS 2 Docker container** over a **custom
HTTP bridge** (`moveit_client.py` / `moveit_server.py`), against a **Franka
Panda**.

None of that matches the current system. The actual, current architecture
(confirmed in `../CLAUDE.md` and `../ARMIN.md`, and in the real
`ros2_ws/src/pragmabot_bridge/` code) is:

- **Host-only, no Docker isolation for the tooling** — only the Control
  side (MoveIt/franka_ros2) runs in a container; GraspGen and
  Grounded-SAM-2 run directly on the host in their own `uv` venvs.
- **ROS 2 Humble throughout** — no ROS 1, no `rospy`, no HTTP bridge.
  `pragmabot_bridge` (`bridge_node.py`) is a native `rclpy` node that talks
  to `/move_action`, `/compute_cartesian_path`, `/execute_trajectory`, and
  the Franka gripper actions directly.
- **Franka FR3** ("Athna"), not a Panda.

Kept here as a historical record of the design that was tried and moved
away from, not as a reference for how the system currently works — read
`../README.md` and `../CLAUDE.md` for that.

---

** so i want to explain this project. we have to reimplement this pragmabot paper:
https://arxiv.org/pdf/2507.16713

in this project until now we use zed camera for camera (camera outside of the robot and fixed in a place), graspgen for grasping, sam ground for detection object, and moveit for controlling ( we use docker container ros2_humble) and inverse kinematik. i also work on follwing pipeline and implement each of the seperately with small example tests that they work. now i want to implement all of them as pipeline like this:
PERCEPTION                    DECISION                      EXECUTION
───────────                   ────────                      ─────────
Grounded SAM  ──┐
  (segment)     │
                ├──► GraspGen ──► 6D grasp pose (camera frame)
ZED depth ──────┘        │
                          │
                     [YOU ARE HERE — everything above this line works]
                          │
                    ══════╪══════  <- calibration wall
                          │
                    camera→robot transform
                          │
                          ▼
                    grasp pose (ROBOT frame)
                          │
                          ▼
                    motion planning (MoveIt)
                          │
                          ▼
                    robot moves, grasps, lifts
                          │
                          ▼
                  [repeat similar chain for FPS → placing]

also in this file code for example some impleementation are empty and i have to implement by my self but how:D:\Armin\pragmabot\pragmabot\pragmabot\pragmabot\nodes\pragmabot_node.py
grasping link: https://github.com/NVlabs/GraspGen
sam ground link: https://github.com/IDEA-Research/Grounded-Segment-Anything



part of recommandations:
Now let me implement this. I'll add the pieces in order: config, the transform loader, the pipeline orchestration, a MoveIt client, and wire it into pragmabot_node.py.

1. Config — add the new section:

● Update(config/config.yaml)
Added 12 lines, removed 1 line
topics:
  color_image: /zedxm/zed_node/rgb/image_rect_color/compressed
  depth_image: /zedxm/zed_node/depth/depth_registered
  camera_info: /zedxm/zed_node/rgb/camera_info
 No newline at end of file
  camera_info: /zedxm/zed_node/rgb/camera_info

# --- Grasp Execution Pipeline (segmentation -> point cloud -> grasp -> robot frame -> MoveIt) ---
execution:
  scratch_dir: /tmp/pragmabot_grasp
  # camera->robot transform saved by easy_handeye2 (eye_on_base calibration, name: fr3_zed_right)
  camera_extrinsics_file: ~/.ros2/easy_handeye2/calibrations/fr3_zed_right.calib.yaml
  groundedsam_python: ~/groundedsam/.venv/bin/python
  graspgen_python: ~/GraspGen/.venv/bin/python
  graspgen_client_script: ~/GraspGen/client-server/graspgen_client.py
  # small HTTP server running inside the ros2_humble docker container, wraps MoveIt
  moveit_server_url: http://localhost:9000
 No newline at end of file

2. The core pipeline module — orchestrates the three subprocess calls and does the frame transform:

● Write(src/pragmabot/grasp_pipeline.py)
Wrote 182 lines to src/pragmabot/grasp_pipeline.py
"""Segmentation -> point cloud -> grasp pose -> robot frame.

Crosses the calibration wall: takes a text-prompted target object plus the
current camera observation, and returns a single best grasp pose in the
ROBOT frame, ready to hand to a MoveIt execution client.

Follows the same cross-venv pattern already used by calibration/detect_object.py
and calibration/calibrate_extrinsic.py: tools that live in their own venv
(Grounded-SAM-2, GraspGen) are invoked as subprocesses, with files as the
interface, instead of imported in-process.
… +172 lines

3. A tiny MoveIt client — plain HTTP, no ROS1↔ROS2 bridge needed:

● Write(src/pragmabot/moveit_client.py)
Wrote 53 lines to src/pragmabot/moveit_client.py
"""HTTP client for the MoveIt execution server running inside the ROS2 docker container.

This node (pragmabot_node.py) runs under ROS1. MoveIt/the arm run under
ROS2 in a separate docker container. Rather than bridging ROS1<->ROS2 TF and
services (painful, unnecessary for a single fixed command), a small HTTP
server inside the ROS2 container wraps MoveIt and accepts plain JSON pose
requests. See docs/moveit_server_skeleton.py for that server's counterpart.
"""

from __future__ import annotations
… +43 lines

4. A skeleton for the server side (drop this inside the ROS2 docker container — it needs your MoveIt-Py specifics filled in):

  Listed 1 directory

● Update(src/pragmabot/moveit_client.py)
Added 1 line, removed 1 line
ROS2 in a separate docker container. Rather than bridging ROS1<->ROS2 TF and
services (painful, unnecessary for a single fixed command), a small HTTP
server inside the ROS2 container wraps MoveIt and accepts plain JSON pose
requests. See docs/moveit_server_skeleton.py for that server's counterpart.
requests. See ros2_execution_server/moveit_server.py for that server's counterpart.
"""

from __future__ import annotations

● Write(/mnt/d/Armin/pragmabot/pragmabot/ros2_execution_server/moveit_server.py)
Wrote 94 lines to ../../ros2_execution_server/moveit_server.py
#!/usr/bin/env python3
"""Small HTTP server wrapping MoveIt -- run this INSIDE the ros2_humble docker container.

Why this exists: pragmabot_node.py runs under ROS1 (rospy), but the arm and
MoveIt run under ROS2 in a separate container. Bridging ROS1<->ROS2 TF and
services for a single "go grasp this pose" command is unnecessary complexity.
Instead this server exposes one plain HTTP endpoint; pragmabot_node.py's
MoveItClient (src/pragmabot/moveit_client.py) just POSTs a JSON pose to it.

STATUS: draft / not yet run against the real robot. The request/response
… +84 lines

5. Wire it into pragmabot_node.py, replacing the NotImplementedError:

● Update(nodes/pragmabot_node.py)
Added 3 lines, removed 1 line
import rospy
from openai import OpenAI

from pragmabot.grasp_pipeline import GraspExecutionPipeline
from pragmabot.moveit_client import MoveItClient
from pragmabot.scene_observer import SceneObserver
from pragmabot.simple_config import get_config
from pragmabot.utils import get_package_path
from pragmabot.vlm_success_detector import VLMSuccessDetector
from pragmabot.vlm_task_planner import VLMTaskPlanner
from pragmabot.vlm_task_planner import RobotSkill, VLMTaskPlanner
from pragmabot.vlm_client import VLMClient
from pragmabot.vlm_exp_summarizer import VLMExperienceSummarizer
from pragmabot.conversation_builder import ConversationBuilder

● Update(nodes/pragmabot_node.py)
Added 4 lines
        self.config = get_config()
        self.scene_observer = SceneObserver(self.config.topics)

        if not self.config.rosbag_replay:
            self.grasp_pipeline = GraspExecutionPipeline(self.config)
            self.moveit_client = MoveItClient(self.config)

        # If vlm model contains gpt, initialize the client
        if "gpt" in self.config.vlm.vlm_model:
            self.client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

Now the actual execution branch:

● Update(nodes/pragmabot_node.py)
Added 26 lines, removed 1 line
            # Send a request for success detection if no action execution
            self.handle_evaluation_request(chatbot)
        else:
            self.execute_action(next_action)
            self.handle_evaluation_request(chatbot)

    def execute_action(self, next_action) -> None:
        """Execute the planned action on the real robot.

        Only PICK is wired up so far: segment the target object, generate a
        grasp, cross into the robot frame, and execute it via MoveIt. PLACE
        and PUSH follow a similar chain but aren't implemented yet.

        Args:
            next_action: The VLM's chosen action (NextBestAction).
        """
        if next_action.chosen_skill != RobotSkill.PICK:
            raise NotImplementedError(
                "Implement your own action execution logic here, and then call the success detection service after executing the a
ction in the real world."
                f"Execution for skill '{next_action.chosen_skill}' is not implemented yet. "
                "Only PICK is wired up so far."
            )

        grasp_pose_robot_frame = self.grasp_pipeline.get_grasp_pose_robot_frame(
            self.color_image,
            self.depth_image,
            self.intrinsics,
            next_action.target_object,
        )
        success = self.moveit_client.execute_grasp(grasp_pose_robot_frame)
        if not success:
            rospy.logerr(f"Grasp execution failed for target_object={next_action.target_object!r}")

    def handle_evaluation_request(self, chatbot):
        """Evaluate action success by comparing before/after observations.


Let me verify the full edited section reads correctly.

  Read 1 file

That's wired correctly. Now let's sanity-check the new modules with a syntax check (no GPU/ROS deps needed for that).

  Ran 1 shell command

All files compile cleanly. Here's the summary of what changed and what's left for you.

What I did

┌─────────────────────────────────────────┬───────────────────────────────────────────────────────────────────────────────────────────────────┐
│                  File                   │                                               What                                                │
├─────────────────────────────────────────┼───────────────────────────────────────────────────────────────────────────────────────────────────┤
│ config/config.yaml                      │ Added an execution: section — scratch dir, path to the easy_handeye2 calibration file, paths to   │
│                                         │ the Grounded-SAM-2 / GraspGen venvs, and a moveit_server_url.                                     │
├─────────────────────────────────────────┼───────────────────────────────────────────────────────────────────────────────────────────────────┤
│                                         │ load_camera_to_robot_transform() reads your .calib.yaml and builds a RigidTransform once.         │
│ src/pragmabot/grasp_pipeline.py (new)   │ GraspExecutionPipeline.get_grasp_pose_robot_frame() runs the 3-step chain — writes                │
│                                         │ rgb/depth/intrinsics to disk, calls detect_object.py (Grounded-SAM-2 venv), calls                 │
│                                         │ mask_to_pointcloud.py, calls GraspGen — then applies the transform.                               │
├─────────────────────────────────────────┼───────────────────────────────────────────────────────────────────────────────────────────────────┤
│ src/pragmabot/moveit_client.py (new)    │ Plain HTTP client — POSTs the robot-frame pose to a server. Sidesteps ROS1↔ROS2 bridging          │
│                                         │ entirely.                                                                                         │
├─────────────────────────────────────────┼───────────────────────────────────────────────────────────────────────────────────────────────────┤
│ ros2_execution_server/moveit_server.py  │ Draft skeleton to copy into your ROS2 docker container — receives the HTTP pose, is supposed to   │
│ (new)                                   │ call MoveIt-Py to plan+execute.                                                                   │
├─────────────────────────────────────────┼───────────────────────────────────────────────────────────────────────────────────────────────────┤
│ nodes/pragmabot_node.py                 │ Replaced the raise NotImplementedError(...) with real dispatch: for PICK actions, calls the       │
│                                         │ pipeline then MoveIt client.                                                                      │
└─────────────────────────────────────────┴───────────────────────────────────────────────────────────────────────────────────────────────────┘

The one piece you need to fill in yourself

In grasp_pipeline.py's _run_graspgen() — I don't have the GraspGen repo on this machine, so I don't know its exact output format. Run:

~/GraspGen/.venv/bin/python ~/GraspGen/client-server/graspgen_client.py --help

and tell me what output flags it has (or paste the --help text) — I'll fill in the parsing.

Also needs your input

The moveit_server.py skeleton is untested — the HTTP plumbing is done, but the actual moveit_py planning/execution calls (marked TODO) need to match whatever your franka_fr3_moveit_config moveit.launch.py setup exposes. Want me to help write that part once you can share what's available in that container (moveit_py version, existing example scripts)?


❯ i have some question you mean that we do not something like bridge because as i know pragmabot is with ros1 and moveit is with ros2. do we need also code file like grasp_pipeline in pragmabot? i could copy graspgen repos to the project and then use it. what do you think? also same questions for sam grounded and moveit.

Good questions — let me clarify each.

1. Bridge — yes, you understood right

pragmabot_node.py uses rospy (ROS1). MoveIt/the arm run via franka_fr3_moveit_config in the ros2_humble docker container (ROS2). Those are two different, incompatible middlewares — a ROS1 node cannot call a ROS2 service directly. The two ways to cross that gap are:

- ros1_bridge: a real bridge process that translates ROS1↔ROS2 topics/services. Works, but is its own separate thing to install/run/debug, and mainly pays off if you need many different message types flowing both ways continuously.
- Plain HTTP (what I set up): the ROS2 container just also runs a tiny web server. pragmabot_node.py sends it one JSON pose over HTTP, gets back "success: true/false". No ROS2 client library needed on the ROS1 side at all.

Since you only need to send one command ("go grasp this pose") and get one result back, HTTP is much less machinery than standing up ros1_bridge. That's the reasoning — not "avoid bridging as a rule," just "don't build a general-purpose bridge for a single request/response."

2. Do we need grasp_pipeline.py in pragmabot?

Yes — regardless of where GraspGen/Grounded-SAM's code physically lives, something has to be the conductor that:
1. saves the current camera frame to disk,
2. calls Grounded-SAM with the right prompt,
3. calls the point-cloud script,
4. calls GraspGen,
5. applies the camera→robot transform.

That conductor is grasp_pipeline.py. It doesn't reimplement GraspGen or Grounded-SAM — it just calls them in order and passes files between them. This file has to exist somewhere in pragmabot no matter what you decide about copying the other repos in.

3. Copying GraspGen / Grounded-SAM into the project

You can, but it won't remove the subprocess pattern — and here's why: the reason they're called via subprocess isn't location, it's Python environment conflicts. Grounded-SAM-2 and GraspGen each need their own specific versions of torch, cv2, etc., which very likely clash with each other and with pragmabot_node.py's own dependencies (it needs rospy, gradio, openai...). That's a separate venv problem, and copying the repo's files into your project folder doesn't merge its dependencies into your venv.

4. Same question for MoveIt

Different situation, harder no: MoveIt isn't a Python dependency issue, it's ROS2 itself. You can't "copy MoveIt into pragmabot" and call it from a ROS1 venv — ROS1 and ROS2 are different frameworks with different build systems (catkin vs colcon), different message serialization, different everything. There's no subprocess trick that fixes this the way it does for a venv conflict. It genuinely has to run as a separate ROS2 process (your existing ros2_humble docker container), and you talk to it via the bridge, or via the small HTTP server. That part of the design doesn't have a simpler alternative.




1. config/config.yaml — the settings sheet

What it is: A plain text file listing paths and settings — nothing runs from this file itself.

Why needed: Instead of hardcoding things like "where is the calibration file" or "which Python to use for GraspGen" directly inside the code, you put them here once. If a path changes (e.g. you move GraspGen to a different folder), you edit one line in this file instead of hunting through code.

Analogy: It's like a contacts list — the code "calls" whichever address is written down here.

---

2. src/pragmabot/grasp_pipeline.py — the conductor

What it is: The orchestrator. It doesn't do segmentation, point clouds, or grasp-picking itself — it calls the other tools in order and passes files between them.

Why needed: Right now you have three separate working tools (Grounded-SAM, the point-cloud script, GraspGen) that were tested individually. Nothing was gluing them together into one automatic sequeniven "grasp the red cup", it:
1. Saves the current camera picture + depth to disk.                                                                                             2. Calls Grounded-SAM (detect_object.py) → gets a mask.
3. Calls mask_to_pointcloud.py → gets a 3D point cloud of just the cup.                                                                          4. Calls GraspGen → gets a grasp pose (still in camera's p
5. Applies the camera→robot transform → grasp pose is now in the robot's point of view.                                                          
Analogy: Like a kitchen line cook who doesn't cook every dish themselves, but passes plates from station to station in the right order.          
---                                                                                                                                              
3. src/pragmabot/moveit_client.py — the messenger                                                                                                
What it is: A small piece of code whose only job is: take the final grasp pose (now in robot coordinates) and send it over the network (HTTP) to wherever MoveIt is running.

Why needed: pragmabot_node.py runs in ROS1 and can't direchis file is the "phone call" that crosses that gap — itjust sends a message (pose) and waits for a reply ("done" / "failed"). It doesn't know or care how MoveIt moves the arm — that's the next file's
job.

Analogy: Like ordering food through a delivery app — you s or need to know how the kitchen cooks it.

---

4. ros2_execution_server/moveit_server.py — the receiver (

What it is: The other end of that phone call — a small webROS2 docker container, next to MoveIt.

Why needed: Someone has to be listening on the ROS2 side tger (moveit_client.py) sends, and actually tell MoveIt"move the arm here, then close the gripper." This is that listener. This is the one file that's still just a draft — the network-receiving part is done, but the actual "call MoveIt to plan and execute" pard in with your specific MoveIt setup.
                                                                                                                                                  Analogy: If moveit_client.py is placing the phone call, thhe phone on the other end and actually walking to thekitchen to make it happen.                                                                                                                        
---                                                                                                                                               
5. nodes/pragmabot_node.py — wiring it all together                                                                                               
What it is: Your main robot brain file — the one that already decides what to do (via the VLM). It previously just crashed with "not implemented" once a decision was made.

Why changed: I replaced that crash with: "if the decided ator (grasp_pipeline.py) to get a grasp pose, then hand itto the messenger (moveit_client.py) to actually execute it." This is the one line that turns "the robot decided to pick up the cup" into "the
robot's arm actually moves."

---
                                                                                                                                                   In one sentence per file: config = settings, grasp_pipelinasp chain in order, moveit_client.py = sends the resultacross the ROS1/ROS2 gap, moveit_server.py = receives it and should tell MoveIt to move, pragmabot_node.py = the point where "decide" now connects to "act."


                                                                                                                            

Good catch on your part — and there's more here than I saw. Let me read what already exists.

  Listed 1 directory, ran 5 shell commands

You're right, and the question surfaced two things I should have caught before writing.

The client's location is correct; the server's isn't.

moveit_client.py runs in the ROS1 process (pragmabot_node.py imports it), so it belongs in pragmabot/.../src/pragmabot/. That one's fine.

moveit_server.py runs inside the container, so it should be under franka_ros2/. Your docker-compose.yml:18 mounts ./:/ros2_ws/src — everything in franka_ros2/ is already inside the container and colcon-buildable. The pragmabot folder is only reachable via docker-compose.yml:22, /home/harish/pragmabot:/pragmabot — a path from a different machine that doesn't exist here (no /home/harish, and no /home/user_armin/pragmabot either). So as placed, the server sits in a directory the container can't reliably see, and can't import rclpy/franka_msgs from where it lives.                                                                                                                            

Why there and not in pragmabot/: the server runs inside franka_ros2_humble and imports rclpy, moveit_msgs, franka_msgs. Only franka_ros2/ is bind-mounted into the container at a path that exists on your machine. The pragmabot folder is mounted via /home/harish/pragmabot:/pragmabot — a path from a different machine; it doesn't exist here, so the old location was effectively invisible to the container.

What changed

┌─────────┬───────────────────────────────────────────────────────────────────────────────────────────────────────┐
│ Action  │                                                 Path                                                  │
├─────────┼───────────────────────────────────────────────────────────────────────────────────────────────────────┤
│ Deleted │ pragmabot/ros2_execution_server/ (file + __pycache__)                                                 │
├─────────┼───────────────────────────────────────────────────────────────────────────────────────────────────────┤
│ Added   │ franka_ros2/pragmabot_moveit_server/moveit_server.py (the full 36 KB version, moved intact, chmod +x) │
├─────────┼───────────────────────────────────────────────────────────────────────────────────────────────────────┤
│ Added   │ franka_ros2/pragmabot_moveit_server/COLCON_IGNORE                                                     │
├─────────┼───────────────────────────────────────────────────────────────────────────────────────────────────────┤
│ Edited  │ pragmabot/.../src/pragmabot/moveit_client.py:7 — docstring now points at the new path                 │
├─────────┼───────────────────────────────────────────────────────────────────────────────────────────────────────┤
│ Edited  │ server docstring USAGE line — now the absolute in-container path                                      │
└─────────┴───────────────────────────────────────────────────────────────────────────────────────────────────────┘

COLCON_IGNORE is there because this is a plain stdlib script you launch with python3, not an ament package — without it colcon build would scan the directory and find no package.xml. If you later want ros2 run pragmabot_moveit_server moveit_server, delete that file and add a package.xml + setup.py (ament_python); nothing in the script needs to change for that.


To run it

docker compose exec franka_ros2_humble bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash          # for franka_msgs
PRAGMABOT_DRY_RUN=1 python3 /ros2_ws/src/pragmabot_moveit_server/moveit_server.py

Then from the ROS 1 side: curl http://localhost:9000/health. network_mode: host means no port mapping is needed. Start with PRAGMABOT_DRY_RUN=1 to exercise the whole ROS1→HTTP→ROS2 chain before the arm actually moves.

Your system is split across two isolated worlds that cannot see each other's files or talk to each other directly:

┌─────────────────────────────────┐         ┌──────────────────────────────────┐
│  HOST machine                   │         │  DOCKER CONTAINER                │
│  (Windows/WSL, /mnt/d/Armin/…)  │         │  franka_ros2_humble              │
│                                 │         │                                  │
│  pragmabot_node.py              │   ???   │  MoveIt (move_group)             │
│  ROS 1  (rospy)                 │ ──────► │  ROS 2 Humble (rclpy)            │
│  Grounded-SAM, GraspGen, ZED    │         │  franka driver, gripper          │
└─────────────────────────────────┘         └──────────────────────────────────┘

A container is a sealed box. It only sees host folders that are explicitly mounted into it. And ROS 1 and ROS 2 use different, incompatible middleware — a rospy node cannot call a ROS 2 action.

So there are two gaps to cross, and they need two different solutions:

┌───────────────────────────────┬────────────────────────────────────────────────────────────────────────────────┐
│              Gap              │                                    Solution                                    │
├───────────────────────────────┼────────────────────────────────────────────────────────────────────────────────┤
│ ROS 1 ↔ ROS 2 (communication) │ A tiny HTTP server. moveit_client.py sends JSON, moveit_server.py receives it. │
├───────────────────────────────┼────────────────────────────────────────────────────────────────────────────────┤
│ Host ↔ container (files)      │ A Docker volume mount. This is what the move was about.                        │
└───────────────────────────────┴────────────────────────────────────────────────────────────────────────────────┘

The sentence for your professor

▎ The MoveIt server is ROS 2 code, so it must run inside the container. Docker only exposes host folders that are bind-mounted, and the project's only working mount is franka_ros2/ → /ros2_ws/src; a second mount in docker-compose.yml points at a path from a collaborator's machine and resolves to an empty directory here. The server was therefore moved out of the ROS 1 application folder into the mounted ROS 2 workspace, where it is visible to the container and editable live from the host. Its HTTP client stays on the ROS 1 side, since that is the process that imports it. The HTTP boundary between them is what replaces ros1_bridge: one request/response pair does not justify a general-purpose middleware bridge.




#!/usr/bin/env python3
"""HTTP server wrapping MoveIt -- run this INSIDE the ros2_humble docker container.

Why this exists: pragmabot_node.py runs under ROS1 (rospy), but the arm and
MoveIt run under ROS2 in a separate container. Bridging ROS1<->ROS2 TF and
services for a handful of "go grasp this pose" commands is unnecessary
complexity. Instead this server exposes a few plain HTTP endpoints;
pragmabot_node.py's MoveItClient (src/pragmabot/moveit_client.py) POSTs JSON
poses to it.

It talks to the already-running `move_group` node (started by
`ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=...`) through
its standard interfaces, so nothing has to be added to that launch file:

    /move_group             (action, moveit_msgs/MoveGroup)     free-space plan+execute
    /compute_cartesian_path (service, moveit_msgs/GetCartesianPath) straight-line segments
    /execute_trajectory     (action, moveit_msgs/ExecuteTrajectory)  runs the above
    /franka_gripper/{move,grasp,homing} (actions, franka_msgs)   the hand

USAGE (inside the container, alongside the running moveit.launch.py):

    source /opt/ros/humble/setup.bash
    source ~/franka_ros2_ws/install/setup.bash      # for franka_msgs
    python3 /ros2_ws/src/pragmabot_moveit_server/moveit_server.py   # stdlib only, no pip installs

Environment overrides (all optional):
    PRAGMABOT_MOVEIT_HOST   default 0.0.0.0
    PRAGMABOT_MOVEIT_PORT   default 9000
    PRAGMABOT_GRIPPER_NS    default /franka_gripper  (use /fr3_gripper if your
                            gripper node is named that -- check `ros2 action list`)
    PRAGMABOT_DRY_RUN       set to 1 to accept and log requests without moving
                            the arm (useful to test the ROS1 side end-to-end)

Endpoints (all JSON in / JSON out):
    GET  /health        -> {"ok": true, "dry_run": false, ...}
    POST /execute_grasp -> full pick: approach, close, lift
    POST /place         -> full place: approach, open, retreat
    POST /move_to_pose  -> single free-space motion to a pose
    POST /gripper       -> {"action": "open"|"close"|"grasp"|"homing", ...}
    POST /go_home       -> move to the SRDF "ready" joint configuration
    POST /stop          -> cancel whatever is executing

Every response has the shape {"success": bool, "message": str, ...}.
All poses are for the *gripper TCP* (fr3_hand_tcp) expressed in the robot base
frame (fr3_link0) -- i.e. exactly what grasp_pipeline.get_grasp_pose_robot_frame
returns after applying the camera->robot calibration.
"""

from __future__ import annotations

import json
import logging
import math
import os
import threading
import time
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Callable

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Vector3
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
    RobotState,
)
from moveit_msgs.srv import GetCartesianPath
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive

try:  # only needed for the real hand; the arm-only paths work without it
    from franka_msgs.action import Grasp as FrankaGrasp
    from franka_msgs.action import Homing as FrankaHoming
    from franka_msgs.action import Move as FrankaMove
    from franka_msgs.msg import GraspEpsilon

    HAVE_FRANKA_MSGS = True
except ImportError:  # pragma: no cover - depends on the container's sourced workspace
    HAVE_FRANKA_MSGS = False

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(name)s: %(message)s")
logger = logging.getLogger("moveit_server")

# --- Robot-specific names, all confirmed from franka_ros2 in this repo ---------
# group + tip link: franka_description/robots/common/group_definition.xacro
#                   (group "${prefix}arm" with prefix "fr3_", tip "fr3_hand_tcp")
# base frame:       chain base_link "${prefix}link0"
# joint names:      franka_fr3_moveit_config/config/fr3_controllers.yaml
PLANNING_GROUP = "fr3_arm"
ROBOT_BASE_FRAME = "fr3_link0"
END_EFFECTOR_LINK = "fr3_hand_tcp"
ARM_JOINTS = [f"fr3_joint{i}" for i in range(1, 8)]
# SRDF group_state "ready"
HOME_JOINT_POSITIONS = [0.0, -math.pi / 4, 0.0, -3 * math.pi / 4, 0.0, math.pi / 2, math.pi / 4]

GRIPPER_NS = os.environ.get("PRAGMABOT_GRIPPER_NS", "/franka_gripper").rstrip("/")
DRY_RUN = os.environ.get("PRAGMABOT_DRY_RUN", "").strip() in {"1", "true", "True"}

# Franka Hand limits / sane defaults.
GRIPPER_MAX_WIDTH = 0.08  # m, fully open
DEFAULT_GRASP_FORCE = 40.0  # N
DEFAULT_GRIPPER_SPEED = 0.1  # m/s
DEFAULT_GRASP_EPSILON = 0.03  # m, tolerance on the commanded width before "grasp failed"

# Motion defaults; every one of these can be overridden per request.
DEFAULT_APPROACH_DISTANCE = 0.10  # m, straight-line stand-off before closing
DEFAULT_RETREAT_DISTANCE = 0.15  # m, straight-line lift after closing
DEFAULT_VELOCITY_SCALING = 0.15  # deliberately slow -- this drives a real arm
DEFAULT_ACCELERATION_SCALING = 0.15
DEFAULT_PLANNING_TIME = 5.0  # s
DEFAULT_PLANNING_ATTEMPTS = 10
POSITION_TOLERANCE = 0.005  # m
ORIENTATION_TOLERANCE = 0.02  # rad
CARTESIAN_MAX_STEP = 0.005  # m between interpolated waypoints
CARTESIAN_MIN_FRACTION = 0.85  # accept a straight-line path only if this complete
ACTION_TIMEOUT_S = 120.0
SERVER_WAIT_TIMEOUT_S = 30.0


class ExecutionError(RuntimeError):
    """Raised when a planning/execution/gripper step fails; turned into a JSON error."""


@dataclass
class Step:
    """One named stage of a multi-stage motion, reported back to the caller."""

    name: str
    success: bool
    detail: str = ""

    def as_dict(self) -> dict[str, Any]:
        return {"step": self.name, "success": self.success, "detail": self.detail}


# --- small math helpers (kept dependency-free: no numpy/scipy needed here) -----


def quat_to_matrix(x: float, y: float, z: float, w: float) -> list[list[float]]:
    """Convert a (x, y, z, w) quaternion to a 3x3 rotation matrix (row-major)."""
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-9:
        raise ExecutionError("Received a zero-length quaternion.")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def offset_along_local_axis(pose: Pose, distance: float, axis: int = 2) -> Pose:
    """Translate ``pose`` by ``distance`` along one of its own (local) axes.

    The gripper approach direction is the TCP frame's +Z, which is the
    convention both GraspGen and franka_description's ``fr3_hand_tcp`` use, so
    a pre-grasp pose is the grasp pose offset by ``-approach`` along local Z.

    Args:
        pose: Pose in the robot base frame.
        distance: Signed offset in metres.
        axis: 0/1/2 for the local X/Y/Z axis.

    Returns:
        A new Pose with the same orientation and the shifted position.
    """
    q = pose.orientation
    rotation = quat_to_matrix(q.x, q.y, q.z, q.w)
    direction = [rotation[0][axis], rotation[1][axis], rotation[2][axis]]

    shifted = Pose()
    shifted.orientation = pose.orientation
    shifted.position.x = pose.position.x + distance * direction[0]
    shifted.position.y = pose.position.y + distance * direction[1]
    shifted.position.z = pose.position.z + distance * direction[2]
    return shifted


def offset_along_world_z(pose: Pose, distance: float) -> Pose:
    """Translate ``pose`` straight up in the robot base frame (used for lifting)."""
    lifted = Pose()
    lifted.orientation = pose.orientation
    lifted.position.x = pose.position.x
    lifted.position.y = pose.position.y
    lifted.position.z = pose.position.z + distance
    return lifted


def pose_from_request(body: dict[str, Any]) -> Pose:
    """Build a geometry_msgs/Pose from the JSON the ROS1 client sends.

    Args:
        body: Parsed request body with ``position`` [x, y, z] in metres and
            ``orientation_xyzw`` [x, y, z, w].

    Returns:
        The Pose, in ROBOT_BASE_FRAME.

    Raises:
        ExecutionError: If either field is missing or the wrong length.
    """
    position = body.get("position")
    orientation = body.get("orientation_xyzw")
    if not isinstance(position, (list, tuple)) or len(position) != 3:
        raise ExecutionError(f"'position' must be [x, y, z], got {position!r}")
    if not isinstance(orientation, (list, tuple)) or len(orientation) != 4:
        raise ExecutionError(f"'orientation_xyzw' must be [x, y, z, w], got {orientation!r}")

    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (float(v) for v in position)
    pose.orientation = Quaternion(
        x=float(orientation[0]), y=float(orientation[1]), z=float(orientation[2]), w=float(orientation[3])
    )
    return pose


def error_code_name(code: int) -> str:
    """Human-readable name for a moveit_msgs/MoveItErrorCodes value."""
    for name, value in vars(MoveItErrorCodes).items():
        if name.isupper() and isinstance(value, int) and value == code:
            return name
    return f"UNKNOWN({code})"


class MoveItExecutor(Node):
    """Thin synchronous wrapper around move_group's actions and services.

    Each public method blocks until the motion is done and raises
    ``ExecutionError`` on failure, so the HTTP layer above stays trivial.
    """

    def __init__(self) -> None:
        super().__init__("pragmabot_moveit_server")
        # Reentrant group: HTTP handler threads wait on futures while the
        # executor thread keeps spinning callbacks that resolve them.
        self.callback_group = ReentrantCallbackGroup()

        self.move_group_client = ActionClient(self, MoveGroup, "/move_group", callback_group=self.callback_group)
        self.execute_client = ActionClient(
            self, ExecuteTrajectory, "/execute_trajectory", callback_group=self.callback_group
        )
        self.cartesian_client = self.create_client(
            GetCartesianPath, "/compute_cartesian_path", callback_group=self.callback_group
        )

        self.gripper_move_client = None
        self.gripper_grasp_client = None
        self.gripper_homing_client = None
        if HAVE_FRANKA_MSGS:
            self.gripper_move_client = ActionClient(
                self, FrankaMove, f"{GRIPPER_NS}/move", callback_group=self.callback_group
            )
            self.gripper_grasp_client = ActionClient(
                self, FrankaGrasp, f"{GRIPPER_NS}/grasp", callback_group=self.callback_group
            )
            self.gripper_homing_client = ActionClient(
                self, FrankaHoming, f"{GRIPPER_NS}/homing", callback_group=self.callback_group
            )

        # One motion at a time -- the arm is a single shared resource.
        self.motion_lock = threading.Lock()
        self._active_goal_handle = None

    # --- infrastructure -------------------------------------------------------

    def wait_for_interfaces(self, timeout_s: float = SERVER_WAIT_TIMEOUT_S) -> None:
        """Block until move_group is up, so the first grasp doesn't time out.

        Raises:
            ExecutionError: If move_group is still absent after ``timeout_s``.
        """
        if DRY_RUN:
            logger.warning("PRAGMABOT_DRY_RUN set -- not waiting for move_group, no motion will happen.")
            return

        if not self.move_group_client.wait_for_server(timeout_sec=timeout_s):
            raise ExecutionError(
                "/move_group action server not available. Is "
                "`ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=...` running "
                "in this container, with the same ROS_DOMAIN_ID?"
            )
        if not self.cartesian_client.wait_for_service(timeout_sec=timeout_s):
            logger.warning("/compute_cartesian_path unavailable -- approach/retreat will fall back to free-space plans.")
        if HAVE_FRANKA_MSGS:
            if not self.gripper_grasp_client.wait_for_server(timeout_sec=5.0):
                logger.warning(
                    "Gripper actions not found under %s. Set PRAGMABOT_GRIPPER_NS to the right "
                    "namespace (check `ros2 action list`); gripper commands will fail until then.",
                    GRIPPER_NS,
                )
        else:
            logger.warning("franka_msgs not importable -- source your franka_ros2 workspace for gripper control.")
        logger.info("move_group is up; ready to accept requests.")

    def _spin_future(self, future, timeout_s: float, what: str):
        """Wait for an rclpy future resolved by the background executor.

        ``spin_until_future_complete`` can't be used here: this node is already
        being spun by a MultiThreadedExecutor in another thread.

        Args:
            future: The rclpy future to wait on.
            timeout_s: Maximum wait in seconds.
            what: Label used in the timeout message.

        Returns:
            The future's result.

        Raises:
            ExecutionError: On timeout.
        """
        deadline = time.monotonic() + timeout_s
        while not future.done():
            if time.monotonic() > deadline:
                future.cancel()
                raise ExecutionError(f"Timed out after {timeout_s:.0f}s waiting for {what}.")
            time.sleep(0.02)
        return future.result()

    def _run_action(self, client: ActionClient, goal_msg, what: str, timeout_s: float = ACTION_TIMEOUT_S):
        """Send an action goal and block until it finishes.

        Args:
            client: The action client to use.
            goal_msg: The goal message.
            what: Label for log/error messages.
            timeout_s: Maximum wait for the whole goal.

        Returns:
            The action result message.

        Raises:
            ExecutionError: If the server is missing, rejects the goal, or the
                goal does not reach SUCCEEDED.
        """
        if client is None:
            raise ExecutionError(f"{what}: action client unavailable (franka_msgs not sourced?).")
        if not client.wait_for_server(timeout_sec=5.0):
            raise ExecutionError(f"{what}: action server not available.")

        goal_handle = self._spin_future(client.send_goal_async(goal_msg), timeout_s, f"{what} (goal acceptance)")
        if not goal_handle.accepted:
            raise ExecutionError(f"{what}: goal rejected by the action server.")

        self._active_goal_handle = goal_handle
        try:
            wrapped = self._spin_future(goal_handle.get_result_async(), timeout_s, f"{what} (result)")
        finally:
            self._active_goal_handle = None

        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            raise ExecutionError(f"{what}: goal ended with status {wrapped.status} (not SUCCEEDED).")
        return wrapped.result

    def cancel_active_goal(self) -> bool:
        """Cancel whatever goal is currently executing. Returns True if one was active."""
        handle = self._active_goal_handle
        if handle is None:
            return False
        handle.cancel_goal_async()
        return True

    # --- motion primitives ----------------------------------------------------

    def _goal_constraints(self, pose: Pose) -> Constraints:
        """Build position + orientation goal constraints for the TCP.

        Equivalent to MoveIt's ``constructGoalConstraints``: a small sphere
        around the target point plus an angular tolerance on the target
        orientation.
        """
        header_frame = ROBOT_BASE_FRAME

        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = header_frame
        position_constraint.link_name = END_EFFECTOR_LINK
        position_constraint.target_point_offset = Vector3(x=0.0, y=0.0, z=0.0)
        position_constraint.weight = 1.0

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [POSITION_TOLERANCE]

        region = BoundingVolume()
        region.primitives = [sphere]
        region_pose = Pose()
        region_pose.position = pose.position
        region_pose.orientation.w = 1.0
        region.primitive_poses = [region_pose]
        position_constraint.constraint_region = region

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = header_frame
        orientation_constraint.link_name = END_EFFECTOR_LINK
        orientation_constraint.orientation = pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = ORIENTATION_TOLERANCE
        orientation_constraint.absolute_y_axis_tolerance = ORIENTATION_TOLERANCE
        orientation_constraint.absolute_z_axis_tolerance = ORIENTATION_TOLERANCE
        orientation_constraint.weight = 1.0

        constraints = Constraints()
        constraints.name = "pragmabot_pose_goal"
        constraints.position_constraints = [position_constraint]
        constraints.orientation_constraints = [orientation_constraint]
        return constraints

    def _joint_constraints(self, positions: list[float]) -> Constraints:
        """Build joint-space goal constraints (used by /go_home)."""
        constraints = Constraints()
        constraints.name = "pragmabot_joint_goal"
        constraints.joint_constraints = []
        for joint_name, value in zip(ARM_JOINTS, positions):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = float(value)
            joint_constraint.tolerance_above = 0.01
            joint_constraint.tolerance_below = 0.01
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)
        return constraints

    def _plan_and_execute(self, constraints: Constraints, what: str, velocity_scaling: float, acceleration_scaling: float) -> None:
        """Plan a free-space motion to ``constraints`` and execute it.

        Raises:
            ExecutionError: If planning or execution reports anything but SUCCESS.
        """
        if DRY_RUN:
            logger.info("[dry-run] would plan+execute: %s", what)
            return

        goal = MoveGroup.Goal()
        goal.request.group_name = PLANNING_GROUP
        goal.request.goal_constraints = [constraints]
        goal.request.num_planning_attempts = DEFAULT_PLANNING_ATTEMPTS
        goal.request.allowed_planning_time = DEFAULT_PLANNING_TIME
        goal.request.max_velocity_scaling_factor = velocity_scaling
        goal.request.max_acceleration_scaling_factor = acceleration_scaling
        # Empty + is_diff means "start from wherever the robot currently is".
        goal.request.start_state = RobotState()
        goal.request.start_state.is_diff = True
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 2

        result = self._run_action(self.move_group_client, goal, what)
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise ExecutionError(f"{what}: MoveIt returned {error_code_name(result.error_code.val)}.")

    def move_to_pose(
        self,
        pose: Pose,
        what: str = "move to pose",
        velocity_scaling: float = DEFAULT_VELOCITY_SCALING,
        acceleration_scaling: float = DEFAULT_ACCELERATION_SCALING,
    ) -> None:
        """Free-space (collision-aware, non-straight-line) motion of the TCP to ``pose``."""
        logger.info(
            "%s -> p=(%.3f, %.3f, %.3f) q=(%.3f, %.3f, %.3f, %.3f)",
            what,
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        self._plan_and_execute(self._goal_constraints(pose), what, velocity_scaling, acceleration_scaling)

    def move_cartesian(
        self,
        target: Pose,
        what: str,
        velocity_scaling: float = DEFAULT_VELOCITY_SCALING,
        acceleration_scaling: float = DEFAULT_ACCELERATION_SCALING,
    ) -> None:
        """Move the TCP to ``target`` in a straight line from the current pose.

        Used for the approach and the lift: a free-space plan there could swing
        the gripper through the object it is about to grasp (or the one it is
        already holding). Falls back to a free-space plan if the Cartesian
        service is unavailable, and raises if the straight-line path can only
        be partially computed.
        """
        if DRY_RUN:
            logger.info("[dry-run] would move cartesian: %s", what)
            return
        if not self.cartesian_client.service_is_ready():
            logger.warning("%s: /compute_cartesian_path unavailable, falling back to a free-space plan.", what)
            self.move_to_pose(target, what, velocity_scaling, acceleration_scaling)
            return

        request = GetCartesianPath.Request()
        request.header.frame_id = ROBOT_BASE_FRAME
        request.start_state = RobotState()
        request.start_state.is_diff = True
        request.group_name = PLANNING_GROUP
        request.link_name = END_EFFECTOR_LINK
        request.waypoints = [target]
        request.max_step = CARTESIAN_MAX_STEP
        request.avoid_collisions = True
        # Field names differ slightly across MoveIt 2 releases; set what exists.
        for field, value in (
            ("jump_threshold", 0.0),
            ("prismatic_jump_threshold", 0.0),
            ("revolute_jump_threshold", 0.0),
            ("max_velocity_scaling_factor", velocity_scaling),
            ("max_acceleration_scaling_factor", acceleration_scaling),
        ):
            if hasattr(request, field):
                setattr(request, field, value)

        response = self._spin_future(self.cartesian_client.call_async(request), ACTION_TIMEOUT_S, what)
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise ExecutionError(f"{what}: Cartesian planning returned {error_code_name(response.error_code.val)}.")
        if response.fraction < CARTESIAN_MIN_FRACTION:
            raise ExecutionError(
                f"{what}: only {response.fraction * 100:.0f}% of the straight-line path was reachable "
                f"(need {CARTESIAN_MIN_FRACTION * 100:.0f}%). The target is probably out of reach or blocked."
            )

        execute_goal = ExecuteTrajectory.Goal()
        execute_goal.trajectory = response.solution
        result = self._run_action(self.execute_client, execute_goal, f"{what} (execute)")
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise ExecutionError(f"{what}: execution returned {error_code_name(result.error_code.val)}.")

    # --- gripper --------------------------------------------------------------

    def gripper_move(self, width: float, speed: float = DEFAULT_GRIPPER_SPEED) -> None:
        """Open/position the fingers to ``width`` metres without applying force."""
        width = max(0.0, min(float(width), GRIPPER_MAX_WIDTH))
        if DRY_RUN:
            logger.info("[dry-run] would move gripper to %.3f m", width)
            return
        goal = FrankaMove.Goal()
        goal.width = width
        goal.speed = float(speed)
        result = self._run_action(self.gripper_move_client, goal, f"gripper move to {width:.3f}m", timeout_s=20.0)
        if not result.success:
            raise ExecutionError(f"Gripper move failed: {result.error}")

    def gripper_grasp(
        self,
        width: float,
        force: float = DEFAULT_GRASP_FORCE,
        speed: float = DEFAULT_GRIPPER_SPEED,
        epsilon: float = DEFAULT_GRASP_EPSILON,
    ) -> None:
        """Close the fingers on an object of roughly ``width`` metres.

        franka_msgs/Grasp only reports success if the final width lands within
        ``epsilon`` of the commanded width, which doubles as a cheap
        "did I actually pick something up" check.
        """
        width = max(0.0, min(float(width), GRIPPER_MAX_WIDTH))
        if DRY_RUN:
            logger.info("[dry-run] would grasp at %.3f m with %.0f N", width, force)
            return
        goal = FrankaGrasp.Goal()
        goal.width = width
        goal.speed = float(speed)
        goal.force = float(force)
        goal.epsilon = GraspEpsilon(inner=float(epsilon), outer=float(epsilon))
        result = self._run_action(self.gripper_grasp_client, goal, f"gripper grasp at {width:.3f}m", timeout_s=20.0)
        if not result.success:
            raise ExecutionError(f"Grasp failed (nothing between the fingers?): {result.error}")

    def gripper_homing(self) -> None:
        """Recalibrate the hand's maximum width. Run this once after powering up."""
        if DRY_RUN:
            logger.info("[dry-run] would home the gripper")
            return
        result = self._run_action(self.gripper_homing_client, FrankaHoming.Goal(), "gripper homing", timeout_s=30.0)
        if not result.success:
            raise ExecutionError(f"Gripper homing failed: {result.error}")

    # --- composite skills -----------------------------------------------------

    def execute_grasp(self, body: dict[str, Any]) -> list[Step]:
        """Full pick: pre-grasp, open, approach, close, lift.

        Args:
            body: Request body. ``position``/``orientation_xyzw`` are required;
                ``approach_distance``, ``retreat_distance``, ``gripper_width``,
                ``grasp_force``, ``open_width``, ``velocity_scaling`` optional.

        Returns:
            One Step per stage, all successful (a failed stage raises instead).
        """
        grasp_pose = pose_from_request(body)
        approach = float(body.get("approach_distance", DEFAULT_APPROACH_DISTANCE))
        retreat = float(body.get("retreat_distance", DEFAULT_RETREAT_DISTANCE))
        open_width = float(body.get("open_width", GRIPPER_MAX_WIDTH))
        grasp_width = float(body.get("gripper_width", 0.0))
        force = float(body.get("grasp_force", DEFAULT_GRASP_FORCE))
        velocity = float(body.get("velocity_scaling", DEFAULT_VELOCITY_SCALING))
        acceleration = float(body.get("acceleration_scaling", DEFAULT_ACCELERATION_SCALING))

        pre_grasp_pose = offset_along_local_axis(grasp_pose, -approach)
        steps: list[Step] = []

        self.move_to_pose(pre_grasp_pose, "pre-grasp", velocity, acceleration)
        steps.append(Step("pre_grasp", True, f"stood off {approach:.3f}m along the approach axis"))

        self.gripper_move(open_width)
        steps.append(Step("open_gripper", True, f"opened to {open_width:.3f}m"))

        self.move_cartesian(grasp_pose, "approach", velocity, acceleration)
        steps.append(Step("approach", True, "straight-line move onto the grasp pose"))

        self.gripper_grasp(grasp_width, force=force)
        steps.append(Step("close_gripper", True, f"closed on ~{grasp_width:.3f}m at {force:.0f}N"))

        lift_pose = offset_along_world_z(grasp_pose, retreat)
        self.move_cartesian(lift_pose, "lift", velocity, acceleration)
        steps.append(Step("lift", True, f"lifted {retreat:.3f}m"))

        return steps

    def execute_place(self, body: dict[str, Any]) -> list[Step]:
        """Full place: pre-place, lower, open, retreat.

        Mirrors ``execute_grasp`` -- the same approach/retreat structure, with
        the gripper opening instead of closing at the bottom.
        """
        place_pose = pose_from_request(body)
        approach = float(body.get("approach_distance", DEFAULT_APPROACH_DISTANCE))
        retreat = float(body.get("retreat_distance", DEFAULT_RETREAT_DISTANCE))
        open_width = float(body.get("open_width", GRIPPER_MAX_WIDTH))
        velocity = float(body.get("velocity_scaling", DEFAULT_VELOCITY_SCALING))
        acceleration = float(body.get("acceleration_scaling", DEFAULT_ACCELERATION_SCALING))

        pre_place_pose = offset_along_world_z(place_pose, approach)
        steps: list[Step] = []

        self.move_to_pose(pre_place_pose, "pre-place", velocity, acceleration)
        steps.append(Step("pre_place", True, f"held {approach:.3f}m above the place pose"))

        self.move_cartesian(place_pose, "lower", velocity, acceleration)
        steps.append(Step("lower", True, "straight-line move down onto the place pose"))

        self.gripper_move(open_width)
        steps.append(Step("open_gripper", True, f"released at {open_width:.3f}m"))

        self.move_cartesian(offset_along_world_z(place_pose, retreat), "retreat", velocity, acceleration)
        steps.append(Step("retreat", True, f"retreated {retreat:.3f}m"))

        return steps

    def go_home(self, body: dict[str, Any]) -> list[Step]:
        """Move the arm to the SRDF ``ready`` configuration."""
        velocity = float(body.get("velocity_scaling", DEFAULT_VELOCITY_SCALING))
        acceleration = float(body.get("acceleration_scaling", DEFAULT_ACCELERATION_SCALING))
        self._plan_and_execute(self._joint_constraints(HOME_JOINT_POSITIONS), "go home", velocity, acceleration)
        return [Step("go_home", True, "at the 'ready' joint configuration")]

    def command_gripper(self, body: dict[str, Any]) -> list[Step]:
        """Dispatch a standalone gripper command (open/close/grasp/homing)."""
        action = str(body.get("action", "open")).lower()
        speed = float(body.get("speed", DEFAULT_GRIPPER_SPEED))

        if action == "open":
            self.gripper_move(float(body.get("width", GRIPPER_MAX_WIDTH)), speed)
        elif action == "close":
            self.gripper_move(float(body.get("width", 0.0)), speed)
        elif action == "grasp":
            self.gripper_grasp(
                float(body.get("width", 0.0)),
                force=float(body.get("force", DEFAULT_GRASP_FORCE)),
                speed=speed,
                epsilon=float(body.get("epsilon", DEFAULT_GRASP_EPSILON)),
            )
        elif action == "homing":
            self.gripper_homing()
        else:
            raise ExecutionError(f"Unknown gripper action {action!r}; expected open/close/grasp/homing.")
        return [Step(f"gripper_{action}", True)]

    def do_move_to_pose(self, body: dict[str, Any]) -> list[Step]:
        """Single free-space (or optionally straight-line) motion to a pose."""
        pose = pose_from_request(body)
        velocity = float(body.get("velocity_scaling", DEFAULT_VELOCITY_SCALING))
        acceleration = float(body.get("acceleration_scaling", DEFAULT_ACCELERATION_SCALING))
        if body.get("cartesian", False):
            self.move_cartesian(pose, "move_to_pose", velocity, acceleration)
        else:
            self.move_to_pose(pose, "move_to_pose", velocity, acceleration)
        return [Step("move_to_pose", True)]


class RequestHandler(BaseHTTPRequestHandler):
    """Maps the HTTP endpoints onto MoveItExecutor methods."""

    executor_node: MoveItExecutor = None  # set in main()
    server_version = "pragmabot-moveit/1.0"

    def log_message(self, fmt: str, *args: Any) -> None:  # quieter, and via logging
        logger.debug("%s - %s", self.address_string(), fmt % args)

    def _send_json(self, payload: dict[str, Any], status: int = 200) -> None:
        encoded = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(encoded)))
        self.end_headers()
        self.wfile.write(encoded)

    def _read_body(self) -> dict[str, Any]:
        length = int(self.headers.get("Content-Length") or 0)
        if length == 0:
            return {}
        raw = self.rfile.read(length)
        try:
            body = json.loads(raw)
        except json.JSONDecodeError as exc:
            raise ExecutionError(f"Request body is not valid JSON: {exc}")
        if not isinstance(body, dict):
            raise ExecutionError("Request body must be a JSON object.")
        return body

    def _run_motion(self, handler: Callable[[dict[str, Any]], list[Step]]) -> None:
        """Run one motion request under the arm lock and report the outcome."""
        node = self.executor_node
        try:
            body = self._read_body()
        except ExecutionError as exc:
            self._send_json({"success": False, "message": str(exc)}, status=400)
            return

        if not node.motion_lock.acquire(blocking=False):
            self._send_json(
                {"success": False, "message": "The arm is already executing another motion."}, status=409
            )
            return
        try:
            steps = handler(body)
            self._send_json(
                {"success": True, "message": "", "steps": [step.as_dict() for step in steps], "dry_run": DRY_RUN}
            )
        except ExecutionError as exc:
            logger.error("%s", exc)
            self._send_json({"success": False, "message": str(exc)})
        except Exception as exc:  # noqa: BLE001 - report to the caller, keep serving
            logger.exception("Unexpected failure while executing the request")
            self._send_json({"success": False, "message": f"{type(exc).__name__}: {exc}"})
        finally:
            node.motion_lock.release()

    def do_GET(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler naming
        if self.path.rstrip("/") in ("/health", ""):
            node = self.executor_node
            self._send_json(
                {
                    "success": True,
                    "ok": True,
                    "dry_run": DRY_RUN,
                    "planning_group": PLANNING_GROUP,
                    "base_frame": ROBOT_BASE_FRAME,
                    "end_effector": END_EFFECTOR_LINK,
                    "gripper_namespace": GRIPPER_NS if HAVE_FRANKA_MSGS else None,
                    "move_group_available": DRY_RUN or node.move_group_client.server_is_ready(),
                    "busy": node.motion_lock.locked(),
                }
            )
            return
        self._send_json({"success": False, "message": f"Unknown endpoint {self.path}"}, status=404)

    def do_POST(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler naming
        node = self.executor_node
        route = self.path.rstrip("/")
        routes: dict[str, Callable[[dict[str, Any]], list[Step]]] = {
            "/execute_grasp": node.execute_grasp,
            "/place": node.execute_place,
            "/move_to_pose": node.do_move_to_pose,
            "/gripper": node.command_gripper,
            "/go_home": node.go_home,
        }
        if route == "/stop":
            cancelled = node.cancel_active_goal()
            self._send_json({"success": True, "message": "cancel sent" if cancelled else "nothing was executing"})
            return
        if route not in routes:
            self._send_json({"success": False, "message": f"Unknown endpoint {self.path}"}, status=404)
            return
        self._run_motion(routes[route])


def main() -> None:
    host = os.environ.get("PRAGMABOT_MOVEIT_HOST", "0.0.0.0")
    port = int(os.environ.get("PRAGMABOT_MOVEIT_PORT", "9000"))

    rclpy.init()
    node = MoveItExecutor()

    ros_executor = MultiThreadedExecutor()
    ros_executor.add_node(node)
    spin_thread = threading.Thread(target=ros_executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.wait_for_interfaces()
    except ExecutionError as exc:
        # Serve anyway: the client gets a clear error per request instead of a
        # connection refused, which is much easier to debug from the ROS1 side.
        logger.error("%s", exc)

    RequestHandler.executor_node = node
    http_server = ThreadingHTTPServer((host, port), RequestHandler)
    logger.info("Serving on http://%s:%d (dry_run=%s)", host, port, DRY_RUN)
    try:
        http_server.serve_forever()
    except KeyboardInterrupt:
        logger.info("Shutting down.")
    finally:
        http_server.server_close()
        ros_executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


"""Segmentation -> point cloud -> grasp pose -> robot frame.

Crosses the calibration wall: takes a text-prompted target object plus the
current camera observation, and returns a single best grasp pose in the
ROBOT frame, ready to hand to a MoveIt execution client.

Follows the same cross-venv pattern already used by calibration/detect_object.py
and calibration/calibrate_extrinsic.py: tools that live in their own venv
(Grounded-SAM-2, GraspGen) are invoked as subprocesses, with files as the
interface, instead of imported in-process.
"""

from __future__ import annotations

import json
import logging
import subprocess
from pathlib import Path

import numpy as np
import yaml
from omegaconf import DictConfig
from PIL import Image
from scipy.spatial.transform import Rotation

from pragmabot.geometry import CameraIntrinsics, RigidTransform
from pragmabot.utils import get_package_path

logger = logging.getLogger(__name__)


def load_camera_to_robot_transform(calibration_file: str) -> RigidTransform:
    """Load the fixed camera->robot transform from an easy_handeye2 calibration file.

    Loaded once at startup, not looked up live via TF -- the camera is
    externally mounted and fixed, so this one transform never changes, and
    a live lookup would mean bridging ROS1 (this node) <-> ROS2 (the arm).

    Args:
        calibration_file: Path to the easy_handeye2 ``.calib.yaml`` file
            (contains a ``transform`` key with ``translation``/``rotation``).

    Returns:
        RigidTransform mapping points/poses from the camera frame into the
        robot base frame (``robot_base_frame`` in the calibration file).
    """
    path = Path(calibration_file).expanduser()
    if not path.is_file():
        raise FileNotFoundError(
            f"Camera->robot calibration file not found: {path}\n"
            "Check config.execution.camera_extrinsics_file, or copy the file "
            "easy_handeye2 saved (see the 'calibration_type: eye_on_base' block "
            "printed by `ros2 launch easy_handeye2 calibrate.launch.py ...`)."
        )

    data = yaml.safe_load(path.read_text())
    t = data["transform"]["translation"]
    r = data["transform"]["rotation"]

    position = np.array([t["x"], t["y"], t["z"]], dtype=np.float64)
    # scipy's default quaternion order is (x, y, z, w), matching the calibration file.
    rotation = Rotation.from_quat([r["x"], r["y"], r["z"], r["w"]]).as_matrix()

    logger.info("Loaded camera->robot transform from %s", path)
    return RigidTransform(position=position, rotation=rotation)


class GraspExecutionPipeline:
    """Runs GroundedSAM -> point cloud -> GraspGen -> robot-frame grasp pose."""

    def __init__(self, config: DictConfig) -> None:
        self.config = config.execution
        self.scratch_dir = Path(self.config.scratch_dir).expanduser()
        self.calibration_dir = get_package_path().parents[1] / "calibration"
        self.camera_to_robot = load_camera_to_robot_transform(self.config.camera_extrinsics_file)

    def get_grasp_pose_robot_frame(
        self,
        color_image: Image.Image,
        depth_image: Image.Image,
        intrinsics: CameraIntrinsics,
        target_object: str,
    ) -> RigidTransform:
        """Return the best grasp pose for ``target_object``, in the robot frame.

        Args:
            color_image: Current RGB observation.
            depth_image: Current depth observation (metres, same size as color_image).
            intrinsics: Camera intrinsics for the observation.
            target_object: Free-text object description from the VLM planner
                (``NextBestAction.target_object``), e.g. "red cup".

        Returns:
            The top-ranked grasp pose (position + rotation), in the robot frame.
        """
        scene_dir = self.scratch_dir / "current_scene"
        scene_dir.mkdir(parents=True, exist_ok=True)

        rgb_path = scene_dir / "rgb.png"
        depth_path = scene_dir / "depth.npy"
        intrinsics_path = scene_dir / "intrinsics.json"

        color_image.save(rgb_path)
        np.save(depth_path, np.array(depth_image, dtype=np.float32))
        intrinsics_path.write_text(
            json.dumps({"fx": intrinsics.fx, "fy": intrinsics.fy, "cx": intrinsics.ppx, "cy": intrinsics.ppy})
        )

        mask_path = self._run_groundedsam(rgb_path, target_object, scene_dir)
        pcd_path = self._run_mask_to_pointcloud(depth_path, intrinsics_path, mask_path, scene_dir)
        grasp_camera_frame = self._run_graspgen(pcd_path)

        return self.camera_to_robot * grasp_camera_frame

    def _run_groundedsam(self, rgb_path: Path, target_object: str, out_dir: Path) -> Path:
        """Text-prompted segmentation via Grounded-SAM-2. Returns the path to mask.npy."""
        prompt = f"{target_object.lower().strip().rstrip('.')}."
        detections_dir = out_dir / "detections"

        cmd = [
            str(Path(self.config.groundedsam_python).expanduser()),
            str(self.calibration_dir / "detect_object.py"),
            "--rgb", str(rgb_path),
            "--prompt", prompt,
            "--out-dir", str(detections_dir),
        ]
        logger.info("Running Grounded-SAM-2: %s", " ".join(cmd))
        subprocess.run(cmd, check=True)

        return detections_dir / "mask.npy"

    def _run_mask_to_pointcloud(self, depth_path: Path, intrinsics_path: Path, mask_path: Path, out_dir: Path) -> Path:
        """Mask + depth + intrinsics -> object point cloud (pure numpy, runs in this venv)."""
        pcd_path = out_dir / "object_pcd.npy"

        cmd = [
            "python3",
            str(self.calibration_dir / "mask_to_pointcloud.py"),
            "--depth", str(depth_path),
            "--intrinsics", str(intrinsics_path),
            "--mask", str(mask_path),
            "--out", str(pcd_path),
        ]
        logger.info("Running mask_to_pointcloud: %s", " ".join(cmd))
        subprocess.run(cmd, check=True)

        return pcd_path

    def _run_graspgen(self, pcd_path: Path) -> RigidTransform:
        """Rank grasps for the object point cloud, return the top one (camera frame).

        TODO (unverified against the actual GraspGen client-server API):
        run `~/GraspGen/.venv/bin/python ~/GraspGen/client-server/graspgen_client.py --help`
        to confirm (a) how to make it write its ranked grasps to a file
        instead of only visualizing, and (b) the exact schema (translation +
        quaternion? 4x4 matrix? per grasp confidence score?). Then replace the
        body below -- the subprocess call and file layout are correct, only
        the output flag/parsing needs filling in.
        """
        grasps_path = pcd_path.parent / "grasps.json"

        cmd = [
            str(Path(self.config.graspgen_python).expanduser()),
            str(Path(self.config.graspgen_client_script).expanduser()),
            "--pcd_file", str(pcd_path),
            # "--out", str(grasps_path),  # <-- confirm this flag name via --help
        ]
        logger.info("Running GraspGen: %s", " ".join(cmd))
        subprocess.run(cmd, check=True)

        if not grasps_path.is_file():
            raise NotImplementedError(
                "GraspGen output parsing is not wired up yet. Run graspgen_client.py "
                "--help to find its output flag, point it at "
                f"{grasps_path}, and implement the parsing here."
            )

        grasps = json.loads(grasps_path.read_text())
        best = max(grasps, key=lambda g: g["score"])
        position = np.array(best["translation"], dtype=np.float64)
        rotation = Rotation.from_quat(best["rotation"]).as_matrix()
        return RigidTransform(position=position, rotation=rotation)







"""VLM-based action planning with short-term and long-term memory integration."""

import logging
from enum import Enum
from typing import Any, Dict, List, Optional, Tuple

from PIL import Image
from pydantic import BaseModel, Field

from pragmabot.utils import encode_pil_image_to_base64
from pragmabot.vlm_client import VLMClient
from pragmabot.conversation_builder import ConversationBuilder

logger = logging.getLogger(__name__)


# --- Prompt Templates ---

PLANNER_SYSTEM_PROMPT = """
You are a helpful assistant for a legged robot equipped with a single arm and a two-finger gripper. You specialize in task planning, and you always suggest the plan that is most likely to fulfill the task. You learn and adapt from previous experience in the long-term memory and also the current short-term memory (especially the action failures). Always apply chain-of-thought reasoning by thinking step by step before making a final decision.
"""

PLANNER_TASK_PROMPT = """
The robot received this instruction from the user: {task}
"""

PLANNER_INSTRUCTION_PROMPT = """
Given the current scene and task, choose the next best action.

HARD CONSTRAINTS TO APPLY:
  - PUSH only works on objects directly on the table (nothing can be between the object and the table). PICK and PLACE would also work.
  - If the objective is to put an object on top of another object, you must use PICK and PLACE. PUSH would not work. Similarly, you cannot PUSH an object off another object because it is not directly on the table.
  - When two objects are next to each other, it's infeasible to directly PUSH the object that is behind the other one.
  - If the robot is holding something, it must PLACE that object before attempting to grasp another.
  - When the target object is tiny or flat, which is hard to grasp, you cannot use PICK.
  - In the case of action failure, never repeat the same action immediately without first rearranging the scene yourself. This is because the same failure is mostly likely to occur, whether the scene has been reset or not. Instead, consider whether other actions could be taken.
  - Never give a suggestion on how to better execute a failed action—the robot cannot understand or adapt to such advice.
  
GENERAL RULES:
  - Could interacting with other objects help? You are allowed to interact with any other objects on the table.
  - Any object on the table can be used as a tool (all are clean and safe to handle). Be creative! But remember to always PICK up the tool first before using it.
  - In the reasoning process, first propose a promising action, then check whether it violates any constraints, one by one. If it does, discard it and propose another. Repeat this until you find a promising action that satisfies all constraints. 

ACTION PARAMETERS:
  - For the pick up, if the object needs to be grasped at a specific section, you must specify that as well.
  - For the place action, you need to specify which object to place the target object on (not next to). If the object needs to be placed at a specific section, you must specify that as well.
  - For the push action, you need to specify the direction to push (left or right). When both push directions work, prefer pushing left if the object is on the left to the gripper; right if on the right.

Output your final decision in the specified structured format. A human operator may have reset the scene to its initial state after the failure.
"""

STM_PROMPT = """
Here is the action history for the current task so far for reference:

```json
{stm_str}
```

This contains the actions proposed and executed, and the feedback after the action execution. Learn from the experience (including past and active logs) and adapt if needed.
"""

LTM_PROMPT = """
Here are the past relevant experiences from long-term memory.

```json
{ltm_str}
```

They illustrate how the robot successfully planned action sequences to complete tasks, sometimes after initial failures. They reflect the robot’s capabilities and limitations. Always first carefully reason how the current scenario is similar to these past ones, including how the target object is similar. Identify any lessons learned that could apply to the current task. This is part of the chain-of-thought reasoning and helps you avoid repeating past failures.
"""


class RobotSkill(str, Enum):
    """Available robot manipulation skills."""

    PUSH = "push"
    PICK = "pick"
    PLACE = "place"


class PushDirection(str, Enum):
    """Allowed push directions for push actions."""

    LEFT = "left"
    RIGHT = "right"


class NextBestAction(BaseModel):
    """Structured VLM response for the next best robot action to take."""

    scene_description: str = Field(
        ...,
        description="An accurate and concise description of the object's surroundings, especially its spatial relationships with nearby objects. Directions should always be described with respect to the camera, not relative to the object.",
    )
    applicable_knowledge: Optional[str] = Field(
        None,
        description="If past experiences from long-term memory are provided, first identify all similar scenarios, and then check whether there is applicable knowledge to avoid any similar failures. Only output this when long-term memory is provided. ",
    )
    chain_of_thought_reasoning: str = Field(
        ...,
        description="Carefully think about the outcome and the feasibility of the actions. Think step by step and check the constraints one by one. Pay attention to the spatial relationship. Choose the best immediate action that is most likely to accomplish the task.",
    )
    chosen_action: str = Field(
        ...,
        description="The action chosen to be executed (only a single action). This needs to a clear text description of the next immediate action, which a VLM success detector will use to determine whether this action was successful or not.",
    )
    chosen_skill: RobotSkill = Field(..., description="The skill to use for the action, chosen from a predefined set.")
    target_object: str = Field(
        ...,
        description="The object the robot should interact with. This is either the object to push, pick up, or object we should place somewhere.",
    )
    should_grasp_at_specific_section: Optional[bool] = Field(
        None,
        description="Whether the robot should grasp the object at a specific section. Required for pick actions and shall not be provided for other actions.",
    )
    placement_object: Optional[str] = Field(
        None,
        description="The object on which the target object should be placed (not next to). Required for place actions and should not be provided for other actions.",
    )
    should_place_at_specific_section: Optional[bool] = Field(
        None,
        description="Whether the robot should place the object at a specific section to ensure stability and proper placement. Required for place actions and shall not be provided for other actions.",
    )
    push_direction: Optional[PushDirection] = Field(
        None,
        description="The direction to push the object, chosen from a predefined set. Prefer pushing left if object is on left side of image; right if on right. Required for push actions and shall not be provided for other actions.",
    )


class VLMTaskPlanner:
    """Plan the next best robot action using a VLM with memory-augmented reasoning."""

    def __init__(
        self,
        vlm_client: VLMClient,
        conversation_log: Optional[List[Dict[str, Any]]] = None,
    ) -> None:
        """Initialize the planner with a VLM client and shared conversation log.

        Args:
            vlm_client: Client for querying the VLM API.
            conversation_log: Optional shared mutable list for the Gradio UI log.
        """
        self.vlm_client = vlm_client
        self.conversation_log = conversation_log

    def plan_action(
        self,
        task: str,
        color_image: Image.Image,
        stm: List[str],
        ltm: List[str],
    ) -> Tuple[NextBestAction, float, int]:
        """Plan the next best action given the current scene and task.

        Constructs a VLM prompt with the task, camera observation, short-term
        memory, and long-term memory, then queries the VLM for the next best
        action.

        Args:
            task: The user-provided task instruction.
            color_image: Current RGB camera observation as a PIL Image.
            stm: List of short-term memory entries (action/feedback strings).
            ltm: List of long-term memory entries to include in the prompt.

        Returns:
            A tuple of (action, response_time, prompt_tokens)
            where action is the parsed NextBestAction, response_time is in seconds,
            and prompt_tokens is the number of input tokens used.

        Raises:
            ValueError: If the VLM fails to parse a structured response.
        """

        logger.info("VLM task planner running...")

        builder = ConversationBuilder(self.conversation_log)
        builder.log_user_message("## VLM task planner running...")

        # VLM system prompt
        builder.add_system_message(PLANNER_SYSTEM_PROMPT)

        # VLM instruction prompt
        vlm_task_prompt = PLANNER_TASK_PROMPT.format(task=task)
        builder.add_user_text(vlm_task_prompt)

        color_image_base64 = encode_pil_image_to_base64(color_image)
        builder.add_user_image("Here's the robot's current observation.", color_image_base64)

        # Add short-term memory
        if len(stm) > 0:
            stm_str = "\n".join(stm)
            vlm_stm_prompt = STM_PROMPT.format(stm_str=stm_str)
            builder.add_user_text(vlm_stm_prompt, log_heading="")

        builder.add_user_text(PLANNER_INSTRUCTION_PROMPT, log_heading="")

        # Add long-term memory
        if len(ltm) > 0:
            ltm_str = "\n".join(ltm)
            vlm_ltm_prompt = LTM_PROMPT.format(ltm_str=ltm_str)
            builder.add_user_text(vlm_ltm_prompt, log_heading="")

        # Query VLM API
        action_obj, response_time, prompt_tokens = self.vlm_client.query_structured(builder, NextBestAction)
        return action_obj, response_time, prompt_tokens





"""HTTP client for the MoveIt execution server running inside the ROS2 docker container.

This node (pragmabot_node.py) runs under ROS1. MoveIt and the arm run under
ROS2 in a separate docker container. Rather than bridging ROS1<->ROS2 TF and
services (painful, and unnecessary for a handful of request/response commands),
a small HTTP server inside the ROS2 container wraps MoveIt and accepts plain
JSON pose requests. See franka_ros2/pragmabot_moveit_server/moveit_server.py for that
server's counterpart -- the two files share the JSON schema documented below.

Poses sent from here are for the gripper TCP (``fr3_hand_tcp``) in the robot
base frame (``fr3_link0``): exactly what
``GraspExecutionPipeline.get_grasp_pose_robot_frame`` returns once the
camera->robot calibration has been applied.

Wire format, per endpoint:
    POST /execute_grasp {"position": [x,y,z], "orientation_xyzw": [x,y,z,w],
                         "approach_distance": m, "retreat_distance": m,
                         "gripper_width": m, "grasp_force": N, ...}
    POST /place         same shape as /execute_grasp
    POST /move_to_pose  pose (+ optional "cartesian": true)
    POST /gripper       {"action": "open"|"close"|"grasp"|"homing", "width": m, ...}
    POST /go_home       {}
    POST /stop          {}
    GET  /health        -> {"ok": true, "move_group_available": true, ...}
Every response: {"success": bool, "message": str, "steps": [...]}
"""

from __future__ import annotations

import logging
from typing import Any, Optional

import numpy as np
import requests
from omegaconf import DictConfig, OmegaConf
from scipy.spatial.transform import Rotation

from pragmabot.geometry import RigidTransform

logger = logging.getLogger(__name__)

DEFAULT_SERVER_URL = "http://localhost:9000"
# Generous: one request covers approach + grasp + lift on a real arm.
DEFAULT_TIMEOUT_S = 180.0


class MoveItExecutionError(RuntimeError):
    """Raised when the execution server cannot be reached or returns a malformed reply.

    A *motion* that fails is not an error here -- that comes back as
    ``success: False`` and is returned as ``False`` to the caller, so the VLM
    success detector can reason about it like any other failed attempt.
    """


class MoveItClient:
    """Sends grasp/place poses (robot frame) to the MoveIt execution server.

    The client is deliberately thin: it does no geometry beyond the rotation
    matrix -> quaternion conversion, and no retry logic beyond an optional
    startup health check. All motion policy (approach direction, stand-off
    distances, gripper force) lives on the server, next to MoveIt.
    """

    def __init__(self, config: DictConfig, timeout_s: float = DEFAULT_TIMEOUT_S) -> None:
        """Configure the client from ``config.execution``.

        Args:
            config: Full pragmabot config; reads ``execution.moveit_server_url``
                and the optional ``execution.grasp.*`` motion defaults.
            timeout_s: Per-request HTTP timeout in seconds.
        """
        self.base_url = str(OmegaConf.select(config, "execution.moveit_server_url") or DEFAULT_SERVER_URL).rstrip("/")
        self.timeout_s = timeout_s

        # Optional per-deployment motion defaults, forwarded verbatim to the
        # server, which applies its own defaults for anything left unset.
        self.approach_distance = OmegaConf.select(config, "execution.grasp.approach_distance")
        self.retreat_distance = OmegaConf.select(config, "execution.grasp.retreat_distance")
        self.gripper_width = OmegaConf.select(config, "execution.grasp.gripper_width")
        self.grasp_force = OmegaConf.select(config, "execution.grasp.grasp_force")
        self.velocity_scaling = OmegaConf.select(config, "execution.grasp.velocity_scaling")

        self.session = requests.Session()

    # --- public skills --------------------------------------------------------

    def execute_grasp(self, pose_robot_frame: RigidTransform, **overrides: Any) -> bool:
        """Pick at ``pose_robot_frame``: approach, close the gripper, lift.

        Args:
            pose_robot_frame: Grasp pose already transformed into the robot base
                frame (the output of the GraspGen -> calibration chain).
            **overrides: Per-call motion overrides (``approach_distance``,
                ``retreat_distance``, ``gripper_width``, ``grasp_force``,
                ``velocity_scaling``) that win over the config defaults.

        Returns:
            True if the server reports the whole pick sequence succeeded,
            False if any stage failed (planning, execution, or an empty grasp).

        Raises:
            MoveItExecutionError: If the server is unreachable or replies with
                something that is not a valid result object.
        """
        payload = self._pose_payload(pose_robot_frame)
        payload.update(self._motion_defaults())
        payload.update(overrides)
        return self._post("/execute_grasp", payload, "grasp")

    def place(self, pose_robot_frame: RigidTransform, **overrides: Any) -> bool:
        """Place the held object at ``pose_robot_frame``: lower, open, retreat.

        Args:
            pose_robot_frame: Target placement pose for the TCP, robot frame.
            **overrides: Same motion overrides as :meth:`execute_grasp`.

        Returns:
            True if the server reports the place sequence succeeded.
        """
        payload = self._pose_payload(pose_robot_frame)
        payload.update(self._motion_defaults())
        payload.update(overrides)
        return self._post("/place", payload, "place")

    def move_to_pose(self, pose_robot_frame: RigidTransform, cartesian: bool = False, **overrides: Any) -> bool:
        """Move the TCP to a pose without touching the gripper.

        Args:
            pose_robot_frame: Target TCP pose, robot frame.
            cartesian: If True, ask for a straight-line motion instead of a
                free-space plan (use when carrying an object).
            **overrides: Motion overrides, as in :meth:`execute_grasp`.

        Returns:
            True if the motion succeeded.
        """
        payload = self._pose_payload(pose_robot_frame)
        payload["cartesian"] = cartesian
        payload.update(overrides)
        return self._post("/move_to_pose", payload, "move")

    def open_gripper(self, width: Optional[float] = None) -> bool:
        """Open the fingers (default: fully open)."""
        payload: dict[str, Any] = {"action": "open"}
        if width is not None:
            payload["width"] = float(width)
        return self._post("/gripper", payload, "open gripper")

    def close_gripper(self, width: float = 0.0, force: Optional[float] = None) -> bool:
        """Close the fingers onto an object of roughly ``width`` metres.

        Args:
            width: Expected object width in metres; 0.0 closes fully.
            force: Grasping force in newtons; None uses the server default.

        Returns:
            True if the gripper reports it reached the commanded width -- which
            also means it actually has something between the fingers.
        """
        payload: dict[str, Any] = {"action": "grasp", "width": float(width)}
        if force is not None:
            payload["force"] = float(force)
        return self._post("/gripper", payload, "close gripper")

    def home_gripper(self) -> bool:
        """Recalibrate the hand's maximum width (run once after power-up)."""
        return self._post("/gripper", {"action": "homing"}, "home gripper")

    def go_home(self) -> bool:
        """Move the arm back to its 'ready' configuration, e.g. to clear the camera view."""
        return self._post("/go_home", {}, "go home")

    def stop(self) -> bool:
        """Cancel whatever motion is currently executing."""
        return self._post("/stop", {}, "stop")

    # --- health ---------------------------------------------------------------

    def health(self) -> dict[str, Any]:
        """Query the server's status.

        Returns:
            The server's health dict (planning group, frames, whether
            ``move_group`` is actually up, whether it's in dry-run mode).

        Raises:
            MoveItExecutionError: If the server cannot be reached.
        """
        try:
            response = self.session.get(f"{self.base_url}/health", timeout=10.0)
            response.raise_for_status()
            return response.json()
        except (requests.RequestException, ValueError) as exc:
            raise MoveItExecutionError(self._unreachable_message(exc)) from exc

    def check_connection(self, raise_on_failure: bool = False) -> bool:
        """Log (and optionally raise on) whether the execution server is usable.

        Worth calling at node startup: it turns "the robot silently never moves"
        into one clear message before the first task is even planned.

        Args:
            raise_on_failure: Raise instead of returning False.

        Returns:
            True if the server answered and reports ``move_group`` available.
        """
        try:
            status = self.health()
        except MoveItExecutionError as exc:
            if raise_on_failure:
                raise
            logger.error("%s", exc)
            return False

        if status.get("dry_run"):
            logger.warning("MoveIt server is in DRY-RUN mode: requests are accepted but the arm will not move.")
        if not status.get("move_group_available", False):
            message = (
                f"MoveIt server at {self.base_url} is up, but move_group is not. Start "
                "`ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=...` in the container."
            )
            if raise_on_failure:
                raise MoveItExecutionError(message)
            logger.error("%s", message)
            return False

        logger.info(
            "MoveIt server ready at %s (group=%s, base=%s, ee=%s)",
            self.base_url,
            status.get("planning_group"),
            status.get("base_frame"),
            status.get("end_effector"),
        )
        return True

    # --- internals ------------------------------------------------------------

    @staticmethod
    def _pose_payload(pose_robot_frame: RigidTransform) -> dict[str, Any]:
        """Serialize a RigidTransform into the server's position/quaternion JSON.

        Raises:
            MoveItExecutionError: If the pose is not a valid rigid transform
                (non-finite values, or a rotation that isn't orthonormal) --
                worth catching here rather than sending garbage to a real arm.
        """
        position = np.asarray(pose_robot_frame.position, dtype=np.float64).reshape(3)
        rotation = np.asarray(pose_robot_frame.rotation, dtype=np.float64).reshape(3, 3)

        if not np.all(np.isfinite(position)) or not np.all(np.isfinite(rotation)):
            raise MoveItExecutionError(f"Grasp pose contains non-finite values: position={position!r}")
        if not np.allclose(rotation @ rotation.T, np.eye(3), atol=1e-3):
            raise MoveItExecutionError(
                "Grasp pose rotation is not a valid rotation matrix (R @ R.T != I); "
                "check the camera->robot transform and the GraspGen output convention."
            )

        # scipy returns (x, y, z, w), which is what geometry_msgs/Quaternion wants.
        quaternion = Rotation.from_matrix(rotation).as_quat()
        return {"position": position.tolist(), "orientation_xyzw": quaternion.tolist()}

    def _motion_defaults(self) -> dict[str, Any]:
        """Config-provided motion parameters, omitting anything left unset."""
        candidates = {
            "approach_distance": self.approach_distance,
            "retreat_distance": self.retreat_distance,
            "gripper_width": self.gripper_width,
            "grasp_force": self.grasp_force,
            "velocity_scaling": self.velocity_scaling,
        }
        return {key: float(value) for key, value in candidates.items() if value is not None}

    def _post(self, endpoint: str, payload: dict[str, Any], what: str) -> bool:
        """POST one command and interpret the standard result object.

        Args:
            endpoint: Server path, e.g. ``/execute_grasp``.
            payload: JSON body.
            what: Short label used in log messages.

        Returns:
            The server's ``success`` flag.

        Raises:
            MoveItExecutionError: On transport failure or a malformed reply.
        """
        url = f"{self.base_url}{endpoint}"
        logger.info("MoveIt %s request -> %s: %s", what, url, payload)

        try:
            response = self.session.post(url, json=payload, timeout=self.timeout_s)
        except requests.RequestException as exc:
            raise MoveItExecutionError(self._unreachable_message(exc)) from exc

        # 4xx/5xx still carry a JSON result body from this server; only fall back
        # to raise_for_status if the body isn't parseable.
        try:
            result = response.json()
        except ValueError as exc:
            raise MoveItExecutionError(
                f"MoveIt server returned a non-JSON reply (HTTP {response.status_code}): {response.text[:200]!r}"
            ) from exc
        if not isinstance(result, dict) or "success" not in result:
            raise MoveItExecutionError(f"MoveIt server returned an unexpected reply: {result!r}")

        success = bool(result["success"])
        if success:
            for step in result.get("steps", []):
                logger.info("  %s: %s", step.get("step"), step.get("detail") or "ok")
            logger.info("MoveIt %s succeeded.", what)
        else:
            logger.error("MoveIt %s failed: %s", what, result.get("message", "no message"))
        return success

    def _unreachable_message(self, exc: Exception) -> str:
        """Explain a transport failure in terms of the ROS1/ROS2 split."""
        return (
            f"Could not reach the MoveIt execution server at {self.base_url} ({type(exc).__name__}: {exc}). "
            "Check that moveit_server.py is running inside the ros2_humble container and that its port "
            "is published to the host (e.g. `docker run -p 9000:9000 ...`), and that "
            "config.execution.moveit_server_url points at it."
        )





#!/usr/bin/env python3
"""Main ROS node for PragmaBot with Gradio UI."""

import datetime
import json
import logging
import os
import sys

import matplotlib

matplotlib.use("Agg")

import gradio as gr
import rospy
from openai import OpenAI

from pragmabot.grasp_pipeline import GraspExecutionPipeline
from pragmabot.moveit_client import MoveItClient
from pragmabot.scene_observer import SceneObserver
from pragmabot.simple_config import get_config
from pragmabot.utils import get_package_path
from pragmabot.vlm_success_detector import VLMSuccessDetector
from pragmabot.vlm_task_planner import RobotSkill, VLMTaskPlanner
from pragmabot.vlm_client import VLMClient
from pragmabot.vlm_exp_summarizer import VLMExperienceSummarizer
from pragmabot.conversation_builder import ConversationBuilder
from pragmabot.vlm_scene_describer import VLMSceneDescriber
from pragmabot.memory_manager import MemoryManager


logger = logging.getLogger(__name__)


class PragmaBot:
    """PragmaBot node with Gradio UI for task planning."""

    def __init__(self) -> None:
        """Initialize the PragmaBot ROS node, services, and Gradio UI."""
        self.data_folder = get_package_path() / "data"
        self.log_folder = self.data_folder / "logs"

        # Initialize ROS node and TF listener
        rospy.init_node("pragmabot_node")

        # Configure Python logging AFTER rospy.init_node(), because rospy overrides the root logger's handlers/level during initialization.
        logging.basicConfig(
            level=logging.INFO, format="[%(levelname)s] [%(name)s]: %(message)s", stream=sys.stdout, force=True
        )

        # Initialize variables
        self.color_image = None
        self.depth_image = None
        self.intrinsics = None
        self.observation_time = None

        self.instruction = None
        self.initial_scene_description = None
        self.action_taken = None
        self.stm = []
        self.ltm = []
        self.conversation_log = []
        self.display_index = 0
        self.time_step = 0

        # Load configuration from YAML file
        self.config = get_config()
        self.scene_observer = SceneObserver(self.config.topics)

        if not self.config.rosbag_replay:
            self.grasp_pipeline = GraspExecutionPipeline(self.config)
            self.moveit_client = MoveItClient(self.config)

        # If vlm model contains gpt, initialize the client
        if "gpt" in self.config.vlm.vlm_model:
            self.client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
            rospy.loginfo(f"Using {self.config.vlm.vlm_model} with OpenAI")
        else:
            rospy.logfatal("Unsupported VLM model: %s", self.config.vlm.vlm_model)
            sys.exit(1)
        self.vlm_client = VLMClient(self.client, self.config.vlm)

        self.memory_manager = MemoryManager(self.vlm_client, self.conversation_log)
        self.scene_describer = VLMSceneDescriber(self.vlm_client, self.conversation_log)
        self.exp_summarizer = VLMExperienceSummarizer(self.vlm_client, self.conversation_log)
        self.success_detector = VLMSuccessDetector(self.vlm_client, self.conversation_log)
        self.task_planner = VLMTaskPlanner(self.vlm_client, self.conversation_log)

        self.start_gradio_interface()

    def run(self):
        """Block on rospy.spin() until shutdown."""
        rospy.spin()

    def handle_planning_request(self, chatbot):
        """Run one planning step: observe scene, plan action, and execute.

        Captures a camera observation, optionally generates a scene description,
        queries the VLM action planner, and dispatches the resulting action
        to the function call handler and behavior tree.

        Args:
            chatbot: Gradio chatbot component (unused, kept for callback API).
        """
        self.time_step += 1

        (
            self.color_image,
            self.depth_image,
            self.intrinsics,
            self.observation_time,
        ) = self.scene_observer.get_scene_observation()

        if self.config.activate_ltm or self.config.save_to_ltm:
            # Need initial scene description for LTM retrieval, but only want to generate it once at the first step (not every step)
            if self.initial_scene_description is None:
                self.initial_scene_description = self.scene_describer.get_scene_description(
                    self.instruction, self.color_image
                )

        if self.config.activate_ltm:
            # Retrieve long-term memory if activated and not already retrieved for this task
            if len(self.ltm) == 0:
                self.ltm, _, _, _, _ = self.memory_manager.retrieve_relevant_experiences(
                    self.instruction,
                    self.initial_scene_description,
                    self.config.retrieval_top_k,
                    self.config.use_random_retrieval,
                )

        next_action, _, _ = self.task_planner.plan_action(self.instruction, self.color_image, self.stm, self.ltm)
        self.append_to_stm_if_activated("action", next_action)
        self.action_taken = next_action.chosen_action

        if self.config.rosbag_replay:
            # Send a request for success detection if no action execution
            self.handle_evaluation_request(chatbot)
        else:
            self.execute_action(next_action)
            self.handle_evaluation_request(chatbot)

    def execute_action(self, next_action) -> None:
        """Execute the planned action on the real robot.

        Only PICK is wired up so far: segment the target object, generate a
        grasp, cross into the robot frame, and execute it via MoveIt. PLACE
        and PUSH follow a similar chain but aren't implemented yet.

        Args:
            next_action: The VLM's chosen action (NextBestAction).
        """
        if next_action.chosen_skill != RobotSkill.PICK:
            raise NotImplementedError(
                f"Execution for skill '{next_action.chosen_skill}' is not implemented yet. "
                "Only PICK is wired up so far."
            )

        grasp_pose_robot_frame = self.grasp_pipeline.get_grasp_pose_robot_frame(
            self.color_image,
            self.depth_image,
            self.intrinsics,
            next_action.target_object,
        )
        success = self.moveit_client.execute_grasp(grasp_pose_robot_frame)
        if not success:
            rospy.logerr(f"Grasp execution failed for target_object={next_action.target_object!r}")

    def handle_evaluation_request(self, chatbot):
        """Evaluate action success by comparing before/after observations.

        Captures a new observation, sends the before/after images to the VLM
        success detector, and handles task completion or re-planning.

        Args:
            chatbot: Gradio chatbot component (unused, kept for callback API).
        """

        color_image_before_action = self.color_image

        (
            self.color_image,
            self.depth_image,
            self.intrinsics,
            self.observation_time,
        ) = self.scene_observer.get_scene_observation()

        success_evaluation = self.success_detector.perform_success_detection(
            self.instruction,
            self.action_taken,
            color_image_before_action,
            self.color_image,
        )
        self.append_to_stm_if_activated("evaluation", success_evaluation)

        if success_evaluation.is_task_completed:
            if self.config.save_to_ltm:
                self.handle_experience_summarization(chatbot)
        else:

            if not success_evaluation.is_action_successful:
                info_msg = "The human operator might have reset the scene after the action failure."
                self.append_to_stm_if_activated("additional_info", info_msg)

            # if using rosbag, call planning directly
            if self.config.rosbag_replay:
                self.handle_planning_request(chatbot)
            else:
                raise NotImplementedError(
                    "Implement your own logic here to decide when to call the planning function again for re-planning, based on the success evaluation results."
                )

    def handle_experience_summarization(self, chatbot):
        """Summarize the current short-term memory into a long-term experience entry.

        Args:
            chatbot: Gradio chatbot component (unused, kept for callback API).
        """
        summarized_experience = self.exp_summarizer.summarize_stm_to_ltm(
            self.instruction, self.initial_scene_description, self.stm
        )
        self.memory_manager.save_experience(self.instruction, self.initial_scene_description, summarized_experience)

    def save_conversation_log(self):
        """Save the conversation log to a timestamped JSON file.

        The file is saved to the logs folder with a name derived from the
        instruction and current timestamp. Only called on exit.

        Returns:
            The file path if saved successfully, or None on failure.
        """
        if self.instruction is None:
            rospy.logwarn("No valid instruction — skipping log save.")
            return

        # Sanitize instruction for filename
        safe_instruction = "".join(c if c.isalnum() else "_" for c in self.instruction[:50]).strip().lower()
        if safe_instruction.endswith("_"):
            safe_instruction = safe_instruction[:-1]
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{timestamp}_{safe_instruction}.json"
        filepath = self.log_folder / filename

        try:
            with open(filepath, "w", encoding="utf-8") as f:
                json.dump(
                    self.conversation_log,
                    f,
                    indent=4,
                    ensure_ascii=False,
                )
            rospy.loginfo(f"Conversation log saved to: {filepath}")
            return filepath
        except Exception as e:
            rospy.logerr(f"Failed to save conversation log: {e}")
            return None

    def load_conversation_log(self, file_path, chatbot):
        """Load a conversation log from a JSON file and update the chatbot.

        Args:
            file_path: Path to the JSON log file to load.
            chatbot: Gradio chatbot component (unused, kept for callback API).

        Returns:
            The updated conversation log list.
        """
        if not file_path or not os.path.exists(file_path):
            error_msg = f"Error: File not found - {file_path}"
            rospy.logerr(error_msg)
            self.conversation_log.append({"role": "user", "content": error_msg})
            return self.conversation_log

        try:
            with open(file_path, "r", encoding="utf-8") as f:
                loaded_log = json.load(f)
                if isinstance(loaded_log, list):
                    self.conversation_log = loaded_log
                    rospy.loginfo(f"Loaded conversation log from: {file_path}")
                else:
                    raise ValueError("Invalid log format — expected a list of messages.")
        except Exception as e:
            error_msg = f"Failed to load conversation log: {e}"
            rospy.logerr(error_msg)
            self.conversation_log.append({"role": "user", "content": error_msg})

        return self.conversation_log

    def append_to_stm_if_activated(self, key: str, data: object) -> None:
        """Build a JSON entry and append it to the short-term memory list.

        Args:
            key: Dictionary key for the data (e.g., ``'action'``, ``'evaluation'``).
            data: Payload to store. Pydantic models are auto-serialized via
                ``model_dump``; dicts and strings are stored as-is.
        """
        if self.config.activate_stm is False:
            return

        # Automatically extract data from Pydantic models
        if hasattr(data, "model_dump"):
            processed_data = data.model_dump(exclude_none=True, exclude_unset=True)
        else:
            processed_data = data

        entry_with_timestep = {
            "time_step": self.time_step,
            key: processed_data,
        }

        entry_json = json.dumps(entry_with_timestep, indent=2)
        self.stm.append(f"{entry_json}\n")

        builder = ConversationBuilder(self.conversation_log)
        builder.log_assistant_message(entry_json, is_json=True)

    def start_gradio_interface(self):
        """Build and launch the Gradio web UI using a Right Sidebar Layout."""

        def get_new_task(user_instruction, chatbot):
            """Handle a new user instruction submission."""
            init_task(chatbot)
            self.conversation_log.append({"role": "user", "content": f"# Instruction: {user_instruction}"})
            self.instruction = user_instruction
            # The gui is updated immediately after this function returns. So the display_index should be updated here.
            self.display_index = len(self.conversation_log)
            return "", self.conversation_log

        def init_task(chatbot):
            """Reset all task-related state for a new planning session."""
            self.color_image = None
            self.depth_image = None
            self.intrinsics = None
            self.observation_time = None

            self.instruction = None
            self.initial_scene_description = None
            self.action_taken = None
            self.stm.clear()
            self.ltm.clear()
            self.conversation_log.clear()
            self.display_index = 0
            self.time_step = 0
            return

        def update_chat(chatbot):
            """Yield new conversation log entries to the Gradio chatbot UI."""
            total_messages = len(self.conversation_log)

            if total_messages == 0:
                # If the chat was cleared, just reset the display (this should already be reset by the clear button)
                self.display_index = 0
                yield []
            elif total_messages > self.display_index:
                new_messages = self.conversation_log[self.display_index :]
                self.display_index = total_messages
                yield chatbot + new_messages
            else:
                yield chatbot

        custom_css = """
        #chatbot .bubble {
            max-width: 80% !important;
            padding-left: 12px !important;
            padding-right: 12px !important;
        }

        #chatbot .bubble.left {
            margin-right: auto !important;
        }

        #chatbot .bubble.right {
            margin-left: auto !important;
        }

        #chatbot {
            height: calc(100vh - 100px) !important; 
            min-height: 600px;
        }
        
        .right-sidebar {
            border-left: 1px solid #e5e7eb;
            padding-left: 20px !important;
        }
        """

        with gr.Blocks(css=custom_css, title="PragmaBot") as demo:
            timer = gr.Timer(value=2)  # updates every 2 seconds

            with gr.Row():
                with gr.Column(scale=4):
                    chatbot = gr.Chatbot(
                        type="messages",
                        elem_id="chatbot",
                        label="VLM Conversation Log",
                    )

                with gr.Column(scale=1, elem_classes="right-sidebar"):
                    gr.Markdown("### 🤖 Instruction")
                    instruction_msg = gr.Textbox(
                        show_label=False,
                        placeholder="Type your instruction here. Press enter to submit...",
                        elem_id="user-instruction",
                    )

                    with gr.Row():
                        clear_button = gr.Button("🗑️ Clear", elem_id="clear")
                        update_button = gr.Button("🔄 Update", elem_id="update")

                    gr.HTML("<hr style='margin-top: 30px; margin-bottom: 30px;'>")

                    # 2. Secondary Actions remain at the bottom
                    gr.Markdown("### 📂 Log Management")
                    load_log_button = gr.UploadButton(
                        "Click to Select JSON Log", file_types=[".json"], elem_id="load-log"
                    )

            gr.Markdown(f"> 💡 Default logs location: `{self.log_folder}`")

            # --- Event Bindings ---
            timer.tick(update_chat, inputs=chatbot, outputs=chatbot)

            instruction_msg.submit(
                get_new_task, [instruction_msg, chatbot], [instruction_msg, chatbot], queue=False
            ).then(self.handle_planning_request, chatbot, None)

            clear_button.click(lambda: init_task(chatbot), None, chatbot, queue=False)
            update_button.click(update_chat, chatbot, chatbot, queue=False)
            load_log_button.upload(
                self.load_conversation_log, inputs=[load_log_button, chatbot], outputs=chatbot, queue=False
            )

        demo.launch(share=self.config.gradio_share, inline=False, prevent_thread_lock=True, server_name="0.0.0.0")


if __name__ == "__main__":
    pragmabot = PragmaBot()

    try:
        pragmabot.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Interrupt received. Shutting down...")
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")
    finally:
        rospy.loginfo("Saving conversation log before shutdown...")
        pragmabot.save_conversation_log()
        rospy.loginfo("Cleanup completed. Node terminated.")
        
        
        
        
        
        
        
        
        
        
----------------------------------------------------------------------------------------------------

IDEA CHANGES:
Good — now I have the real picture, straight from your own commit history and RUNBOOK.md. Here's where things actually stand.

What's already built (as of your last session, 2026-08-24 night)

Your pipeline is fully wired end-to-end and has already moved the real arm. The flow is:

Gradio UI → VLMTaskPlanner (Claude) → chosen_skill="pick", target_object="red cup"
        → panda_skill_executor.py sends a ROS2 action goal (ExecuteSkill)
        → bridge_node.py (the ROS2 action server) receives it
        → live_perception.py: GroundedSAM (ZMQ :5557) → mask → point cloud
        → GraspGen (ZMQ :5556) → ranked 6-DoF grasps (camera frame)
        → grasp_transform.py: fail-closed tilt gate picks the best perpendicular grasp,
          transforms it into fr3_link0 via live TF (camera→robot calibration)
        → MoveIt2: Cartesian standoff → approach → close gripper → lift

Everything here is real ROS2 (rclpy), talking to franka_fr3_moveit_config/MoveIt2 and franka_gripper directly — no HTTP bridge, no ROS1, nothing from Armin's design is needed or used. Forget that transcript entirely; it was based on a stale assumption about your architecture.





Terminal 1 (inside the container, or wherever MoveIt already runs):
ros2 launch easy_handeye2 publish.launch.py name:=fr3_zed_right
Wait for it to print the "Loading the calibration..." line and then go quiet (no crash, no more output) — that's it running correctly. Leave this terminal alone, don't close it, don't Ctrl-C it, for as long as you're using the robot.

Terminal 2 — is your ZED camera actually running right now? If you haven't launched it yet this session:
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2
Check: ros2 node list | grep zed_node should show it. This also needs to stay running. If it's already running from earlier, skip this.

Terminal 3 — only now, check the transform:
ros2 run tf2_ros tf2_echo fr3_link0 zed_left_camera_frame_optical
It should print real translation/rotation numbers, repeating, with no "does not exist" errors.

If it still fails after both Terminal 1 and Terminal 2 are confirmed alive and untouched, tell me exactly what Terminal 3 prints and I'll dig further — but in most cases 

Terminal 3 — only now, check the transform:
ros2 run tf2_ros tf2_echo fr3_link0 zed_left_camera_frame_optical
It should print real translation/rotation numbers, repeating, with no "does not exist" errors.

If it still fails after both Terminal 1 and Terminal 2 are confirmed alive and untouched, tell me exactly what Terminal 3 prints and I'll dig further — but in most cases this is exactly the "three things all have to be running at once" pattern your own RUNBOOK.md warns about repeatedly (killing perception's terminal by accident caused a near-identical failure last session).

Yes — that's confirmed calibrated. Two proofs: the translation [0.912, -0.064, 0.514] matches your calibration document almost exactly, and /zed/zed_node shows the camera is actually running. Good, that blocker is resolved.

One correction to what I told you earlier: I flagged the fr3_hand vs fr3_hand_tcp mismatch as a risk to check later. I just checked grasp_transform.py's own header comment — it explicitly says GraspGen's grasp poses are defined with their origin at "the gripper base/root link, not the fingertip/TCP." That matches bridge_node.py's default eef_link="fr3_hand" exactly by design. So that's not a bug — leave it as-is, don't change it. Good that we verified instead of guessing.       
        
        



        


1. Cartesian vs. joint-space planning — you're mostly already there

Your supervisor's point is largely already implemented in bridge_node.py. Here's the actual split:

┌───────────────────────────────────────────┬──────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┐
│              Motion segment               │                                                       Planner used                                                       │
├───────────────────────────────────────────┼──────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┤
│ Pick: standoff approach                   │ Cartesian first (/compute_cartesian_path), OMPL only as fallback if the straight line has no IK (bridge_node.py:941-948) │
├───────────────────────────────────────────┼──────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┤
│ Pick: descent into grasp                  │ Cartesian (:1007)                                                                                                        │
├───────────────────────────────────────────┼──────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┤
│ Pick: lift / retreat                      │ Cartesian (:1076)                                                                                                        │
├───────────────────────────────────────────┼──────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┤
│ Place: descent, lift                      │ Cartesian (:1295, :1313)                                                                                                 │
├───────────────────────────────────────────┼──────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┤
│ Push: descent, lateral push, lift         │ Cartesian (:1529-1562)                                                                                                   │
├───────────────────────────────────────────┼─────────────────────────────────────────────────────────────────────────────────────────────┤
│ Place: initial free move above drop point │ OMPL _move_to_pose / MoveGroup (:1276)                                                                                   │
├───────────────────────────────────────────┼──────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┤  │ Push: initial free move to standoff       │ OMPL (:1524)                                                                                │
├───────────────────────────────────────────┼──────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┤
│ Pick standoff, only if Cartesian fails    │ OMPL fallback                                                                                                            │  └───────────────────────────────────────────┴─────────────────────────────────────────────────────────────────────────────────────────────┘

So every fine motion (the parts where smoothness matters — approach, descent, retreat) is already Cartesian. The only remaining OMPL calls are the big free-space transit moves — going from wherever the arm parked to above the next target. Ttesian: a straight line across the whole workspace frequently crosses a singularity or joint limit (that's exactly what the max_joint_jump_rad and revolute_jump_threshold guards catch), so forcing Cartesian there would make picks fail, not smoother.

If your supervisor wants those transit moves smoother too, the better levers are:
- Lower cartesian_max_step / raise cartesian_max_tries for finer interpolation
- The velocity retiming in _execute_trajectory (grasp_transform.retimejerk from MoveIt's constant-velocity assumption — you can apply it tothe OMPL transit trajectories too (currently only descent/push segments get retimed)
- Switch OMPL to a smoothing-friendly planner (e.g. RRTstar + longer allowed_planning_time, or add TimeOptimalTrajectoryGeneration post-processing) via the MoveGroup goal in _move_to_pose (:2027)

Verdict: you're correct that OMPL is still used, but only for the two whole-arm transit moves, and that's intentional. Everything close to the object is Cartesian.      




-------------------------------------------------------------
IMPLEMENTATION CODE :

container franka:
echo $ROS_DOMAIN_ID
echo $DISPLAY
docker exec -it -e DISPLAY=$DISPLAY franka_ros2_humble bash
source /ros2_ws/install/setup.bash
ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=10.10.10.10

calibration:  ros2 launch easy_handeye2 publish.launch.py name:=fr3_zed_right

in host:
echo $ROS_DOMAIN_ID
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2

we should add:
    export ROS_DOMAIN_ID=7
for every host termianl

        
        
Next: bring up the rest of the pipeline and try a pick

You already have running: ZED camera, MoveIt/gripper (presumably, from earlier in the session), and the calibration publisher. From here it's pragmabot/RUNBOOK.md §2's gate check, then §3 onward. Do these in order, each in its own terminal you don't touch again:

Step A — confirm the gripper is alive (do this before anything else, it's the #1 failure point):
ros2 action info /franka_gripper/homing
Must say Action servers: 1. If it says 0, the gripper node died — restart it (RUNBOOK §2 has the exact command) before continuing.

Step B — Terminal 3: GroundedSAM perception server
source ~/groundedsam/.venv/bin/activate
cd ~/irm2pragmabot/pragmabot
PRAGMABOT_DEBUG_SAVE=1 python3 calibration/perception_server.py #best one
# python3 calibration/perception_server.py
Wait for: Listening on tcp://127.0.0.1:5557

Step C — Terminal 4: GraspGen server
source ~/GraspGen/.venv/bin/activate
python3 ~/GraspGen/client-server/graspgen_server.py     --gripper_config ~/GraspGen/GraspGenModels/checkpoints/graspgen_franka_panda.yml

    
Wait for: GraspGen ZMQ server listening on tcp://0.0.0.0:5556 (~45s first time)


Terminal 5 — GraspGen visualizer (leave running, like the server):
source ~/GraspGen/.venv/bin/activate
python3 ~/GraspGen/client-server/graspgen_client.py \
    --pcd_file /tmp/pragmabot_live_cloud.npy \
    --visualize
    
best candidate: ~/GraspGen/.venv/bin/python3 calibration/visualize_live_grasps.py --max_tilt_deg 30 --max_crossaxis_deg 20
    


Step D — Terminal 5: the bridge
export ROS_DOMAIN_ID=7
source /opt/ros/humble/setup.bash
source ~/pragmabot_bridge_ws/install/setup.bash
export PYTHONPATH=$PYTHONPATH:$HOME/GraspGen
ros2 run pragmabot_bridge bridge_node

ros2 run pragmabot_bridge bridge_node --ros-args   -p calib_correction_x:=-0.05   -p calib_correction_y:=0.0   -p calib_correction_z:=0.0
ros2 run pragmabot_bridge bridge_node --ros-args   -p calib_correction_x:=-0.05   -p calib_correction_y:=-0.0   -p calib_correction_z:=-0.03
Wait for: Serving /pragmabot/execute_skill - waiting for planner goals

Step E — Terminal 6: the VLM planner + Gradio UI
export ROS_DOMAIN_ID=7
source /opt/ros/humble/setup.bash
source ~/pragmabot_bridge_ws/install/setup.bash
export PYTHONPATH=$PYTHONPATH:$HOME/irm2pragmabot/pragmabot/pragmabot/src:$HOME/GraspGen
cd ~/irm2pragmabot/pragmabot
python3 pragmabot/nodes/pragmabot_node.py
Wait for Running on local URL: http://0.0.0.0:7860, open that in a brocup (start a fresh task, not a continued one).



harish@Alonnisos:~/ros2_ws$ source /opt/ros/humble/setup.bash && source ~/pragmabot_bridge_ws/install/setup.bash
harish@Alonnisos:~/ros2_ws$ colcon build --symlink-install --packages-select pragmabot_bridge
        
        
first position:
python3 scripts/home_pose.py --go

release the gripper:
user@Alonnisos:/ros2_ws$ ros2 action send_goal /franka_gripper/move franka_msgs/action/Move "{width: 0.08, speed: 0.05}"

ls -l /dev/shm/ | grep -iE 'fastrtps|fast_dds|sem\.'
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*
ls /dev/shm/ | grep -i fastrtps 

banana and long object
ros2 param set /pragmabot_bridge graspgen_frame_yaw_offset_deg -45.0
        
others
ros2 param set /pragmabot_bridge graspgen_frame_yaw_offset_deg 0.0

ros2 param list /pragmabot_bridge

ros2 param get /pragmabot_bridge graspgen_frame_yaw_offset_deg       
        
        
        
        
        
        
 

