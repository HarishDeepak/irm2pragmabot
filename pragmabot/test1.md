
Fair question. Let me answer it with real numbers rather than excuses:

  Ran 2 shell commands

The pipeline is fast. Measured this session:

┌──────────────────────┬────────────────────────┐
│        Stage         │          Time          │
├──────────────────────┼────────────────────────┤
│ Live RGB-D capture   │ < 0.5 s                │
├──────────────────────┼────────────────────────┤
│ GroundedSAM detect → │ 0.22–1.25 s            │
│  cloud               │                        │
├──────────────────────┼────────────────────────┤
│ GraspGen (100        │ 0.82 s                 │
│ grasps)              │                        │
├──────────────────────┼────────────────────────┤
│ Perception total per │ ~1.5 s                 │
│  pick                │                        │
├──────────────────────┼────────────────────────┤
│ MoveIt plan + arm    │ 10–20 s (physics,      │
│ motion               │ unavoidable)           │
├──────────────────────┼────────────────────────┤
│ VLM planner call     │ 9 s                    │
└──────────────────────┴────────────────────────┘

What ate the session wasn't the pipeline:

1. The disk filling — the single biggest chunk, and unrelated to any of this.
2. Nothing had ever been built. pragmabot_interfaces, franka_msgs and the wired bridge had no build in any reachable workspace. Today was the first successful build of the 1108-line bridge.
3. Two environment bugs that would have blocked you regardless — brotli (every VLM call failed) and setuptools/packaging (every Python build failed).
4. Model servers take ~45 s to load — paid once, and I restarted them three times.
5. I chose to verify instead of assume on the 20° tilt. That one was worth it: had I skipped it, the alternative was finding out by driving the gripper into the table.

From here it's fast. Servers are up (:5557 perception, :5556 GraspGen), MoveIt and ZED are live, _scene_source is wired.

---

B3 — step 1 of 2: start the bridge

Note the command changed — host, not container, and it needs the overlay plus grasp_gen on the path:

export ROS_DOMAIN_ID=7
source /opt/ros/humble/setup.bash
source ~/pragmabot_bridge_ws/install/setup.bash
export PYTHONPATH=$PYTHONPATH:$HOME/GraspGen
ros2 run pragmabot_bridge bridge_node

Expected on startup: one line — pragmabot_bridge started - waiting for /move_action, .... It should then sit idle. No errors, no traceback.

Health check, from another host terminal:

export ROS_DOMAIN_ID=7
ros2 node list | grep pragmabot_bridge
ros2 action list | grep execute_skill
ros2 param get /pragmabot_bridge use_live_perception
ros2 param get /pragmabot_bridge place_offset_xyz

Expect /pragmabot_bridge, /pragmabot/execute_skill, use_live_perception: True, place_offset_xyz: [0.0, 0.2, 0.0].

This still moves nothing. Once it's up and the four checks look right, tell me — then I'll give you the pick goal together with the phase-by-phase description of what should happen, so you can stop it the moment reality diverges


ridge is alive. Here's what you just started, then the next step.

What happened

Upstream PragmaBot ships action execution as NotImplementedError — the VLM plans, but nothing ever moves. bridge_node is the layer that closes that gap, and it's the project's actual engineering contribution.

What came up when you ran it:

ExecuteSkill action server  →  /pragmabot/execute_skill
MoveIt clients              →  /move_action, /execute_trajectory,
                               /compute_cartesian_path
Gripper clients             →  /franka_gripper/{grasp,homing,move}
TF listener                 →  fr3_link0 ← ... ← zed_left_camera_frame_optical
ZMQ clients                 →  :5557 perception,  :5556 GraspGen

Three design points worth being able to defend:

It's an action server, not a service. PandaSkillExecutor on the planner side sends one goal per planner decision. Actions give feedback (perceiving → picking) and cancellation for free; a service gives neither, and a pick takes 20 s.

The callbacks are blocking, so it needs a MultiThreadedExecutor and a ReentrantCallbackGroup. execute_pick() calls spin_until_future_complete internally while waiting on MoveIt. Under the default single-threaded executor that's an instant deadlock — the callback waits on a future only the executor it's blocking could complete.

It runs on the host, not in the container — my change to your plan. The container has no pip (so no zmq/msgpack) and can't see ~/GraspGen. The host had everything except two message packages, which now build at ~/pragmabot_bridge_ws. Since network_mode: host, DDS talks to MoveIt inside the container exactly as before.

use_live_perception: True means the goal's target_object string becomes the GroundedSAM prompt — no stale .npz. That's the correctness fix that stops "pick the cup" from picking whatever was segmented last week.
