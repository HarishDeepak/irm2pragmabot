# Panda Gap Analysis: PragmaBot Reimplementation

## 1. Goal
Reimplement PragmaBot on a Franka Emika Panda using the existing Panda ROS/MoveIt/ZED stack, while preserving the high-level VLM planner, memory, and RAG logic.

## 2. What Can Be Reused As-Is
- VLM planner logic and prompts (except robot description text)
- Success detector logic and prompts (except robot description text)
- Scene describer and experience summarizer
- STM and LTM data structures and RAG retrieval pipeline
- NextBestAction and SuccessEvaluation schemas
- SceneObserver interface (RGB, depth, intrinsics, timestamp)
- Conversation logging and STM appending

## 3. What Must Be Adapted or Implemented for Panda

| Component              | Original (ANYmal)                            | Panda-side replacement                         | Notes                                              | Status      |
|------------------------|----------------------------------------------|-----------------------------------------------|---------------------------------------------------|------------|
| Robot platform         | Legged robot with arm                        | Fixed-base Franka Panda arm                   | No locomotion; tabletop only                      | Not started |
| Motion execution       | ANYmal robot backend                         | MoveIt + Panda controller                     | Use your IRM1 Panda stack                         | Not started |
| Gripper                | Robotiq 2F-140                               | Franka Hand / lab gripper                     | Confirm gripper type and jaw width                | Open       |
| Camera input           | ZED X Mini on arm                            | Existing ZED pipeline                         | Update ROS topics in config.yaml                  | In progress |
| Camera TF chain        | ANYmal-specific frames                       | ZED → panda_link0 transform                   | Needs calibration / static TF                     | Not started |
| IK feasibility         | Pinocchio                                    | MoveIt compute_ik                             | Replace with Panda URDF-based checks              | Not started |
| Grasp generation       | AnyGrasp                                     | Your IRM1 grasp stack                         | Must work from RGB-D + mask                       | Not started |
| Skill execution API    | Original action executor (not released)      | PandaSkillExecutor                            | Implement in new Panda adapter module             | Not started |
| System prompts         | “legged robot with a single arm…”            | “fixed-base Franka Emika Panda robot arm…”    | Update planner + detector system prompts          | Not started |
| SceneObserver topics   | /zedxm/...                                   | /zed/... (from franka_zed_gazebo)             | Topic rename only                                 | Not started |
| Observation pose       | Implicit in original setup                   | Explicit go_to_observation_pose()             | Must be defined for consistent pre/post images    | Not started |

## 4. Required Panda Primitives and Policies

### Core primitives
- `pick(target_object, annotate, color, depth, intrinsics)`
- `place(placement_object, annotate, color, depth, intrinsics)`
- `push(target_object, direction, color, depth, intrinsics)`

### Utility policies
- `go_to_observation_pose()` – move to fixed joint config for RGB-D capture
- `go_to_home()` – safe reset configuration
- `open_gripper()` / `close_gripper()`
- `recover()` – conservative recovery from error states
- `is_holding_object()` – check gripper width / state

## 5. Perception & Annotation Requirements
- RGB image + object name → 2D mask (SAM2 / GroundedSAM)
- Depth + intrinsics → point cloud in camera frame (Open3D)
- Camera TF → map points to panda_link0 frame (tf2)
- Grasp generator → candidate grasp poses from point cloud + mask
- IK filter → use MoveIt to filter feasible grasps
- Push direction → map “left/right” in image frame to Cartesian push vector in panda_link0 frame

## 6. Open Questions
- Exact ZED topic names and message types in franka_zed_gazebo
- Exact gripper model in the lab (Franka Hand vs other)
- Desired observation pose (joint configuration)
- How much of your IRM1 grasp/segmentation pipeline can be reused without changes

## 7. Immediate Next Steps
1. Update config.yaml with your ZED topics and test SceneObserver in isolation.
2. Define and test go_to_observation_pose() on Panda.
3. Create PandaSkillExecutor skeleton with no-op pick/place/push that only logs and returns to observation pose.
4. Wire PandaSkillExecutor into pragmabot_node.py where NotImplementedError currently sits.
5. Run the PragmaBot loop with rosbag replay (no real robot motion) to confirm planner → success detector → STM/LTM integration.
