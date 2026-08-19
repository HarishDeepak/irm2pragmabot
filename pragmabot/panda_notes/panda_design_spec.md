# PragmaBot × Franka Panda — Design Specification
**Project:** IRM2 — PragmaBot Reimplementation on Franka Emika Panda  
**Author:** Harish Deepak, PEARL Lab, TU Darmstadt  
**Source:** Analysis of PragmaBot codebase (`pragmabot_node.py`, `vlm_task_planner.py`, `vlm_success_detector.py`, `scene_observer.py`, `geometry.py`, `config.yaml`) against paper arXiv:2507.16713v2  
**Status:** Design phase — pre-implementation

---

## Table of Contents

1. [Overview and Goal](#1-overview-and-goal)
2. [Full Data Contract](#2-full-data-contract)
3. [Robot-Specific Assumptions and Panda Changes](#3-robot-specific-assumptions-and-panda-changes)
4. [PandaSkillExecutor Specification](#4-pandaskillexecutor-specification)
5. [Primitive and Policy Definitions](#5-primitive-and-policy-definitions)
6. [Perception and Annotation Interface](#6-perception-and-annotation-interface)
7. [Observation Pose Requirement](#7-observation-pose-requirement)
8. [Prompt Changes Required](#8-prompt-changes-required)
9. [Configuration Changes](#9-configuration-changes)
10. [Next Implementation Steps](#10-next-implementation-steps)

---

## 1. Overview and Goal

### Problem

PragmaBot (ETH RSL, IEEE RAL 2026) is a VLM-based task planning framework that enables robots to learn from real-world experience using short-term memory (STM), long-term memory (LTM), and retrieval-augmented generation (RAG). The original implementation targets a legged manipulator (ANYmal + 6-DoF arm + Robotiq 2F-140 + ZED X Mini).

The goal of this project is to reimplement PragmaBot on a **fixed-base Franka Emika Panda** arm in a tabletop manipulation setting, preserving the full VLM planning and memory pipeline while replacing only the robot-specific execution and perception layers.

### Design Principle

> Preserve the planner interface and replace the execution layer. Do not modify VLM modules, data schemas, or memory logic.

### What Must Not Change

- `NextBestAction` Pydantic schema (field names and types)
- `SuccessEvaluation` Pydantic schema
- `VLMTaskPlanner`, `VLMSuccessDetector`, `VLMSceneDescriber`, `VLMExperienceSummarizer`
- `MemoryManager`, STM/LTM/RAG pipeline
- `SceneObserver` interface (return types)
- `CameraIntrinsics`, `RigidTransform` geometry utilities
- `append_to_stm_if_activated()` and STM/LTM JSON format
- Overall `handle_planning_request` / `handle_evaluation_request` loop structure

### What Must Be Replaced

- The `NotImplementedError` execution blocks in `pragmabot_node.py`
- All robot-specific motion, gripper, and grasp logic
- IK feasibility filtering (Pinocchio → MoveIt)
- Grasp generation (AnyGrasp → existing Panda grasp stack from IRM1)
- System prompt robot descriptions
- ROS topic names in `config.yaml`

---

## 2. Full Data Contract

### 2.1 Main Execution Loop

```
User instruction (Gradio text box)
  │
  └─► handle_planning_request()
        │
        ├─► SceneObserver.get_scene_observation()
        │     → color_image (PIL.Image, RGB uint8)
        │     → depth_image (PIL.Image, float32 passthrough)
        │     → intrinsics (CameraIntrinsics)
        │     → observation_time (rospy.Time)
        │
        ├─► VLMSceneDescriber.get_scene_description()    [once, step t=0 only]
        │     → initial_scene_description (str)
        │
        ├─► MemoryManager.retrieve_relevant_experiences()  [once per task]
        │     → ltm (List[str])
        │
        ├─► VLMTaskPlanner.plan_action(instruction, color_image, stm, ltm)
        │     → NextBestAction
        │
        ├─► [PANDA EXECUTION — NotImplementedError]
        │     inputs:  NextBestAction + color_image + depth_image + intrinsics
        │     side effect: skill executed, arm returned to observation pose
        │
        └─► handle_evaluation_request()
              │
              ├─► SceneObserver.get_scene_observation()   [post-action image]
              │
              ├─► VLMSuccessDetector.perform_success_detection(
              │       task, action_str, img_before, img_after)
              │     → SuccessEvaluation
              │
              ├─► append_to_stm_if_activated("evaluation", success_eval)
              │
              ├─► [if task complete]
              │     → handle_experience_summarization()
              │           → VLMExperienceSummarizer → MemoryManager.save_experience()
              │
              └─► [if not complete]
                    → handle_planning_request()   [recurse]
```

### 2.2 `SceneObserver.get_scene_observation()` — Return Contract

| Field | Type | Source | Used by |
|---|---|---|---|
| `color_image` | `PIL.Image` (RGB uint8) | ZED CompressedImage → BGR decode → RGB | Planner, success detector, scene describer |
| `depth_image` | `PIL.Image` (float32, passthrough) | ZED depth Image msg | Execution layer only (3D pose estimation) |
| `intrinsics` | `CameraIntrinsics` | CameraInfo K matrix | Execution layer (point cloud unprojection) |
| `observation_time` | `rospy.Time` | depth_msg header stamp | Logging / TF lookup |

`SceneObserver` uses `message_filters.ApproximateTimeSynchronizer` (queue=10, slop=1.0s) over three topics simultaneously. It is a one-shot pull: `request_pending` gate ensures exactly one synchronized triplet is returned per call.

**Color is `CompressedImage`, depth is raw `Image` (passthrough).** Both must be available on your ZED driver.

### 2.3 `CameraIntrinsics` — Fields

```python
@dataclass(frozen=True)
class CameraIntrinsics:
    width:  int
    height: int
    fx:     float   # K[0]
    fy:     float   # K[4]
    ppx:    float   # K[2]  (principal point x)
    ppy:    float   # K[5]  (principal point y)
```

Populated directly from `sensor_msgs/CameraInfo.K` array. No changes needed.

### 2.4 `VLMTaskPlanner.plan_action()` — Inputs and Output

**Inputs:**

| Argument | Type | Notes |
|---|---|---|
| `task` | `str` | Raw user instruction string |
| `color_image` | `PIL.Image` | Current pre-action RGB observation |
| `stm` | `List[str]` | JSON strings from `append_to_stm_if_activated` |
| `ltm` | `List[str]` | Retrieved experience strings from `MemoryManager` |

**Output — `NextBestAction` (Pydantic model, must not be changed):**

| Field | Type | Required | Notes |
|---|---|---|---|
| `scene_description` | `str` | Always | VLM's scene description |
| `applicable_knowledge` | `str \| None` | Only when LTM provided | |
| `chain_of_thought_reasoning` | `str` | Always | VLM reasoning trace |
| `chosen_action` | `str` | Always | Natural language — passed verbatim to success detector |
| `chosen_skill` | `RobotSkill` enum | Always | `"pick"` / `"place"` / `"push"` |
| `target_object` | `str` | Always | Object name string |
| `should_grasp_at_specific_section` | `bool \| None` | pick only | Annotation request flag |
| `placement_object` | `str \| None` | place only | Receptacle object name |
| `should_place_at_specific_section` | `bool \| None` | place only | Annotation request flag |
| `push_direction` | `PushDirection` enum | push only | `"left"` / `"right"` (image frame) |

### 2.5 Execution Layer Contract

The execution layer (your `PandaSkillExecutor`) sits at the `NotImplementedError` in `handle_planning_request`. It must satisfy:

**Inputs consumed:**
```
next_action:   NextBestAction   (from planner)
color_image:   PIL.Image        (pre-action RGB, already captured)
depth_image:   PIL.Image        (pre-action depth, already captured)
intrinsics:    CameraIntrinsics (already captured)
```

**Outputs produced:** None (side effect only).

**Postcondition (mandatory):** Robot arm has returned to the fixed observation pose. Gripper state reflects outcome. Scene has settled.

**Error handling:** Must not raise on recoverable motion failures. Log internally and let the success detector classify the outcome from the images.

### 2.6 `VLMSuccessDetector.perform_success_detection()` — Contract

**Inputs:**

| Argument | Type | Source |
|---|---|---|
| `task` | `str` | Original user instruction |
| `action_to_detect` | `str` | `next_action.chosen_action` (verbatim NL string from planner) |
| `color_image_before` | `PIL.Image` | `self.color_image` captured before execution |
| `color_image_after` | `PIL.Image` | New `SceneObserver` call after execution |

**Output — `SuccessEvaluation` (Pydantic model, must not be changed):**

| Field | Type | Meaning |
|---|---|---|
| `scene_description` | `str` | What changed between before/after images |
| `is_action_successful` | `bool` | Did this specific action succeed |
| `is_task_completed` | `bool` | Is the entire task done |

### 2.7 STM Entry Format

Every step appends two JSON entries to `self.stm` via `append_to_stm_if_activated()`:

```json
// After planning (key = "action"):
{
  "time_step": 1,
  "action": {
    "scene_description": "...",
    "chain_of_thought_reasoning": "...",
    "chosen_action": "Pick up the apple.",
    "chosen_skill": "pick",
    "target_object": "apple",
    "should_grasp_at_specific_section": false
  }
}

// After evaluation (key = "evaluation"):
{
  "time_step": 1,
  "evaluation": {
    "scene_description": "Apple is now held by the gripper.",
    "is_action_successful": true,
    "is_task_completed": false
  }
}
```

`None`/unset fields are excluded (`exclude_none=True, exclude_unset=True`).

### 2.8 `RigidTransform` — Geometry Utility

```python
@dataclass(frozen=True)
class RigidTransform:
    position: np.ndarray  # shape (3,)
    rotation: np.ndarray  # shape (3, 3)
```

Supports: `.inv()`, `T1 * T2` (composition), `T * point` (3,), `T * cloud` (N,3).  
Used internally in the execution/annotation layer for camera↔base transforms. Fully reusable.

---

## 3. Robot-Specific Assumptions and Panda Changes

### 3.1 Platform

| Assumption | Original | Panda Impact | Action |
|---|---|---|---|
| System prompt: "legged robot with a single arm" | `PLANNER_SYSTEM_PROMPT`, `DETECTOR_SYSTEM_PROMPT` | Wrong platform description → planner may reason incorrectly | **Rewrite both prompts** |
| Locomotion-capable base | ANYmal quadruped | Panda is fixed-base; no locomotion primitives | **Remove locomotion references from prompts** |
| Gripper: Robotiq 2F-140 | 140mm max aperture | Franka Hand (80mm) or lab gripper | **Verify grip width; update prompt if needed** |
| Arm: 6-DoF on mobile base | ANYmal + arm | Panda is 7-DoF fixed | Larger workspace, no base mobility |

### 3.2 Camera / Observation

| Assumption | Location | Panda Impact | Action |
|---|---|---|---|
| Color topic: `/zedxm/zed_node/rgb/image_rect_color/compressed` | `config.yaml` | Different ZED node name on Panda | Update topic name |
| Depth topic: `/zedxm/zed_node/depth/depth_registered` | `config.yaml` | Same | Update topic name |
| Camera info topic: `/zedxm/zed_node/rgb/camera_info` | `config.yaml` | Same | Update topic name |
| Color published as `CompressedImage` | `scene_observer.py` | Verify your ZED driver publishes compressed color; if not, change subscriber to `ImageMsg` | Check ZED driver config |
| Camera mounted on arm **elbow** | Paper hardware section | Your ZED is likely wrist- or fixed-mount; affects observable workspace at each step | Account for camera pose at observation pose |

### 3.3 Execution Layer

| Assumption | Location | Panda Impact | Action |
|---|---|---|---|
| `NotImplementedError` for action execution | `pragmabot_node.py` line 133 | Entire skill dispatch missing | **Implement `PandaSkillExecutor`** |
| `NotImplementedError` for re-planning trigger | `pragmabot_node.py` line 177 | Re-planning logic missing | **Call `handle_planning_request` directly** |
| AnyGrasp for grasp proposals | Paper / annotation | Not available for Panda stack | Replace with IRM1 grasp pipeline |
| Pinocchio for IK feasibility filtering | Paper / annotation | Not in your stack | Replace with MoveIt `compute_ik` service |

### 3.4 Coordinate Frames

| Assumption | Panda Impact | Action |
|---|---|---|
| Camera-to-robot-base TF chain calibrated for original robot | Need ZED extrinsics → `panda_link0` | Calibrate and publish static TF |
| Push direction "left/right" defined in image frame | Compatible — direction is image-space | Map to Panda Cartesian frame using camera orientation at obs pose |

### 3.5 Workspace Constraints (Compatible As-Is)

- `PUSH` works only on objects directly on the table — compatible with tabletop Panda
- `PLACE` requires specifying receptacle object — compatible
- Scene reset by human operator after failure — compatible, keep as-is

---

## 4. PandaSkillExecutor Specification

### 4.1 Interface

```python
from pragmabot.vlm_task_planner import NextBestAction, RobotSkill
from pragmabot.geometry import CameraIntrinsics
from PIL import Image

class PandaSkillExecutor:
    """
    Dispatches pick/place/push to Franka Panda via MoveIt.
    Sits at the NotImplementedError in handle_planning_request.
    Must return to observation pose after every skill.
    Must not raise on recoverable motion failures.
    """

    def execute(
        self,
        action: NextBestAction,
        color_image: Image.Image,
        depth_image: Image.Image,
        intrinsics: CameraIntrinsics,
    ) -> None:
        skill = action.chosen_skill

        if skill == RobotSkill.PICK:
            self._pick(
                target_object=action.target_object,
                annotate=bool(action.should_grasp_at_specific_section),
                color_image=color_image,
                depth_image=depth_image,
                intrinsics=intrinsics,
            )
        elif skill == RobotSkill.PLACE:
            self._place(
                placement_object=action.placement_object,
                annotate=bool(action.should_place_at_specific_section),
                color_image=color_image,
                depth_image=depth_image,
                intrinsics=intrinsics,
            )
        elif skill == RobotSkill.PUSH:
            self._push(
                target_object=action.target_object,
                direction=action.push_direction,   # "left" | "right"
                color_image=color_image,
                depth_image=depth_image,
                intrinsics=intrinsics,
            )

        self._return_to_observation_pose()  # MANDATORY
```

### 4.2 Integration into `pragmabot_node.py`

**Replace `NotImplementedError` in `handle_planning_request` (line 133):**

```python
# Before (original):
raise NotImplementedError(
    "Implement your own action execution logic here..."
)

# After (Panda):
self.panda_executor.execute(
    next_action,
    self.color_image,
    self.depth_image,
    self.intrinsics,
)
self.handle_evaluation_request(chatbot)
```

**Replace `NotImplementedError` in `handle_evaluation_request` (line 177):**

```python
# Before (original):
raise NotImplementedError(
    "Implement your own logic here to decide when to call the planning function again..."
)

# After (Panda):
self.handle_planning_request(chatbot)
```

The re-planning trigger for Panda is unconditional: always replan immediately after a non-complete evaluation. This matches the `rosbag_replay` code path that already exists.

### 4.3 Behaviour Contracts per Skill

**`_pick(target_object, annotate, color_image, depth_image, intrinsics)`**

- Segment `target_object` in `color_image` → 2D mask
- Unproject masked depth → 3D point cloud in camera frame
- Transform to `panda_link0` frame
- Generate grasp candidates (IRM1 grasp pipeline)
- If `annotate=True`: run FPS on mask → overlay numbered regions → VLM selects preferred region → filter grasps by proximity to selected region
- Filter grasps by MoveIt IK feasibility
- Score: `g* = argmax sconf(g) * sloc(g)` (if annotation used) or `argmax sconf(g)` otherwise
- Execute: move to pre-grasp → approach → close gripper → lift to clearance height
- On failure: log warning, do not raise
- On completion: arm at observation pose, gripper closed (object held) or open (pick failed)

**`_place(placement_object, annotate, color_image, depth_image, intrinsics)`**

- Precondition: gripper is closed / holding an object
- Segment `placement_object` in `color_image` → 2D mask
- Apply FPS on mask → candidate placement poses
- If `annotate=True`: VLM selects preferred placement location from overlaid candidates
- Compute 3D placement pose in `panda_link0` frame
- Execute: move above placement pose → descend → open gripper → retract
- On completion: arm at observation pose, gripper open

**`_push(target_object, direction, color_image, depth_image, intrinsics)`**

- `direction` is `"left"` or `"right"` in **image frame** (defined from camera perspective)
- Segment `target_object` → centroid in image space → project to 3D in `panda_link0` frame
- Map image-frame direction to Cartesian push vector:
  - `"left"` in image → negative camera-x direction projected onto table plane
  - `"right"` in image → positive camera-x direction projected onto table plane
  - Requires: camera orientation TF at observation pose
- Compute contact approach pose (offset from object centroid opposite to push direction)
- Execute: move to contact approach → push along direction → retract
- Constraint: object must be on table (enforced by planner prompt, not by executor)
- On completion: arm at observation pose, gripper open

### 4.4 Error Handling Policy

The executor must not raise on recoverable motion failures. The VLM success detector is the failure classifier — it receives before/after images and determines whether the action succeeded. If the executor fails to complete a motion (e.g., IK failure, collision abort), it should:

1. Log the error internally
2. Call `_return_to_observation_pose()` unconditionally
3. Return normally

The success detector will see an unchanged scene and classify the action as failed, triggering STM self-reflection and re-planning.

---

## 5. Primitive and Policy Definitions

### 5.1 Core Manipulation Primitives

These are the three skills the VLM planner can select. They must be exposed to the executor and fully implemented before VLM integration.

| Primitive | Signature | Precondition | Postcondition |
|---|---|---|---|
| `pick` | `pick(target_object, annotate, color, depth, intrinsics)` | Gripper open; arm at obs pose | Object in gripper OR failed; arm at obs pose |
| `place` | `place(placement_object, annotate, color, depth, intrinsics)` | Object in gripper; arm at obs pose | Object on receptacle; gripper open; arm at obs pose |
| `push` | `push(target_object, direction, color, depth, intrinsics)` | Object on table; arm at obs pose | Object displaced; arm at obs pose |

### 5.2 Utility Primitives (Internal)

These are not selectable by the VLM planner. They are called internally by the executor and core primitives.

| Primitive | Purpose | Notes |
|---|---|---|
| `go_to_observation_pose()` | Move to fixed joint configuration for consistent RGB-D capture | Must be called after every skill; defined once and saved |
| `go_to_home()` | Move to safe collapsed configuration for startup/reset | Conservative joint config away from table |
| `open_gripper()` | Send open command to Franka gripper / lab gripper | Direct hardware command |
| `close_gripper()` | Send grasp/close command | May use force threshold |
| `recover()` | Attempt recovery from stuck or collision state | Try home pose; if MoveIt reports failure, go to home and stop |
| `is_holding_object()` | Check current gripper width against threshold | Used for pre-condition assertion in `place`; planner prompt already enforces "must PLACE before picking again" |

### 5.3 VLM Planner Constraints (Already in Prompt — Do Not Change)

The following constraints are already encoded in `PLANNER_INSTRUCTION_PROMPT` and do not need to be enforced by the executor (the VLM enforces them at planning time):

- `PUSH` only on objects directly on the table
- Cannot `PUSH` object off another object
- Cannot `PUSH` object behind another object
- Must `PLACE` held object before picking again
- Cannot `PICK` tiny or flat objects
- Never repeat same failed action immediately
- Any object on table can be used as tool

---

## 6. Perception and Annotation Interface

### 6.1 Overview

The annotation module is not shipped in the public PragmaBot release. The README recommends: GroundedSAM (Grounding DINO + SAM) for segmentation, AnyGrasp for grasp generation, Pinocchio for IK filtering. For Panda, these are replaced with your existing IRM1 stack.

Annotation is **on-demand**: the planner sets `should_grasp_at_specific_section=True` only for complex objects (e.g., skewers, drumsticks). For simple objects (apple, box), annotation is skipped.

### 6.2 Required Perception Sub-Components

| Component | Input | Output | Recommended Tool |
|---|---|---|---|
| Object segmenter | RGB image + object name (str) | 2D binary mask (H×W numpy bool) | SAM2 + text prompt (you have this from IRM1); or GroundedSAM |
| Point cloud builder | Depth float32 array + `CameraIntrinsics` | Open3D `PointCloud` in camera frame | Open3D (you have this from IRM1) |
| Camera→base transform | Point(s) in camera frame | Point(s) in `panda_link0` frame | `tf2` lookup of ZED extrinsic static TF |
| Grasp generator | Point cloud (masked) + optional region hint | List of candidate grasp poses + confidence scores | IRM1 grasp stack (replaces AnyGrasp) |
| IK feasibility filter | List of grasp poses in `panda_link0` frame | Feasible subset | MoveIt `compute_ik` service (`moveit_msgs/GetPositionIK`) |
| FPS candidate regions | Binary mask + point cloud | N candidate 3D region centroids | Open3D FPS on masked points |
| Push direction mapper | `"left"` / `"right"` + camera-to-base TF | Push vector in `panda_link0` XY plane | Camera x-axis projected to table plane |
| VLM region selector | RGB image + overlaid numbered region masks | Selected region index | `VLMClient.query_structured` (already available) |

### 6.3 Depth Image Format

`SceneObserver` returns depth as `PIL.Image` wrapping a float32 array (passthrough encoding from `sensor_msgs/Image`). To use in Open3D:

```python
import numpy as np
depth_array = np.array(depth_image, dtype=np.float32)  # shape (H, W), values in meters
```

### 6.4 Point Cloud Unprojection

Using `CameraIntrinsics`:

```python
def depth_to_pointcloud(depth_array, intrinsics):
    h, w = depth_array.shape
    u, v = np.meshgrid(np.arange(w), np.arange(h))
    z = depth_array
    x = (u - intrinsics.ppx) * z / intrinsics.fx
    y = (v - intrinsics.ppy) * z / intrinsics.fy
    points = np.stack([x, y, z], axis=-1).reshape(-1, 3)
    return points[np.isfinite(points[:, 2]) & (points[:, 2] > 0)]
```

### 6.5 Camera TF Chain

The ZED camera must have a calibrated static TF published from the camera optical frame to `panda_link0`. This is a hard prerequisite for all 3D pose estimation.

```
panda_link0
  └─► panda_link8 (or mount link)
        └─► zed_camera_link
              └─► zed_left_camera_optical_frame   ← all point clouds in this frame
```

Publish as a `static_transform_publisher` node in your launch file, with values from your hand-eye calibration.

### 6.6 IK Filter Replacement (Pinocchio → MoveIt)

The original code uses Pinocchio to filter kinematically infeasible grasps. Replace with MoveIt's IK service:

```python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from geometry_msgs.msg import PoseStamped

ik_service = rospy.ServiceProxy('/compute_ik', GetPositionIK)

def is_ik_feasible(pose_stamped: PoseStamped, group="panda_arm") -> bool:
    req = GetPositionIKRequest()
    req.ik_request.group_name = group
    req.ik_request.pose_stamped = pose_stamped
    req.ik_request.timeout = rospy.Duration(0.5)
    resp = ik_service(req)
    return resp.error_code.val == 1  # MoveItErrorCodes.SUCCESS
```

Apply this filter over all grasp candidates before scoring.

---

## 7. Observation Pose Requirement

### 7.1 The Contract

The success detector's image caption in `vlm_success_detector.py` states explicitly:

> *"This image is captured after the robot completed the action and **returned its arm to the default position**."*

This is a **hard contract**, not a suggestion. The VLM is prompted to expect the post-action image to show the robot arm in a neutral pose. If the arm remains in an arbitrary post-skill configuration, the VLM may misclassify success/failure because the arm occludes the scene or changes the apparent object positions.

### 7.2 Requirements

- Define **exactly one** observation joint configuration. Save it as a named target in MoveIt or as a hardcoded joint angle array.
- Call `go_to_observation_pose()` as the **last action** inside every skill (`_pick`, `_place`, `_push`), before returning to the executor.
- This also applies after failed motions — `_return_to_observation_pose()` must be in a `finally` block.

### 7.3 Observation Pose Design Criteria

- Arm must be out of the field of view of the ZED camera (or at minimum at the image boundary)
- Full tabletop workspace must be visible from the ZED at this pose
- Pose must be reachable from any expected end-of-skill arm position
- Pose must be collision-free with the table and any common objects

### 7.4 Implementation Pattern

```python
def _pick(self, ...):
    try:
        # ... grasp logic ...
        pass
    except Exception as e:
        rospy.logwarn(f"Pick failed: {e}")
    finally:
        self._return_to_observation_pose()  # Always runs

def _return_to_observation_pose(self):
    self.move_group.set_named_target("observation_pose")
    self.move_group.go(wait=True)
    self.move_group.stop()
    self.move_group.clear_pose_targets()
```

---

## 8. Prompt Changes Required

### 8.1 `PLANNER_SYSTEM_PROMPT` — `vlm_task_planner.py`

**Current:**
```
You are a helpful assistant for a legged robot equipped with a single arm
and a two-finger gripper.
```

**Replace with:**
```
You are a helpful assistant for a fixed-base Franka Emika Panda robot arm
mounted on a table, equipped with a parallel-jaw gripper. The robot cannot
move its base — it operates within a fixed tabletop workspace.
```

### 8.2 `DETECTOR_SYSTEM_PROMPT` — `vlm_success_detector.py`

**Current:**
```
You are a helpful assistant for a legged robot equipped with a single arm
and a two-finger gripper.
```

**Replace with:**
```
You are a helpful assistant for a fixed-base Franka Emika Panda robot arm
mounted on a table, equipped with a parallel-jaw gripper.
```

### 8.3 No Other Prompt Changes Needed

`PLANNER_INSTRUCTION_PROMPT` does not contain locomotion-specific language. Push direction rules ("prefer pushing left if object is on left side of image") are image-frame and remain valid for Panda.

---

## 9. Configuration Changes

### 9.1 `config.yaml` — Topics

Update the three ROS topic names to match your ZED2/ZED2i driver:

```yaml
# Original (ANYmal ZED X Mini):
topics:
  color_image: /zedxm/zed_node/rgb/image_rect_color/compressed
  depth_image: /zedxm/zed_node/depth/depth_registered
  camera_info: /zedxm/zed_node/rgb/camera_info

# Panda ZED (update to your actual topic names):
topics:
  color_image: /zed/zed_node/rgb/image_rect_color/compressed
  depth_image: /zed/zed_node/depth/depth_registered
  camera_info: /zed/zed_node/rgb/camera_info
```

### 9.2 ZED Driver Requirement

`SceneObserver` expects:
- Color: `sensor_msgs/CompressedImage` on the color topic
- Depth: `sensor_msgs/Image` (float32, passthrough encoding) on the depth topic
- Camera info: `sensor_msgs/CameraInfo` on the info topic

Verify these are published by your ZED ROS driver. If color is published as uncompressed `Image`, change the subscriber type in `scene_observer.py`:

```python
# Original (compressed):
self.color_sub = message_filters.Subscriber(self.color_image_topic, CompressedImageMsg)

# If uncompressed:
self.color_sub = message_filters.Subscriber(self.color_image_topic, ImageMsg)
# And update _sync_callback decode:
color_array = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="rgb8")
color_image = Image.fromarray(color_array)
```

### 9.3 Other Config Fields — No Changes Needed

| Field | Value | Status |
|---|---|---|
| `rosbag_replay` | `false` (real robot) | Change to `false` when running live |
| `activate_stm` | `true` | Keep |
| `activate_ltm` | `true` | Keep |
| `save_to_ltm` | `true` | Keep |
| `retrieval_top_k` | `5` | Keep |
| `vlm.vlm_model` | `gpt-4o-2024-08-06` | Keep |
| `vlm.text_embedding_model` | `text-embedding-3-large` | Keep |

---

## 10. Next Implementation Steps

Steps are ordered by dependency. Do not proceed to step N+1 until step N is verified.

### Step 1 — Verify `SceneObserver` on Panda

Update topic names in `config.yaml`. Launch your ZED driver and test `SceneObserver.get_scene_observation()` in isolation:

```python
obs = scene_observer.get_scene_observation()
# Verify: color_image is PIL RGB, depth_image is float32 PIL, intrinsics are non-zero
```

Confirm synchronized color + depth are returned without timeout.

### Step 2 — Define and Save Observation Pose

Move the Panda arm manually (freedrive or MoveIt RViz) to a position where:
- The full tabletop workspace is visible to the ZED
- The arm is not occluding the scene
- The pose is comfortably reachable from any post-skill configuration

Save as a MoveIt named target (`"observation_pose"`) or hardcode as joint angles. Implement and test `go_to_observation_pose()`.

### Step 3 — Implement Utility Primitives

Implement and test individually, in this order:
1. `go_to_home()`
2. `open_gripper()` / `close_gripper()`
3. `go_to_observation_pose()` (already done in step 2)
4. `is_holding_object()` (gripper width threshold check)
5. `recover()` (go to home; log warning)

No VLM integration yet. Run each on the real robot and verify.

### Step 4 — Stub Executor (End-to-End Pipeline Test)

Implement `PandaSkillExecutor` with all three skills as no-ops:

```python
def _pick(self, **kwargs):
    rospy.loginfo("STUB: pick called")
    # no motion

def _place(self, **kwargs):
    rospy.loginfo("STUB: place called")

def _push(self, **kwargs):
    rospy.loginfo("STUB: push called")
```

Wire into `handle_planning_request`. Set `rosbag_replay: false`. Run the full pipeline (instruction → planner → stub executor → observation pose → success detector → STM → LTM) and verify the loop completes without error. This validates the planner↔executor↔detector↔memory integration before adding real motion.

### Step 5 — Perception Stack (Segmentation + Point Cloud + TF)

Implement the perception sub-components needed for real skills:
1. Object segmenter (SAM2 or GroundedSAM with text prompt)
2. `depth_to_pointcloud()` utility
3. Static TF: ZED optical frame → `panda_link0` (from hand-eye calibration)
4. `transform_pointcloud_to_base()` using `tf2`

Test segmentation on a known object on the table. Verify 3D centroid in base frame.

### Step 6 — Implement `pick`

Build the pick skill:
1. Segment object → mask
2. Build masked point cloud
3. Run IRM1 grasp generator on point cloud
4. Filter grasps with MoveIt IK
5. Select best grasp (confidence score)
6. Execute: pre-grasp approach → grasp → lift
7. Call `_return_to_observation_pose()`

Test with a fixed known object (apple, box). Do not enable VLM annotation yet (`should_grasp_at_specific_section=False` for simple objects).

### Step 7 — Implement `place`

Build the place skill:
1. Segment receptacle → mask
2. Compute placement centroid in 3D
3. FPS candidates if `annotate=True`
4. Execute: move above → descend → open gripper → retract
5. Call `_return_to_observation_pose()`

Test by instructing "put the apple on the plate."

### Step 8 — Implement `push`

Build the push skill:
1. Segment object → 3D centroid
2. Map `"left"`/`"right"` to Cartesian push vector using camera-to-base TF
3. Compute contact approach pose (offset opposite push direction, at table height)
4. Execute: move to contact → push along direction → retract
5. Call `_return_to_observation_pose()`

Test with a simple pushable object.

### Step 9 — Full Integration Test (STM Self-Reflection)

Run a challenging task (e.g., "put the apple on the plate" with an occluder) multiple times. Verify:
- On first failure, VLM generates a self-reflection in STM
- On second attempt, the planner adapts (e.g., pushes occluder first)
- STM entries are formatted correctly as JSON

### Step 10 — LTM + RAG Test

After collecting successful episodes, verify:
- Experiences are summarized and saved to LTM CSV
- On a structurally similar new task, relevant memories are retrieved via cosine similarity
- First-attempt success rate improves compared to no-LTM baseline

### Step 11 — Annotation (Optional, Add Last)

Implement the VLM annotation module for complex objects:
1. FPS within object mask → N candidate region centroids
2. Overlay numbered region masks on RGB image
3. Call `VLMClient.query_structured` to select preferred region
4. Filter and score grasps by proximity to selected region

Enable only when `should_grasp_at_specific_section=True` (set by planner for non-trivial objects).

---

## Appendix A — Files Analyzed

| File | Status | Key Findings |
|---|---|---|
| `pragmabot_node.py` | Fully read | Main execution loop; two `NotImplementedError` integration points |
| `vlm_task_planner.py` | Fully read | `NextBestAction` schema; 3 skills; full prompt templates |
| `vlm_success_detector.py` | Fully read | `SuccessEvaluation` schema; image-only; observation pose postcondition |
| `scene_observer.py` | Fully read | Synchronized CompressedImage + depth + CameraInfo; PIL return types |
| `geometry.py` | Fully read | `RigidTransform`, `CameraIntrinsics`; fully reusable |
| `config.yaml` | Fully read | Topic names; VLM model; memory flags |
| Annotation modules | Not released | Must implement from scratch per README recommendations |

## Appendix B — What the Public Repo Does Not Include

Per README: action execution (step 4 in the pipeline) is intentionally a `NotImplementedError`. The annotation/grasp/IK modules are not shipped. The README recommends:

- **Segmentation:** [Grounded SAM](https://github.com/IDEA-Research/Grounded-Segment-Anything)
- **Grasp generation:** [GraspGen](https://github.com/NVlabs/GraspGen) (or AnyGrasp as used in paper)
- **IK / kinematics:** [Pinocchio](https://stack-of-tasks.github.io/pinocchio/)

For Panda, replace with:
- **Segmentation:** SAM2 (from IRM1) or GroundedSAM
- **Grasp generation:** IRM1 grasp pipeline
- **IK:** MoveIt `compute_ik` service

---

*End of design specification.*
