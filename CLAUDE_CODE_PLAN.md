# PragmaBot × Panda — Claude Code Implementation Plan
**PEARL Lab, TU Darmstadt | IRM2 | Branch: `feature/panda-adaptation`**

---

## Context for Claude Code

You are helping implement the Panda-specific execution layer for PragmaBot, a VLM-based robotic task planning system (ETH Zurich RSL, IEEE RAL 2026).

**The rule:** Everything above `NotImplementedError` in `pragmabot_node.py` is untouched. You only build the execution layer below it.

**Environment:**
- Container: `pragmabot_panda` (Docker, `3liyounes/pearl_robots:franka`)
- Catkin workspace: `/catkin_ws`
- Project path: `/catkin_ws/src/pragmabot`
- ROS: Noetic, Ubuntu 20.04, Python 3.8
- Branch: `feature/panda-adaptation`

**Already in container:** `panda_moveit_config`, `franka_gripper`, MoveIt (KDL), `actionlib`

**Not yet installed:** `zed_wrapper`, GroundedSAM, GraspGen, `ros-numpy`, `message-filters`

---

## Phase 1 — Environment Verification

**Goal:** Confirm what is in the container, install what is missing, verify the base PragmaBot node launches cleanly.

### 1.1 Verify existing ROS packages

Run each of these. Record which ones fail:

```bash
rospack find panda_moveit_config
rospack find franka_gripper
rospack find franka_ros
rospack find actionlib
rospack find moveit_ros_planning_interface
```

### 1.2 Verify Python imports

```bash
python3 -c "import moveit_commander; print('moveit_commander ok')"
python3 -c "import actionlib; print('actionlib ok')"
python3 -c "import openai; print('openai ok')"
python3 -c "import gradio; print('gradio ok')"
python3 -c "import pydantic; print('pydantic ok')"
python3 -c "import omegaconf; print('omegaconf ok')"
```

### 1.3 Install missing Python dependencies

```bash
pip install ros-numpy --break-system-packages
pip install numpy opencv-python Pillow requests --break-system-packages
# Verify
python3 -c "import ros_numpy; print('ros_numpy ok')"
```

### 1.4 Verify base PragmaBot builds and imports cleanly

```bash
cd /catkin_ws
catkin_make --pkg pragmabot
source devel/setup.bash
```

Then launch — it should fail ONLY at the OpenAI API key check, not from import errors:

```bash
roslaunch pragmabot launch_pragmabot.launch
```

**Expected:** Node starts, Gradio URL prints, then fails with `OPENAI_API_KEY not set` or similar. Any other error (ImportError, missing package) must be fixed before proceeding.

### 1.5 Verify the repo structure matches expectations

Check these files exist:

```bash
ls /catkin_ws/src/pragmabot/pragmabot/nodes/pragmabot_node.py
ls /catkin_ws/src/pragmabot/pragmabot/src/pragmabot/vlm_task_planner.py
ls /catkin_ws/src/pragmabot/pragmabot/src/pragmabot/vlm_client.py
ls /catkin_ws/src/pragmabot/pragmabot/src/pragmabot/scene_observer.py
ls /catkin_ws/src/pragmabot/pragmabot/config/config.yaml
```

### 1.6 Find and record the NotImplementedError location

```bash
grep -n "NotImplementedError" /catkin_ws/src/pragmabot/pragmabot/nodes/pragmabot_node.py
```

Record the exact line number and surrounding function name. This is the integration point for Phase 6.

### Phase 1 done when:
- All rospack finds succeed
- moveit_commander, actionlib, ros_numpy import cleanly
- `catkin build pragmabot` succeeds with zero errors
- `roslaunch launch_pragmabot.launch` fails only at API key, not at imports

---

## Phase 2 — ROS .action Message Definitions + Package Scaffold

**Goal:** Create the `action/` directory, define the three `.action` files, update `CMakeLists.txt` and `package.xml`, and create empty stub files for everything we will build. Build must succeed.

**Do this phase without a robot — it is pure file creation.**

### 2.1 Create directory structure

```bash
cd /catkin_ws/src/pragmabot/pragmabot
mkdir -p action
```

### 2.2 Create `action/PandaPick.action`

```
# Goal
string target_object
bool use_annotation
---
# Result
bool success
string message
geometry_msgs/PoseStamped grasp_pose
---
# Feedback
string status
```

### 2.3 Create `action/PandaPlace.action`

```
# Goal
string target_location
geometry_msgs/PoseStamped place_pose
---
# Result
bool success
string message
---
# Feedback
string status
```

### 2.4 Create `action/PandaPush.action`

```
# Goal
string target_object
string goal_region
---
# Result
bool success
string message
---
# Feedback
string status
```

### 2.5 Update `CMakeLists.txt`

Add to the existing `CMakeLists.txt` (do NOT replace — surgical additions only):

In `find_package(catkin REQUIRED COMPONENTS ...)` block, add:
```
actionlib
actionlib_msgs
geometry_msgs
message_generation
```

Add after `find_package`:
```cmake
add_action_files(
  FILES
  PandaPick.action
  PandaPlace.action
  PandaPush.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
)
```

In `catkin_package(...)`, add to `CATKIN_DEPENDS`:
```
actionlib actionlib_msgs geometry_msgs message_runtime
```

### 2.6 Update `package.xml`

Add these lines inside `<package>` if not present:
```xml
<build_depend>actionlib</build_depend>
<build_depend>actionlib_msgs</build_depend>
<build_depend>geometry_msgs</build_depend>
<build_depend>message_generation</build_depend>
<exec_depend>actionlib</exec_depend>
<exec_depend>actionlib_msgs</exec_depend>
<exec_depend>geometry_msgs</exec_depend>
<exec_depend>message_runtime</exec_depend>
```

### 2.7 Create empty stub files (so imports resolve before logic is written)

Create each file with just the class skeleton and a `pass` or `raise NotImplementedError`:

**`src/pragmabot/grounded_sam.py`** — stub:
```python
class GroundedSAM:
    def __init__(self):
        raise NotImplementedError("GroundedSAM not yet installed — Phase 3")
    def segment(self, image, text, box_threshold=0.3, text_threshold=0.25):
        raise NotImplementedError
```

**`src/pragmabot/graspgen_client.py`** — stub:
```python
class GraspGenClient:
    def __init__(self, url="http://localhost:8080/grasp"):
        self.url = url
    def generate(self, point_cloud, gripper="franka_panda", num_grasps=20):
        raise NotImplementedError("GraspGen not yet installed — Phase 4")
```

**`src/pragmabot/panda_skill_executor.py`** — stub:
```python
import rospy

class PandaSkillExecutor:
    def __init__(self):
        rospy.loginfo("PandaSkillExecutor stub — Phase 6 will complete this")

    def execute(self, action) -> dict:
        """Stub: returns fake success so VLM loop can be tested offline."""
        rospy.loginfo(f"[STUB] execute called: skill={action.skill}, target={action.target_object}")
        return {
            "success": True,
            "pre_image": None,
            "post_image": None,
            "message": "stub response",
        }
```

**`nodes/panda_pick_server.py`** — stub:
```python
#!/usr/bin/env python3
import rospy
import actionlib
from pragmabot.msg import PandaPickAction, PandaPickResult

class PandaPickServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer(
            "/panda/pick", PandaPickAction, execute_cb=self.execute_cb, auto_start=False
        )
        self.server.start()
        rospy.loginfo("PandaPickServer STUB ready")

    def execute_cb(self, goal):
        rospy.loginfo(f"[pick stub] goal: {goal.target_object}")
        result = PandaPickResult(success=False, message="not implemented yet")
        self.server.set_aborted(result)

if __name__ == "__main__":
    rospy.init_node("panda_pick_server")
    PandaPickServer()
    rospy.spin()
```

Create equivalent stubs for `nodes/panda_place_server.py` and `nodes/panda_push_server.py`.

### 2.8 Create `launch/launch_panda_servers.launch`

```xml
<launch>
  <node name="panda_pick_server"  pkg="pragmabot" type="panda_pick_server.py"  output="screen"/>
  <node name="panda_place_server" pkg="pragmabot" type="panda_place_server.py" output="screen"/>
  <node name="panda_push_server"  pkg="pragmabot" type="panda_push_server.py"  output="screen"/>
</launch>
```

### 2.9 Make server nodes executable

```bash
chmod +x /catkin_ws/src/pragmabot/pragmabot/nodes/panda_pick_server.py
chmod +x /catkin_ws/src/pragmabot/pragmabot/nodes/panda_place_server.py
chmod +x /catkin_ws/src/pragmabot/pragmabot/nodes/panda_push_server.py
```
The launch file

<launch>
  <node name="panda_pick_server" pkg="pragmabot" type="panda_pick_server.py" output="screen"/>
  ...
</launch>

This is a ROS launch file. It starts all three action server nodes in one command instead of you having to open 3 terminals and run each script manually. When you run:
roslaunch pragmabot launch_panda_servers.launch
ROS reads this file and spawns all three nodes at once.

---
chmod +x

ROS needs to execute these Python files directly (like a program, not python3 file.py). chmod +x marks them as executable so the OS allows that. Withouh a "permission denied" error.

---                                                                                                           Why CMakeLists.txt is updated

catkin_install_python(PROGRAMS ...) tells catkin which Python scripts are ROS nodes. If a script isn't listed there, roslaunch won't find it via the type= attribute. That's why the three server files were added to that  list.
                                                                                                              The add_action_files + generate_messagesenerate Python/C++ message classes fromyour .action files. That's what produces pragmabot.msg.PandaPickAction etc. — without those lines, the import would fail.

---
Why package.xml is updated

package.xml is the dependency declaration for the ROS build system. It lists what your package needs to build and run. Adding actionlib, actionlib_msgs, geometry_msgs, message_generation, message_runtime there tells:
- other developers what to install
- rosdep what to auto-install
- catkin what to link against

Short version: CMakeLists.txt = how to build. package.xml = what is needed to build.

### 2.10 Build and verify action messages generated

```bash
cd /catkin_ws
catkin build pragmabot
source devel/setup.bash
python3 -c "from pragmabot.msg import PandaPickAction, PandaPlaceAction, PandaPushAction; print('action msgs ok')"
```

### Phase 2 done when:
- `catkin build` succeeds with zero errors
- Action message import succeeds
- Stub servers launch without error via `roslaunch pragmabot launch_panda_servers.launch`

---

## Phase 3 — PragmaBot Node Wiring (Stub Integration)

**Goal:** Wire `PandaSkillExecutor` stub into `pragmabot_node.py` so the full VLM planning loop runs end-to-end with stub execution. Validate STM, LTM, RAG all work before any real robot code exists.

**Do this phase without a robot.**

### 3.1 Find the NotImplementedError in `pragmabot_node.py`

```bash
grep -n "NotImplementedError\|execute_skill\|handle_planning" \
  /catkin_ws/src/pragmabot/pragmabot/nodes/pragmabot_node.py
```

### 3.2 Make the four surgical edits to `pragmabot_node.py`

**Edit 1** — Add import at top of file (after existing imports):
```python
from pragmabot.panda_skill_executor import PandaSkillExecutor
```

**Edit 2** — In the node `__init__` or wherever the node object is constructed, add:
```python
self.executor = PandaSkillExecutor()
```

**Edit 3** — Find the `NotImplementedError` line. It will look something like:
```python
raise NotImplementedError("Action execution not implemented")
```
Replace that block with:
```python
exec_result = self.executor.execute(next_action)
success = exec_result.get("success", False)
pre_image = exec_result.get("pre_image", None)
post_image = exec_result.get("post_image", None)
```

**Edit 4** — Ensure `pre_image` and `post_image` flow correctly into the success detector call that follows. Read the surrounding code carefully — do not change the success detector call signature, only ensure it receives the images from `exec_result`.

**Important:** Read the full `handle_planning_request` function (or equivalent) before making any edit. Understand what variables the success detector expects to receive. Only change the execution block — nothing else.

### 3.3 Verify the wiring compiles

```bash
cd /catkin_ws && catkin build pragmabot && source devel/setup.bash
python3 -c "
import sys; sys.path.insert(0, '/catkin_ws/devel/lib/python3/dist-packages')
import rospy
" 
```

### 3.4 Test in rosbag replay mode

Set in `config/config.yaml`:
```yaml
rosbag_replay: true
```

Then in separate terminals:
```bash
# Terminal 1
rosbag play -l /path/to/sample.bag

# Terminal 2
roslaunch pragmabot launch_panda_servers.launch

# Terminal 3 — set a fake API key to get past the check, or use real key
export OPENAI_API_KEY="your-key"
roslaunch pragmabot launch_pragmabot.launch
```

Open the Gradio URL, submit an instruction like `"put the apple on the plate"`.

**Expected behavior:**
- VLMSceneDescriber runs (calls OpenAI)
- RAG retrieves from LTM (empty at first = no retrieved experiences)
- VLMTaskPlanner returns a NextBestAction
- `executor.execute()` stub logs `[STUB] execute called: skill=pick ...` and returns `success=True`
- VLMSuccessDetector runs on before/after images from rosbag
- STM entry is appended
- Loop continues or task completes
- On completion, LTM entry is written

### Phase 3 done when:
- Full loop runs end-to-end in rosbag replay mode
- STM entries appear in logs
- LTM CSV is written after task completion
- No crashes or unexpected errors

---
Good — rosbag_replay is already true. Here's what we tested and what it means:

---
What we just verified

Import test — PandaSkillExecutor, ObjectDetector, GraspEstimator all import cleanly. No missing modules.

Instantiation test — PandaSkillExecutor() creates without error. No ROS master needed for construction.

Execute test — Called executor.execute() with a fake PICK action and dummy images. It ran _pick() (which logs STUB _pick: would pick 'apple') then _return_to_observation_pose() — no crash.

---
- real OpenAI API key → VLM actually planning actions
- Gradio UI → submit an instruction, watch the loop run
- STM entries appearing in logs
- LTM CSV written after task completion

That part we can't run right now because it needs a rosbag file and an OpenAI API key set. But the code path is:

rosbag_replay: true
→ handle_planning_request() calls VLM planner
→ executor.execute() logs STUB (no robot moves)
→ handle_evaluation_request() calls success detector on rosbag images
→ STM updated → loop continues


## Phase 4 — GroundedSAM Installation and Offline Test

**Goal:** Install GroundedSAM into the container, replace the stub, and validate it produces correct masks on saved ZED2 images.

**Do this phase without a robot — use saved images from rosbag.**

### 4.1 Install Grounding DINO

```bash
cd /tmp
git clone https://github.com/IDEA-Research/GroundingDINO.git
cd GroundingDINO
pip install -e . --break-system-packages
```

Download weights:
```bash
mkdir -p /catkin_ws/src/pragmabot/weights
cd /catkin_ws/src/pragmabot/weights
wget -q https://github.com/IDEA-Research/GroundingDINO/releases/download/v0.1.0-alpha/groundingdino_swint_ogc.pth
```

### 4.2 Install SAM

```bash
pip install segment-anything --break-system-packages
```

Download SAM ViT-H weights:
```bash
cd /catkin_ws/src/pragmabot/weights
wget -q https://dl.fbaipublicfiles.com/segment_anything/sam_vit_h_4b8939.pth
```

### 4.3 Verify imports

```bash
python3 -c "from groundingdino.util.inference import load_model, predict; print('groundingdino ok')"
python3 -c "from segment_anything import sam_model_registry, SamPredictor; print('sam ok')"
```

### 4.4 Write `src/pragmabot/grounded_sam.py` (full implementation)

Replace the stub created in Phase 2 with the full implementation:

```python
import numpy as np
import torch
import cv2
from groundingdino.util.inference import load_model, predict
from segment_anything import sam_model_registry, SamPredictor

GDINO_CONFIG  = "/tmp/GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py"
GDINO_WEIGHTS = "/catkin_ws/src/pragmabot/weights/groundingdino_swint_ogc.pth"
SAM_WEIGHTS   = "/catkin_ws/src/pragmabot/weights/sam_vit_h_4b8939.pth"
SAM_TYPE      = "vit_h"

class GroundedSAM:

    def __init__(self):
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.gdino = load_model(GDINO_CONFIG, GDINO_WEIGHTS)
        self.gdino.to(self.device)
        sam = sam_model_registry[SAM_TYPE](checkpoint=SAM_WEIGHTS)
        sam.to(self.device)
        self.predictor = SamPredictor(sam)

    def segment(
        self,
        image: np.ndarray,          # H×W×3 uint8, BGR (from ros_numpy / OpenCV)
        text: str,
        box_threshold: float = 0.3,
        text_threshold: float = 0.25,
    ) -> tuple:
        """
        Returns:
            mask       : H×W bool ndarray  (True = object pixel)
            confidence : float             (Grounding DINO detection score)
        Returns (zero mask, 0.0) if nothing detected above threshold.
        """
        rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        boxes, logits, _ = predict(
            model=self.gdino,
            image=rgb,
            caption=text,
            box_threshold=box_threshold,
            text_threshold=text_threshold,
            device=self.device,
        )

        if len(boxes) == 0:
            return np.zeros(image.shape[:2], dtype=bool), 0.0

        best_idx = int(logits.argmax())
        confidence = float(logits[best_idx])
        box = boxes[best_idx]                          # normalized [cx,cy,w,h]

        H, W = image.shape[:2]
        cx, cy, bw, bh = box.tolist()
        x1 = int((cx - bw / 2) * W)
        y1 = int((cy - bh / 2) * H)
        x2 = int((cx + bw / 2) * W)
        y2 = int((cy + bh / 2) * H)

        self.predictor.set_image(rgb)
        masks, _, _ = self.predictor.predict(
            box=np.array([x1, y1, x2, y2]),
            multimask_output=False,
        )
        return masks[0].astype(bool), confidence
```

### 4.5 Offline validation script

Create `/catkin_ws/src/pragmabot/scripts/test_grounded_sam.py`:

```python
#!/usr/bin/env python3
"""
Offline test: run GroundedSAM on a saved image.
Usage: python3 test_grounded_sam.py --image /path/to/image.png --text "red cup"
"""
import argparse
import cv2
import numpy as np
import sys
sys.path.insert(0, "/catkin_ws/src/pragmabot/pragmabot/src")

from pragmabot.grounded_sam import GroundedSAM

parser = argparse.ArgumentParser()
parser.add_argument("--image", required=True)
parser.add_argument("--text",  required=True)
args = parser.parse_args()

img = cv2.imread(args.image)
assert img is not None, f"Could not load image: {args.image}"

gsam = GroundedSAM()
mask, conf = gsam.segment(img, args.text)

print(f"Confidence: {conf:.3f}")
print(f"Mask pixels: {mask.sum()} / {mask.size} ({100*mask.mean():.1f}%)")

# Visualise: overlay mask in green
vis = img.copy()
vis[mask] = vis[mask] * 0.4 + np.array([0, 200, 0]) * 0.6
cv2.imwrite("/tmp/gsam_result.png", vis.astype(np.uint8))
print("Saved visualisation to /tmp/gsam_result.png")
```

Run it on a frame extracted from a rosbag:
```bash
python3 /catkin_ws/src/pragmabot/scripts/test_grounded_sam.py \
    --image /tmp/scene.png \
    --text "apple"
```

### Phase 4 done when:
- GroundedSAM imports cleanly
- Offline test on a tabletop image produces a mask with >0 pixels and confidence >0.3
- Visualisation confirms mask covers the correct object

---

## Phase 5 — GraspGen Installation and Offline Spike

**Goal:** Install GraspGen, validate it returns ranked 6-DOF grasp poses from a saved point cloud.

**Do this phase without a robot.**

### 5.1 Install GraspGen

```bash
cd /tmp
git clone https://github.com/NVlabs/GraspGen.git
cd GraspGen
pip install -e . --break-system-packages
```

### 5.2 Download Franka Panda pretrained model

Follow GraspGen README to download `franka_panda` checkpoint into their default weights directory.

### 5.3 Start GraspGen server

GraspGen supports a client-server mode. Start the server in a separate terminal:

```bash
cd /tmp/GraspGen
python scripts/grasp_server.py --gripper franka_panda --port 8080
```

### 5.4 Write `src/pragmabot/graspgen_client.py` (full implementation)

Replace the Phase 2 stub:

```python
import numpy as np
import requests
import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_matrix


class GraspGenClient:

    def __init__(self, url: str = "http://localhost:8080/grasp"):
        self.url = url

    def generate(
        self,
        point_cloud: np.ndarray,    # Nx3 float32, panda_link0 frame
        gripper: str = "franka_panda",
        num_grasps: int = 20,
    ) -> list:
        """
        Returns list of {"pose": PoseStamped, "score": float}
        sorted descending by score. Frame = panda_link0.
        """
        payload = {
            "points":     point_cloud.tolist(),
            "gripper":    gripper,
            "num_grasps": num_grasps,
        }
        resp = requests.post(self.url, json=payload, timeout=15.0)
        resp.raise_for_status()
        data = resp.json()

        results = []
        for g in data["grasps"]:
            T = np.array(g["pose"])            # 4×4 homogeneous

            ps = PoseStamped()
            ps.header.frame_id = "panda_link0"
            ps.header.stamp = rospy.Time.now()
            ps.pose.position.x = T[0, 3]
            ps.pose.position.y = T[1, 3]
            ps.pose.position.z = T[2, 3]

            q = quaternion_from_matrix(T)      # [x, y, z, w]
            ps.pose.orientation.x = q[0]
            ps.pose.orientation.y = q[1]
            ps.pose.orientation.z = q[2]
            ps.pose.orientation.w = q[3]

            results.append({"pose": ps, "score": float(g["score"])})

        results.sort(key=lambda x: x["score"], reverse=True)
        return results
```

### 5.5 Offline validation script

Create `/catkin_ws/src/pragmabot/scripts/test_graspgen.py`:

```python
#!/usr/bin/env python3
"""
Offline test: send a saved point cloud to GraspGen server, print returned poses.
Usage: python3 test_graspgen.py --pcd /path/to/points.npy
"""
import argparse
import numpy as np
import sys
sys.path.insert(0, "/catkin_ws/src/pragmabot/pragmabot/src")

# Minimal ROS init needed for rospy.Time.now() inside graspgen_client
import rospy
rospy.init_node("test_graspgen", anonymous=True)

from pragmabot.graspgen_client import GraspGenClient

parser = argparse.ArgumentParser()
parser.add_argument("--pcd", required=True, help="Path to Nx3 float32 .npy point cloud")
args = parser.parse_args()

pts = np.load(args.pcd).astype(np.float32)
print(f"Point cloud shape: {pts.shape}")

client = GraspGenClient()
grasps = client.generate(pts, gripper="franka_panda", num_grasps=10)

print(f"Returned {len(grasps)} grasps")
for i, g in enumerate(grasps[:5]):
    p = g["pose"].pose.position
    print(f"  [{i}] score={g['score']:.3f}  pos=({p.x:.3f}, {p.y:.3f}, {p.z:.3f})")
```

To extract a test point cloud from a rosbag depth image, run:
```bash
python3 /catkin_ws/src/pragmabot/scripts/extract_pointcloud_from_bag.py \
    --bag /path/to/sample.bag \
    --mask /tmp/gsam_mask.npy \
    --out /tmp/test_pointcloud.npy
```

(Write this extraction helper script as part of this phase — it uses ros_numpy + the same back-projection logic as `_mask_to_pointcloud` in the pick server.)

### Phase 5 done when:
- GraspGen server starts without error
- Client sends a point cloud and receives ≥1 grasp pose back
- Top pose has a score >0 and position values that are plausible for a tabletop scene (Z between 0.1–0.8m)

---

## Phase 6 — Full Action Server Implementations

**Goal:** Replace all three stub servers with full implementations. Requires robot access for final validation, but the code can be written and statically verified beforehand.

### 6.1 Write `nodes/panda_pick_server.py` (full implementation)

Implement the full `PandaPickServer` class. The execution pipeline inside `execute_cb` is:

```
open_gripper()
move_to_observation_pose()        # MoveIt to OBSERVATION_JOINT_CONFIG
capture_rgbd()                    # wait for synchronized ZED2 RGB + depth
grounded_sam.segment(rgb, target) # → binary mask + confidence
mask_to_pointcloud(mask, depth)   # → Nx3 in panda_link0
graspgen.generate(pointcloud)     # → ranked grasp poses
filter_ik(grasp_poses[:8])        # → feasible subset with trajectories
arm.execute(best_trajectory)      # MoveIt motion execution
close_gripper()                   # franka_gripper GraspAction
lift(+0.10m in Z)                 # post-grasp lift
set_succeeded / set_aborted
```

Key constants to define at file top — fill in from physical robot measurement:
```python
OBSERVATION_JOINT_CONFIG = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]  # REPLACE with measured values
GRIPPER_MAX_WIDTH = 0.08   # metres — Franka Hand max opening
GRIPPER_GRASP_FORCE = 20.0 # Newtons — adjust per object fragility
LIFT_HEIGHT = 0.10         # metres
```

Implement helpers as class methods:
- `_open_gripper(self)`
- `_close_gripper(self, width, force)`
- `_move_to_observation_pose(self)`
- `_capture_rgbd(self) -> (rgb_msg, depth_msg)`
- `_mask_to_pointcloud(self, mask, depth_msg) -> np.ndarray`
- `_filter_ik(self, pose_list) -> list[(PoseStamped, trajectory)]`
- `_send_feedback(self, text)`

Wrap the entire `execute_cb` body in `try/except RuntimeError` → `set_aborted`, `except Exception` → `set_aborted`.

### 6.2 Write `nodes/panda_place_server.py` (full implementation)

Execution pipeline inside `execute_cb`:

```
move_to_observation_pose()
capture_rgbd()
grounded_sam.segment(rgb, target_location)   # segment the receptacle
extract_centroid_2d(mask)                    # pixel centroid of receptacle
depth_at_centroid → 3D point in panda_link0  # back-project single point
construct_place_pose(3d_point)               # above centroid + fixed approach orientation
arm.plan_and_execute(approach_pose)          # move above place location
descend(0.05m in -Z)                         # lower to place height
open_gripper()
retreat(0.10m in +Z)
set_succeeded / set_aborted
```

### 6.3 Write `nodes/panda_push_server.py` (full implementation)

Execution pipeline inside `execute_cb`:

```
move_to_observation_pose()
capture_rgbd()
gsam.segment(rgb, target_object)   → object_centroid_3d
gsam.segment(rgb, goal_region)     → goal_centroid_3d
push_vector = normalize(goal - object)
approach_point = object_centroid - push_vector * 0.05  # 5cm behind object
arm.plan_and_execute(approach_point, orientation=horizontal)
execute_cartesian_path(waypoints=[object_centroid, goal_centroid])
retreat(+Z)
set_succeeded / set_aborted
```

### 6.4 Write full `src/pragmabot/panda_skill_executor.py`

```python
import rospy
import actionlib
from pragmabot.msg import (
    PandaPickAction,  PandaPickGoal,
    PandaPlaceAction, PandaPlaceGoal,
    PandaPushAction,  PandaPushGoal,
)

ACTION_TIMEOUT = 60.0  # seconds per skill

class PandaSkillExecutor:

    def __init__(self):
        self.pick_client  = actionlib.SimpleActionClient("/panda/pick",  PandaPickAction)
        self.place_client = actionlib.SimpleActionClient("/panda/place", PandaPlaceAction)
        self.push_client  = actionlib.SimpleActionClient("/panda/push",  PandaPushAction)

        rospy.loginfo("Waiting for action servers...")
        self.pick_client.wait_for_server(timeout=rospy.Duration(30.0))
        self.place_client.wait_for_server(timeout=rospy.Duration(30.0))
        self.push_client.wait_for_server(timeout=rospy.Duration(30.0))
        rospy.loginfo("All action servers connected")

    def execute(self, action) -> dict:
        """
        action: NextBestAction Pydantic object from VLMTaskPlanner
        Returns: {"success": bool, "pre_image": ..., "post_image": ..., "message": str}
        """
        skill = action.skill.lower()

        if skill == "pick":
            return self._execute_pick(action)
        elif skill == "place":
            return self._execute_place(action)
        elif skill == "push":
            return self._execute_push(action)
        elif skill == "done":
            return {"success": True, "pre_image": None, "post_image": None, "message": "task marked done"}
        else:
            return {"success": False, "pre_image": None, "post_image": None,
                    "message": f"unknown skill: {skill}"}

    def _execute_pick(self, action) -> dict:
        goal = PandaPickGoal(
            target_object=action.target_object,
            use_annotation=getattr(action, "use_annotation", False),
        )
        self.pick_client.send_goal(goal, feedback_cb=self._feedback_cb)
        finished = self.pick_client.wait_for_result(rospy.Duration(ACTION_TIMEOUT))

        if not finished:
            self.pick_client.cancel_goal()
            return {"success": False, "pre_image": None, "post_image": None,
                    "message": "pick timed out"}

        result = self.pick_client.get_result()
        return {"success": result.success, "pre_image": None, "post_image": None,
                "message": result.message}

    def _execute_place(self, action) -> dict:
        from geometry_msgs.msg import PoseStamped
        goal = PandaPlaceGoal(
            target_location=action.target_location or "",
            place_pose=PoseStamped(),   # server resolves pose via GroundedSAM
        )
        self.place_client.send_goal(goal, feedback_cb=self._feedback_cb)
        finished = self.place_client.wait_for_result(rospy.Duration(ACTION_TIMEOUT))

        if not finished:
            self.place_client.cancel_goal()
            return {"success": False, "pre_image": None, "post_image": None,
                    "message": "place timed out"}

        result = self.place_client.get_result()
        return {"success": result.success, "pre_image": None, "post_image": None,
                "message": result.message}

    def _execute_push(self, action) -> dict:
        goal = PandaPushGoal(
            target_object=action.target_object,
            goal_region=action.target_location or "",
        )
        self.push_client.send_goal(goal, feedback_cb=self._feedback_cb)
        finished = self.push_client.wait_for_result(rospy.Duration(ACTION_TIMEOUT))

        if not finished:
            self.push_client.cancel_goal()
            return {"success": False, "pre_image": None, "post_image": None,
                    "message": "push timed out"}

        result = self.push_client.get_result()
        return {"success": result.success, "pre_image": None, "post_image": None,
                "message": result.message}

    @staticmethod
    def _feedback_cb(fb):
        rospy.loginfo(f"[skill] {fb.status}")
```

### 6.5 Build and import-verify

```bash
cd /catkin_ws && catkin build pragmabot && source devel/setup.bash
python3 -c "from pragmabot.panda_skill_executor import PandaSkillExecutor; print('executor ok')"
```

### Phase 6 done when:
- All three action server files are fully implemented (not stubs)
- `catkin build` passes
- Import check passes
- With robot: launch servers + send a test goal via `rostopic pub` and confirm feedback is published and result is returned

---

## Phase 7 — Physical Robot Validation

**Requires lab access.**

### 7.1 Measure and record observation pose

With the Panda powered on and FCI active:
1. Manually jog the arm to a configuration where the entire arm is out of the ZED2 field of view
2. Read joint values: `rostopic echo /joint_states`
3. Record all 7 values
4. Replace the placeholder `OBSERVATION_JOINT_CONFIG` in `panda_pick_server.py` with the measured values

### 7.2 ZED2 static TF calibration

Calibrate extrinsic: `zed2_left_camera_frame` → `panda_link0`

Create `launch/static_zed2_tf.launch`:
```xml
<launch>
  <node pkg="tf2_ros" type="static_transform_publisher" name="zed2_to_panda"
        args="TX TY TZ QX QY QZ QW zed2_left_camera_frame panda_link0" />
</launch>
```

Replace TX TY TZ QX QY QZ QW with calibrated values.

Test calibration:
```bash
rosrun tf tf_echo panda_link0 zed2_left_camera_frame
```

### 7.3 Motion basics test

Write `/catkin_ws/src/pragmabot/scripts/test_motion_basics.py`:

```python
#!/usr/bin/env python3
"""Manual motion test: home → observation pose → open gripper → close gripper"""
import rospy
import actionlib
import moveit_commander
from franka_gripper.msg import MoveAction, MoveGoal, GraspAction, GraspGoal

OBSERVATION_JOINT_CONFIG = [...]  # fill in from 7.1

rospy.init_node("test_motion_basics")
arm = moveit_commander.MoveGroupCommander("panda_arm")

print("Moving to observation pose...")
arm.set_joint_value_target(OBSERVATION_JOINT_CONFIG)
arm.go(wait=True)
arm.stop()
print("Done")

print("Opening gripper...")
move_client = actionlib.SimpleActionClient("/franka_gripper/move", MoveAction)
move_client.wait_for_server()
move_client.send_goal_and_wait(MoveGoal(width=0.08, speed=0.05))
print("Done")

print("Closing gripper...")
grasp_client = actionlib.SimpleActionClient("/franka_gripper/grasp", GraspAction)
grasp_client.wait_for_server()
goal = GraspGoal(width=0.0, speed=0.03, force=10.0)
goal.epsilon.inner = 0.01
goal.epsilon.outer = 0.01
grasp_client.send_goal_and_wait(goal)
print("Done — all motion basics passed")
```

### 7.4 End-to-end pick test

With GraspGen server running, launch pick server and test against a real object:

```bash
# Terminal 1: GraspGen server
cd /tmp/GraspGen && python scripts/grasp_server.py --gripper franka_panda --port 8080

# Terminal 2: ROS + MoveIt
roslaunch panda_moveit_config franka_control.launch robot_ip:=<IP>

# Terminal 3: pick server
roslaunch pragmabot launch_panda_servers.launch

# Terminal 4: send a test goal
rostopic pub -1 /panda/pick/goal pragmabot/PandaPickActionGoal \
  "{ header: { stamp: now }, goal_id: { stamp: now, id: 'test_1' }, goal: { target_object: 'apple', use_annotation: false } }"
```

Watch pick server logs for feedback stages. Confirm:
- Arm moves to observation pose
- ZED2 image captured
- GroundedSAM produces a mask
- GraspGen returns poses
- MoveIt plans successfully
- Arm executes grasp
- Gripper closes
- Lift succeeds
- Result published: `success: true`

### 7.5 Full end-to-end PragmaBot test

```bash
# Set rosbag_replay: false in config.yaml
export OPENAI_API_KEY="your-paid-key"
roslaunch pragmabot launch_panda_servers.launch
roslaunch pragmabot launch_pragmabot.launch
```

Open Gradio URL. Submit: `"put the apple on the plate"`.

Validate full loop:
- Scene described by GPT-4o
- Action planned
- Pick server executes real pick
- Success detector evaluates before/after
- STM updated
- Loop continues until done
- LTM entry written

---

## File Creation Checklist

Track which files exist vs. still needed:

| File | Phase created | Status |
|------|--------------|--------|
| `action/PandaPick.action` | 2 | |
| `action/PandaPlace.action` | 2 | |
| `action/PandaPush.action` | 2 | |
| `src/pragmabot/grounded_sam.py` | 2 (stub) → 4 (full) | |
| `src/pragmabot/graspgen_client.py` | 2 (stub) → 5 (full) | |
| `src/pragmabot/panda_skill_executor.py` | 2 (stub) → 6 (full) | |
| `nodes/panda_pick_server.py` | 2 (stub) → 6 (full) | |
| `nodes/panda_place_server.py` | 2 (stub) → 6 (full) | |
| `nodes/panda_push_server.py` | 2 (stub) → 6 (full) | |
| `launch/launch_panda_servers.launch` | 2 | |
| `launch/static_zed2_tf.launch` | 7 | |
| `scripts/test_grounded_sam.py` | 4 | |
| `scripts/test_graspgen.py` | 5 | |
| `scripts/test_motion_basics.py` | 7 | |
| `nodes/pragmabot_node.py` (4 edits) | 3 | |

---

## Constraints Claude Code Must Respect

1. **Never modify** `vlm_client.py`, `vlm_task_planner.py`, `vlm_success_detector.py`, `vlm_scene_describer.py`, `vlm_exp_summarizer.py`, `memory_manager.py`, `scene_observer.py`. These are upstream PragmaBot code — untouched.

2. **Only four edits** to `pragmabot_node.py`: import, executor init, replace NotImplementedError, wire pre/post images.

3. **Python 3.8 only** — no walrus operator, no `match`, no 3.10+ syntax.

4. **ROS1 Noetic** — use `rospy`, `actionlib`, `moveit_commander`. Not ROS2.

5. **All pip installs** must use `--break-system-packages` flag in this container.

6. **Catkin build must pass** after every phase before moving to the next.

7. **Stubs before full implementations** — Phase 2 creates stubs so the build graph is always consistent. Full implementations replace stubs in later phases.

8. When `roslaunch pragmabot launch_pragmabot.launch` fails with an error other than `OPENAI_API_KEY` — stop and report it. Do not attempt to fix upstream PragmaBot code.
