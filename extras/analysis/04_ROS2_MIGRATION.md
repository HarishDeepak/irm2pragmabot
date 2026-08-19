# ROS 2 Migration — Findings and Plan

**Question answered:** is anything ROS 1, and do we need to port everything?
**Answer:** one package is ROS 1. Port ~515 lines, delete ~648, leave ~1,400 untouched.
**Date:** 2026-08-19. All claims from files on disk, not git history.

---

## 1. Where ROS 1 lives

Exactly one package: **`D:/pb/home/pragmabot/pragmabot/`** (upstream PragmaBot, catkin).

Evidence:
- 10 files import `rospy` / `actionlib`
- `CMakeLists.txt` + `package.xml` are catkin
- 6 XML `.launch` files
- Upstream targets Ubuntu 20.04 + ROS Noetic

**Everything else is ROS 2.** 36 `.launch.py` files vs 6 `.launch`. ROS 2 confirmed in: `franka_ros2` (Humble), `zed_ros2_ws`, `pragmabot_bridge`, `pragmabot_interfaces`, `easy_handeye2`, `calibration/*.py`, `bags/extract_bag.py`, `GraspGen/scripts/capture_zed_frame.py`.

---

## 2. The coupling is shallow

Of **17** library modules in `pragmabot/src/pragmabot/`, only **3** import ROS.

### KEEP — zero changes (~1,400 lines)

| Module | Lines |
|---|---|
| `memory_manager.py` | 308 |
| `vlm_task_planner.py` | 206 |
| `vlm_success_detector.py` | 130 |
| `conversation_builder.py` | 129 |
| `gemini_vlm_client.py` | 125 |
| `claude_vlm_client.py` | 120 |
| `vlm_exp_summarizer.py` | 100 |
| `vlm_scene_describer.py` | 95 |
| `vlm_client.py` | 93 |
| `geometry.py` | 85 |
| `grounded_sam.py` | 76 |
| `utils.py` | 67 |
| `simple_config.py` | 29 |
| `__init__.py` | 1 |

**This is the entire paper contribution — planner, memory, RAG, success detection, summarisation — and it is framework-free Python.** It runs unchanged under ROS 2.

> This satisfies the project's "never modify upstream VLM modules" rule **by construction**, not by exception. That is a strong argument for porting rather than bridging.

### PORT — real work (515 lines)

| File | Lines | Change |
|---|---|---|
| `nodes/pragmabot_node.py` | 423 | `rospy` → `rclpy`; keep the Gradio UI |
| `src/pragmabot/scene_observer.py` | 92 | camera subscribe + time sync → `rclpy` |

### DELETE — do not port (648 lines)

| File | Lines | Why |
|---|---|---|
| `nodes/panda_pick_server.py` | 188 | ROS 1 action server, superseded by `pragmabot_bridge` |
| `nodes/panda_push_server.py` | 194 | same |
| `nodes/panda_place_server.py` | 171 | same |
| `src/pragmabot/graspgen_client.py` | 95 | standalone GraspGen client already covers it |

### FIX — rewrite (93 lines)

`src/pragmabot/panda_skill_executor.py` → `rclpy` `ActionClient`.

**It also carries a live bug** (see §4).

### OPTIONAL — defer

| File | Lines | Note |
|---|---|---|
| `nodes/memory_manager_node.py` | 296 | separate Gradio UI for LTM inspection; useful, not required |
| `nodes/image_republisher_node.py` | 121 | rosbag replay helper; ROS 2 bag replay may remove the need |
| `nodes/image_decompressor_node.py` | 57 | same |

---

## 3. Why port instead of bridge

The earlier recommendation in `02_REPRODUCTION_PLAN.md` was a ZMQ bridge. **That was wrong** — it assumed a large port. Revised:

1. **`ros1_bridge`** — needs a second ROS distro installed, a bridge process kept alive, and message-type mappings maintained. More moving parts than the port itself.
2. **ZMQ between planner and executor** — you would hand-roll goals, feedback, cancellation, and introspection. ROS 2 actions give all of that free, plus `ros2 action list/send_goal` for debugging.
3. **One `colcon` workspace** builds `pragmabot`, `pragmabot_bridge`, `pragmabot_interfaces` together. This **removes the rsync workaround** currently forced by `~/pragmabot` being mounted at `/pragmabot`, outside the colcon build path (`docker-compose.yml:17` vs `:21`).
4. **Removes the duplicate-copy hazard.** `pragmabot_bridge` and `pragmabot_interfaces` exist in two places today (verified byte-identical, but one missed rsync away from silent divergence).
5. **`ROS_DOMAIN_ID=7` already unifies discovery** across container and host — no extra transport needed.

**ZMQ stays for GraspGen.** That boundary is real: incompatible CUDA env (`torch==2.1.0` vs `>=2.3.1`), and it ships its own server.

---

## 4. Bug to fix during the rewrite

`panda_skill_executor.py` reads fields that do not exist on the planner's output.

| Line | Code | Reality |
|---|---|---|
| `:30` | `action.skill.lower()` | **`AttributeError`** — field is `chosen_skill` (`vlm_task_planner.py:107`) |
| `:38` | `elif skill == "done"` | dead branch — `RobotSkill` has only push/pick/place (`:73-79`) |
| `:63`, `:79` | `getattr(action, "target_location", "")` | **silently `""`** — field is `placement_object` (`:116`) |

Never caught because `config.yaml` sets `rosbag_replay: true`, and `pragmabot_node.py:139-141` skips the executor entirely. **That code path has never executed.**

**`PROJECT_OVERVIEW.md` §6 documents the wrong schema.** Do not write the new executor against it.

### The real schema to code against

```python
class RobotSkill(str, Enum):                 # vlm_task_planner.py:73-79
    PUSH = "push"; PICK = "pick"; PLACE = "place"

class NextBestAction(BaseModel):             # :88-128
    scene_description: str
    applicable_knowledge: Optional[str]      # RAG integration point
    chain_of_thought_reasoning: str
    chosen_action: str                       # text the success detector grades
    chosen_skill: RobotSkill
    target_object: str
    should_grasp_at_specific_section: Optional[bool]
    placement_object: Optional[str]          # "place ON, not next to"
    should_place_at_specific_section: Optional[bool]
    push_direction: Optional[PushDirection]  # LEFT | RIGHT
```

---

## 5. The action interface to create

`pragmabot_interfaces/CMakeLists.txt:16-22` has `rosidl_generate_interfaces` commented out with the open decision recorded. **Resolve it as one collapsed action**, because the three skills share a dispatch path in `bridge_node.py` and the planner emits a single `chosen_skill` enum.

```
# pragmabot_interfaces/action/ExecuteSkill.action
string  chosen_skill                        # "pick" | "place" | "push"
string  target_object
string  placement_object                    # "" when not a place
string  push_direction                      # "" when not a push
bool    should_grasp_at_specific_section
bool    should_place_at_specific_section
---
bool    success
string  message
---
string  status                              # feedback
```

**Done when:** `ros2 interface show pragmabot_interfaces/action/ExecuteSkill` prints all six request fields.

---

## 6. Ordered steps

| # | Step | Time | Done when |
|---|---|---|---|
| 1 | Delete the 4 dead files (§2 DELETE) | 10 min | `grep -r actionlib` returns nothing |
| 2 | Enable `pragmabot_interfaces` + add `ExecuteSkill.action` | 1 h | `ros2 interface show` works |
| 3 | Port `scene_observer.py` to `rclpy` | 1–2 h | subscribes to `/zed/zed_node/...`, returns synced RGB+depth+intrinsics |
| 4 | Port `pragmabot_node.py` to `rclpy` | 0.5–1 day | `ros2 run pragmabot pragmabot_node` starts, Gradio loads |
| 5 | Rewrite executor as `ActionClient` + add server in `bridge_node.py` | 0.5 day | `ros2 action send_goal` round-trips all six fields |

**Total ≈ 2 days.**

**Verification that the port stayed clean:** `diff` the 14 KEEP modules against their originals — expect **zero** changes.

---

## 7. Also fix while you are in there

1. **`config.yaml` topics** point at `/zedxm/zed_node/...` (upstream ANYmal camera). Yours is `/zed/zed_node/...`. Blocks the planner from seeing anything.
2. **Missing from `requirements.txt`** but imported: `pyzmq`, `msgpack`, `msgpack-numpy`, `sentence-transformers`, `anthropic`, `google-generativeai`.
3. **`.env~` is staged** in `franka_ros2` (empty file). Unstage it: `git restore --staged .env~ && rm .env~`.

---

## 8. What is already good — do not touch

1. `bridge_node.py` (511) — already ROS 2, singularity guard + gripper fault recovery
2. `grasp_transform.py` (160) — already ROS 2, math verified correct against real artifacts
3. `mask_to_pointcloud.py` (326) — no ROS at all, your strongest original work
4. `detect_object.py` (192) — no ROS, runs in its own venv
5. The 14 KEEP modules above
