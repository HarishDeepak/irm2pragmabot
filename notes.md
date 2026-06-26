# PragmaBot × Panda — Project Notes

Running log of understanding, design decisions, and insights built up during development.

---

## Project Overview

PragmaBot is a **VLM-driven robotic task learning system** from ETH Zurich RSL (IEEE RAL 2026), forked and adapted for the Franka Panda at PEARL Lab, TU Darmstadt (IRM2 project).

The core idea: instead of hard-coded task logic, the robot uses a large vision-language model (VLM) to observe a scene, plan which manipulation skill to use, execute it, check if it worked, and remember the experience for next time. No task-specific code — the VLM generalizes.

---

## Architecture Understanding

### The 7-Step Pipeline (runs once per planning step)

```
1. VLMSceneDescriber   → GPT/Claude describes what it sees in the camera image
2. MemoryManager       → cosine-similarity lookup over past experience embeddings (LTM)
3. VLMTaskPlanner      → picks: PICK / PLACE / PUSH + which object/location
4. Action Execution    → PandaSkillExecutor → action server → MoveIt → robot
5. VLMSuccessDetector  → compares before/after images, returns {success, task_done}
6. STM update          → appends (action, evaluation) to short-term memory; loops if failed
7. VLMExperienceSummarizer → on task done, distills STM episode into compact LTM entry
```

Steps 3–6 loop until the task is marked complete or the user resets.

### Key Design: Upstream Code is Untouched

The ETH Zurich VLM pipeline files (`vlm_client.py`, `vlm_task_planner.py`, `vlm_scene_describer.py`, `vlm_success_detector.py`, `vlm_exp_summarizer.py`, `memory_manager.py`, `scene_observer.py`) are **read-only**. All Panda-specific work is added in new files or in the 4 surgical edits to `pragmabot_node.py`.

### VLMClient Interface

Everything downstream consumes this interface (duck typing — not an ABC):

```python
class VLMClient:
    def query_structured(builder, response_format) -> (parsed_obj, elapsed_sec, prompt_tokens)
    def get_text_embedding(text, builder=None) -> (List[List[float]], elapsed_sec, tokens)
```

`ConversationBuilder` accumulates messages in **OpenAI API format** (image_url with base64 data-URIs). All VLM client forks must convert from this format.

---

## VLM Client Forks

### Why Multiple Backends?

- Supervisor provides Anthropic API access (Claude)
- User has Gemini Pro subscription
- OpenAI path remains for completeness / reproducibility

### OpenAI (original): `vlm_client.py`
- `chat.completions.parse()` → native structured output via Pydantic
- Embeddings: `embeddings.create()` → `text-embedding-3-large`
- Image format: `image_url` with `data:image/jpeg;base64,...`

### Claude: `claude_vlm_client.py`
- `client.messages.parse(output_format=PydanticModel, thinking={"type":"adaptive"})` → structured output
- Adaptive thinking enabled by default (supervisor specifically requested chain-of-thought reasoning for NextBestAction)
- **No embeddings API** → falls back to `sentence-transformers` (`all-MiniLM-L6-v2`) running locally
  - Embedding dimension: 384 (vs 3072 for OpenAI). This means existing LTM CSVs from OpenAI embeddings are **incompatible** — must rebuild LTM when switching backends.
- Image format must be converted: `image_url` → `{"type":"image","source":{"type":"base64",...}}`
- System prompt is a separate parameter (not a message role)

### Gemini: `gemini_vlm_client.py`
- `genai.GenerativeModel(model_name=..., system_instruction=...)` 
- Structured output: `GenerationConfig(response_mime_type="application/json", response_schema=PydanticModel)`
  - Response is JSON text → manually parse with `response_format.model_validate_json(response.text)`
- Embeddings: `genai.embed_content(model="models/text-embedding-004", content=text)` — native, no fallback needed
- Image format: `inline_data` with `mime_type` and raw base64 data (no `data:...` prefix)
- Gemini role for assistant is `"model"` not `"assistant"`

### Message Format Conversion (OpenAI → Anthropic/Gemini)

```
OpenAI system:  {"role": "system", "content": "..."}
  → Claude:   extracted to top-level `system=` parameter
  → Gemini:   extracted to `system_instruction=` on GenerativeModel

OpenAI image:  {"type": "image_url", "image_url": {"url": "data:image/jpeg;base64,AAA..."}}
  → Claude:   {"type": "image", "source": {"type": "base64", "media_type": "image/jpeg", "data": "AAA..."}}
  → Gemini:   {"inline_data": {"mime_type": "image/jpeg", "data": "AAA..."}}
```

### Switching Backends

In `config.yaml`:
```yaml
vlm:
  vlm_model: claude-opus-4-8    # or gpt-4o-2024-08-06 or gemini-2.5-pro
```

`pragmabot_node.py` detects "gpt" / "claude" / "gemini" in the model name string and imports the right client.

---

## Panda Adaptation (Phases 1–6 Done)

### Action Flow

```
PandaSkillExecutor.execute(NextBestAction)
  → sends ROS action goal to /panda/{pick,place,push}
  ← result: {success, message}
```

### Pick Server (panda_pick_server.py)
1. Move arm to observation pose (out of camera view)
2. Get RGB-D image from ZED2
3. Run GroundedSAM to get object mask
4. Convert mask pixels to 3D point cloud (using camera intrinsics + depth)
5. Send point cloud to GraspGen server (ZMQ, host:5556) → get grasp pose
6. Execute grasp with MoveIt

### Place Server (panda_place_server.py)
1. GroundedSAM → mask of target location
2. Centroid of mask → 3D pose (using camera intrinsics + depth)
3. Move to pose with MoveIt, open gripper

### Push Server (panda_push_server.py)
1. GroundedSAM → object mask → 3D centroid (push start)
2. GroundedSAM → goal region mask → 3D centroid (push end)
3. Cartesian push trajectory with MoveIt

### GroundedSAM
- GroundingDINO for open-vocabulary detection (text → bounding box)
- SAM (ViT-H) for mask segmentation from bounding box prompt
- **Lives in `/tmp/GroundingDINO`** — lost on container restart, reinstall needed
- Weights persist in `/catkin_ws/src/pragmabot/weights/` (volume-mounted)
- Runs on CPU (CUDA extension fails to compile on Python 3.8 + old CUDA)

### GraspGen
- NVlabs GraspGen: SE(3) grasp generation from point clouds
- **Runs on HOST machine** (needs Python 3.10 + CUDA)
- Container communicates via ZMQ on port 5556 (host at `172.17.0.1:5556`)
- Protocol: MsgPack-serialized request `{points: np.array, model_cfg: str}` → response `{grasp_pose: dict, score: float}`

---

## Memory System

### Short-Term Memory (STM)
- Per-episode list of `{time_step, action}` and `{time_step, evaluation}` entries
- JSON-formatted, injected into VLMTaskPlanner prompt as context
- Enables re-planning with awareness of what was already tried

### Long-Term Memory (LTM)
- Two CSVs merged on `scenario` key:
  - `ltm.csv`: human-readable experience summaries
  - `ltm_<model>.csv`: base64-encoded text embeddings
- Scenario key: `"Instruction: {task}\nScene: {initial_scene_description}"`
- Retrieval: cosine similarity on text embeddings → top-k most relevant past experiences

### Embedding Dimension Mismatch Warning
Switching VLM backends changes embedding dimensions:
- OpenAI `text-embedding-3-large`: 3072D
- Claude fallback `all-MiniLM-L6-v2`: 384D
- Gemini `text-embedding-004`: 768D

Cosine similarity still works within a backend but **LTM from one backend cannot be queried by another**. If you switch backends, either delete LTM CSVs or keep them separate.

---

## Lab Setup Remaining (Phase 7)

1. **Observation joint config**: jog arm out of camera view, `rostopic echo /joint_states`, replace placeholder `[0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]` in all 3 servers
2. **Camera intrinsics**: replace `fx=fy=700` placeholder with real ZED2 values from `/zed2/zed_node/rgb/camera_info` (`K[0]`, `K[4]`, `K[2]`, `K[5]`)
3. **ZED2 → panda_link0 TF**: create `launch/static_zed2_tf.launch` with calibrated transform
4. **GraspGen server on host**: `python graspgen_server.py --gripper_config graspgen_franka_panda.yml --port 5556`

---

## Key Files Quick Reference

| File | Role |
|------|------|
| `nodes/pragmabot_node.py` | Main node; Gradio UI; 7-step loop orchestrator |
| `nodes/panda_pick_server.py` | ROS action server for PICK |
| `nodes/panda_place_server.py` | ROS action server for PLACE |
| `nodes/panda_push_server.py` | ROS action server for PUSH |
| `src/pragmabot/panda_skill_executor.py` | ROS action client bridge |
| `src/pragmabot/vlm_client.py` | Original OpenAI VLM client (read-only) |
| `src/pragmabot/claude_vlm_client.py` | Anthropic Claude fork |
| `src/pragmabot/gemini_vlm_client.py` | Google Gemini fork |
| `src/pragmabot/grounded_sam.py` | GroundingDINO + SAM perception |
| `src/pragmabot/graspgen_client.py` | ZMQ client for GraspGen on host |
| `src/pragmabot/conversation_builder.py` | Dual-log message builder (upstream, read-only) |
| `config/config.yaml` | All runtime config including `vlm.vlm_model` |

---

## Concepts to Keep in Mind

### VLM Prompt Structure (all components use same 3-layer pattern)
1. **System prompt**: role, capabilities, output schema description
2. **Task prompt**: user instruction + current observation image
3. **Instruction prompt**: reasoning rules, hard constraints, STM/LTM injected here

### Action Skills & Constraints (enforced in VLM prompts)
- **PICK**: target must be graspable (not flat/tiny), gripper must be empty
- **PLACE**: must be holding object first
- **PUSH**: only for table-top objects; use when object is too flat/small to pick

### Rosbag Replay Mode (`rosbag_replay: true`)
Bypasses real action execution — success detector is called directly after planning. Used for offline testing of the VLM reasoning + memory loop without needing the robot or action servers running.

### Gradio UI Flow
- User types instruction → `get_new_task()` → `handle_planning_request()`
- Timer polls `update_chat()` every 2 seconds to stream new conversation log entries
- The VLM pipeline runs in the main Gradio thread (blocking), updates are visible incrementally via the timer

---

## Common Gotchas

- `catkin build` fails due to build-space conflict — always use `catkin_make --pkg pragmabot`
- GroundingDINO must be reinstalled after container restart (lives in `/tmp`)
- `sentence-transformers` needs to be pip-installed separately for Claude backend
- LTM embeddings are backend-specific — don't mix CSVs from different embedding models
- Gemini assistant role is `"model"` not `"assistant"` in the API
- Claude system prompt is a top-level parameter, not a message in the `messages` list
- `client.messages.parse()` needs `max_tokens` set explicitly (no default)
