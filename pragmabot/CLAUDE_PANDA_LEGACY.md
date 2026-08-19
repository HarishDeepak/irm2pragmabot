# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Commands

**Build:**
```bash
catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo -DPYTHON_EXECUTABLE=$(which python3)
catkin build pragmabot
```

**Run:**
```bash
roslaunch pragmabot launch_pragmabot.launch          # Main UI + pipeline
roslaunch pragmabot manage_memory.launch             # LTM inspection UI
roslaunch pragmabot replay_rosbag.launch bag_path:=/path/to/your.bag  # Replay without robot
```

**Install Python dependencies:**
```bash
pip install -r requirements.txt
```

**Required environment variable:**
```bash
export OPENAI_API_KEY="your-key"
```

There are no automated tests — this is an application/research codebase.

## Architecture

PragmaBot is a **VLM-driven robotic task learning system** built on ROS Noetic. It learns manipulation tasks by observing scenes, recovering from failures via short-term memory (STM), and accumulating long-term memory (LTM) across episodes.

### 7-Step Pipeline (per task)

1. **VLMSceneDescriber** → natural-language description of initial camera observation
2. **MemoryManager** → retrieves top-k relevant past experiences via cosine similarity on text embeddings
3. **VLMTaskPlanner** → selects next action using current observation + LTM context + accumulated STM
4. **Action Execution** → **not implemented** (`NotImplementedError` in `pragmabot_node.py`); integration point for object detection, grasp generation, and motion planning
5. **VLMSuccessDetector** → compares before/after images to determine action/task success
6. **STM update** → appends `(action, evaluation)` to STM; loops back to step 3 on failure
7. **VLMExperienceSummarizer** → on task completion, distills STM episode into a compact LTM entry

### Key Components

- **`pragmabot_node.py`** — main orchestrator; runs the 7-step loop inside a Gradio UI
- **`memory_manager_node.py`** — separate Gradio UI for LTM inspection, heatmaps, and embedding management
- **`vlm_client.py`** — OpenAI wrapper: uses `chat.completions.parse()` with Pydantic models for type-safe structured outputs, and `text-embedding-3-large` for embeddings
- **`scene_observer.py`** — syncs ROS color/depth/camera-info topics via `ApproximateTimeSynchronizer` (1.0 s tolerance) and returns PIL images + `CameraIntrinsics`
- **`conversation_builder.py`** — maintains two parallel logs in sync: OpenAI API format (base64 images) and Gradio Markdown format
- **`memory_manager.py`** — LTM stored in two CSVs merged on `scenario` key: `ltm.csv` (human-readable) and `ltm_<model>.csv` (base64 embeddings); keeps embedding models swappable

### Configuration

All runtime parameters live in `pragmabot/config/config.yaml` (loaded via OmegaConf). Key flags:

- `rosbag_replay: true` — skips real action execution; triggers success detector directly (use for reproducible testing)
- `activate_stm / activate_ltm` — toggle memory components
- `vlm_model` / `text_embedding_model` — model selection
- ROS topic names (configurable for different robot setups)

### Memory Schema

LTM scenario key format: `"Instruction: {task}\nScene: {initial_scene_description}"` — used for all embedding queries and retrieval.

### Action Skills

Three skills: **PUSH**, **PICK**, **PLACE**. Hard constraints are enforced via VLM prompts (e.g., PUSH only on table-top objects, cannot PICK tiny/flat objects, must PLACE before grasping another object).

### VLM Prompt Structure

Every VLM component uses three prompt layers: system prompt (role/capabilities), task prompt (user instruction), and instruction prompt (reasoning rules/constraints). STM and LTM are injected as template variables.

### Rosbag Replay Mode

With `rosbag_replay: true`, the pipeline executes synchronously and action execution is bypassed. The `image_republisher_node.py` + RQT allow manual frame-by-frame control. This is the primary mode for research and debugging.
