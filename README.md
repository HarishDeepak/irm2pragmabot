# Memory Representations for Robotic Task Planning

**Franka Panda adaptation of PragmaBot — iRobMan Praktikum (Project 2), PEARL Lab, TU Darmstadt**

> Forked from [leggedrobotics/pragmabot](https://github.com/leggedrobotics/pragmabot) · Qu et al., *A Pragmatist Robot: Learning to Plan Tasks by Experiencing the Real World*, IEEE RAL 2026

[![IEEE RAL](https://img.shields.io/badge/IEEE_RAL-2026-blue)](https://ieeexplore.ieee.org/document/11419794)
[![arXiv](https://img.shields.io/badge/arXiv-2507.16713-b31b1b)](https://arxiv.org/abs/2507.16713)
[![ROS](https://img.shields.io/badge/ROS-Noetic-blue)](https://wiki.ros.org/noetic)
[![Python](https://img.shields.io/badge/Python-3.8%2B-green)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-BSD--3--Clause-orange)](LICENSE)

---

## What This Fork Builds

This project adapts the PragmaBot VLM-memory architecture to a **7-DoF Franka Panda** at PEARL Lab. The upstream system provides the full cognitive loop — VLM planning, STM self-reflection, LTM distillation and retrieval — but leaves action execution as a `NotImplementedError`. This fork implements that layer.

**Goal:** An agentic VLM-based planning system where a VLM evaluates each robot action outcome via self-reflection and triggers replanning on failure; outcomes stored in a short-term memory buffer for within-task adaptation. Successful sequences are distilled into long-term memory for retrieval-augmented plan generation, enabling cross-task knowledge reuse on a real Franka Panda.

**Additional investigations (beyond upstream):**
- Ontology-based experience representations for richer semantic retrieval
- Local VLM acceleration to reduce API latency on real hardware

---

## What This Fork Adds (Franka Execution Layer)

The upstream pipeline calls `handle_planning_request()` for action execution and raises `NotImplementedError`. This fork implements that integration point for a Franka Panda:

| Component | Implementation |
|-----------|---------------|
| Object detection | GroundingDINO (open-vocabulary, text-prompted) |
| 6D pose estimation | ZED2i RGB-D stereo camera |
| Motion planning | MoveIt + TRAC-IK on 7-DoF Franka Panda |
| ROS bridge | Upstream Python nodes wired to ROS topics/services |

**Status: in progress** — ROS node architecture designed; execution layer implementation ongoing at PEARL Lab.

---

## Upstream: PragmaBot Cognitive Loop

The upstream system enables robots to learn to plan tasks by experiencing the real world — without model fine-tuning or dense human supervision. A VLM evaluates action outcomes and self-reflects on failures, storing reflections in STM for within-task adaptation. After each task, lessons are distilled into LTM and retrieved via RAG for new tasks.

> **Upstream results (on legged manipulator, ETH Zürich):** STM self-reflection raises task success from **35% → 84%**. LTM with RAG raises single-trial success from **22% → 80%** across unseen scenarios.

### Pipeline

1. `VLMSceneDescriber` — natural-language description of the scene
2. `MemoryManager` — retrieves top-*k* relevant LTM experiences via cosine similarity
3. `VLMTaskPlanner` — selects next action from observation + LTM + STM context
4. **Action execution** — *(this fork: GroundingDINO + MoveIt + Franka Panda)*
5. `VLMSuccessDetector` — compares before/after images; returns success signal + scene description
6. STM updated with (action, evaluation) pair; replanning triggered on failure — steps 3–6 repeat
7. On task completion, `VLMExperienceSummarizer` distils STM episode → LTM entry

---

## Prerequisites

- Ubuntu 20.04 with [ROS Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu/)
- Python 3.8+
- OpenAI API key (GPT-4o)
- Franka Panda with ZED2i RGB-D camera *(for execution layer)*

## Installation

```bash
git clone https://github.com/HarishDeepak/IRM2.git
cd IRM2
pip install -r requirements.txt
```

```bash
cd <catkin_workspace>
catkin build pragmabot
```

```bash
export OPENAI_API_KEY="your-openai-api-key"
```

## Usage

```bash
roslaunch pragmabot launch_pragmabot.launch
```

---

## Citation

```bibtex
@article{qu2026pragmatist,
  title={A Pragmatist Robot: Learning to Plan Tasks by Experiencing the Real World},
  author={Qu, Kaixian and Lan, Guowei and Zurbrügg, René and Chen, Changan and
          Mower, Christopher E and Bou-Ammar, Haitham and Hutter, Marco},
  journal={IEEE Robotics and Automation Letters},
  year={2026},
  publisher={IEEE}
}
```

## License

BSD 3-Clause. Original copyright © 2026 ETH Zürich (Qu, Lan, Chen et al.).  
Fork contributions © 2026 Harish Deepak, TU Darmstadt.  
See [LICENSE](LICENSE) for details.
