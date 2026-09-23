# Memory Representations for Robotic Task Planning

**Franka FR3 execution layer for PragmaBot — iRobMan Praktikum (Part II), PEARL Lab, TU Darmstadt**

> Forked from [leggedrobotics/pragmabot](https://github.com/leggedrobotics/pragmabot) · Qu et al., *A Pragmatist Robot: Learning to Plan Tasks by Experiencing the Real World*, IEEE RAL 2026

[![IEEE RAL](https://img.shields.io/badge/IEEE_RAL-2026-blue)](https://ieeexplore.ieee.org/document/11419794)
[![arXiv](https://img.shields.io/badge/arXiv-2507.16713-b31b1b)](https://arxiv.org/abs/2507.16713)
[![ROS](https://img.shields.io/badge/ROS-2_Humble-blue)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.10%2B-green)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-BSD--3--Clause-orange)](LICENSE)

> **Corrected 2026-09-23:** this file previously described a Franka **Panda**
> + ROS **Noetic (ROS1)** + TRAC-IK design. That was an earlier plan that was
> superseded — the actual robot is a **Franka FR3** ("Athna"), the actual
> stack is **ROS 2 Humble** running host-only (no ROS1, no bridge process),
> and the current IK solver is `lma_kinematics_plugin`, not TRAC-IK. See the
> top-level **[`../README.md`](../README.md)** for the accurate architecture,
> and **[`CLAUDE.md`](CLAUDE.md)** for verified facts and hard rules. This
> file is kept for the project pitch/citation; don't treat the tables below
> as current implementation detail.

---

## What This Fork Builds

This project adapts the PragmaBot VLM-memory architecture to a **7-DoF Franka FR3** at PEARL Lab. The upstream system provides the full cognitive loop — VLM planning, STM self-reflection, LTM distillation and retrieval — but leaves action execution as a `NotImplementedError`. This fork implements that layer.

**Goal:** An agentic VLM-based planning system where a VLM evaluates each robot action outcome via self-reflection and triggers replanning on failure; outcomes stored in a short-term memory buffer for within-task adaptation. Successful sequences are distilled into long-term memory for retrieval-augmented plan generation, enabling cross-task knowledge reuse on a real Franka FR3.

**Additional investigations (beyond upstream):**
- Ontology-based experience representations for richer semantic retrieval
- Local VLM acceleration to reduce API latency on real hardware

---

## What This Fork Adds (Franka Execution Layer)

The upstream pipeline calls `handle_planning_request()` for action execution and raises `NotImplementedError`. This fork implements that integration point for a Franka FR3:

| Component | Implementation |
|-----------|---------------|
| Object detection | Grounded-SAM-2 (open-vocabulary, text-prompted) |
| Perception | ZED2 RGB-D stereo camera, fixed off-arm |
| Grasp synthesis | GraspGen (6-DoF, NVIDIA) |
| Motion planning | MoveIt 2 (`lma_kinematics_plugin`) on the 7-DoF FR3 |
| Execution bridge | `pragmabot_bridge`, a native ROS 2 node (see `ros2_ws/src/pragmabot_bridge/`) |

**Status: pick + place working reliably on real hardware** (see `ARMIN.md` for the day-by-day log); push implemented; LTM/STM both live.

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

## Prerequisites, installation and usage

See the top-level **[`../README.md`](../README.md)** — it has the actual,
verified setup (Ubuntu + ROS 2 Humble, colcon not catkin, `bash setup.sh`,
per-tool venvs for GraspGen/Grounded-SAM-2) and the `docker compose` /
`ros2 launch` commands that actually run this system.

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
