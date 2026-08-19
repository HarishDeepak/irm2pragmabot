# Third-Party Licences

This repository vendors several third-party projects. Each keeps its own `LICENSE` file in place, unmodified, in its own directory. This file is a summary — the licence files themselves are authoritative.

| Component | Path | Licence | Upstream |
|---|---|---|---|
| PragmaBot | `pragmabot/` | **BSD-3-Clause** | [leggedrobotics/pragmabot](https://github.com/leggedrobotics/pragmabot) |
| GraspGen | `GraspGen/` | **NVIDIA License** | [NVlabs/GraspGen](https://github.com/NVlabs/GraspGen) |
| Grounded-SAM-2 | `groundedsam/Grounded-SAM-2/` | **Apache-2.0** | [IDEA-Research/Grounded-SAM-2](https://github.com/IDEA-Research/Grounded-SAM-2) |
| franka_ros2 | `ros2_ws/franka_ros2/` | **Apache-2.0** | [frankarobotics/franka_ros2](https://github.com/frankarobotics/franka_ros2) |
| libfranka | `ros2_ws/franka_ros2/libfranka/` | **Apache-2.0** | [frankarobotics/libfranka](https://github.com/frankarobotics/libfranka) |
| easy_handeye2 | `ros2_ws/franka_ros2/easy_handeye2/` | **LGPL-3.0** | [marcoesposito1988/easy_handeye2](https://github.com/marcoesposito1988/easy_handeye2) |
| zed-ros2-wrapper | `zed_ros2_ws/src/zed-ros2-wrapper/` | **Apache-2.0** | [stereolabs/zed-ros2-wrapper](https://github.com/stereolabs/zed-ros2-wrapper) |

---

## Important terms

### GraspGen — NVIDIA License: non-commercial use only

> **§3.3 Use Limitation.** "The Work and any derivative works thereof only may be used or intended for use **non-commercially**. … As used herein, 'non-commercially' means for **research or evaluation purposes only**."

This repository is academic coursework (TU Darmstadt, *Praktikum zur intelligenten Robotermanipulation*), which falls within that limitation. **Anyone reusing this repository inherits this restriction for the `GraspGen/` component.**

§3.1 additionally requires that redistribution keep a complete copy of the licence and retain all copyright, patent, trademark and attribution notices unmodified. `GraspGen/LICENSE` is present and unaltered.

### easy_handeye2 — LGPL-3.0

Vendored **unmodified**; it is driven entirely through launch arguments. `easy_handeye2/LICENSE.md` is retained.

If this component is ever modified, LGPL-3.0 requires those modifications be made available under the same licence.

### PragmaBot — BSD-3-Clause

`pragmabot/LICENSE` retains the original notice: *Creators: Kaixian Qu, Guowei Lan, Changan Chen — Copyright (c) 2026, ETH Zürich.*

This repository's history begins at upstream commit `ee68710b2525bdb1e1cdab20d934be547643913b`, the official release commit. Verify with:

```bash
git remote add upstream https://github.com/leggedrobotics/pragmabot.git
git fetch upstream
git merge-base HEAD upstream/main   # -> ee68710b2525bdb1e1cdab20d934be547643913b
```

---

## Model weights

No model checkpoints are committed. `setup.sh` downloads each from its official source, so their separate licence terms apply directly between the user and the provider:

| Weight | Source | Terms |
|---|---|---|
| GraspGen checkpoints | [huggingface.co/adithyamurali/GraspGenModels](https://huggingface.co/adithyamurali/GraspGenModels) | NVIDIA |
| SAM 2.1 | Meta (`dl.fbaipublicfiles.com`) | per Meta's SAM 2 terms |
| GroundingDINO | IDEA-Research GitHub Releases | Apache-2.0 |

---

## Citation

If this work is referenced, cite the original paper:

```bibtex
@article{qu2026pragmabot,
  title   = {A Pragmatist Robot: Learning to Plan Tasks by Experiencing the Real World},
  author  = {Qu, Kaixian and Lan, Guowei and Zurbr\"ugg, Ren\'e and Chen, Changan
             and Mower, Christopher E. and Bou-Ammar, Haitham and Hutter, Marco},
  journal = {IEEE Robotics and Automation Letters},
  year    = {2026},
  note    = {arXiv:2507.16713}
}
```
