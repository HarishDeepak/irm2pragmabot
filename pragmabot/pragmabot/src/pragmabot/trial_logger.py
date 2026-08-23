"""Per-trial JSONL logging — the numbers a results table is made of.

WHY THIS EXISTS
---------------
`save_conversation_log()` writes one JSON blob, once, from `main()`'s
`finally`. Three problems, each fatal for the report:

  1. A crash mid-run can bypass it, and the run that crashes is often the
     interesting one.
  2. It stores the conversation, not the measurements. No retrieval
     similarities, no ground-truth labels, no timings, no token counts.
  3. It is one file per session, so trials cannot be counted, grouped or
     joined.

You cannot re-run experiments in report week. Anything not captured while
the robot is in front of you is gone. This module writes one line per step,
flushed immediately, so a killed process still leaves usable data.

DESIGN
------
- One JSONL file per trial: `data/logs/<timestamp>_<uuid6>.jsonl`.
- Append one line per step, `flush()` + `os.fsync()` every write. Slower
  than buffering, and correct when the process dies.
- Never raises into the caller. A logging failure must not abort a robot
  trial — every public method swallows and warns. Losing a log line is bad;
  losing an afternoon of robot time to a logging bug is worse.
- Does not touch any of the 11 protected upstream modules.

The `human_gt_action_success` field cannot be reconstructed afterwards: the
paper reports false-positive and false-negative rates for the success
detector (5% FP, 6.67% FN), which needs a human label per step to build a
confusion matrix against. It defaults to None and is filled from the UI.
"""

import datetime
import json
import logging
import os
import time
import uuid
from pathlib import Path
from typing import Any, Dict, List, Optional

logger = logging.getLogger(__name__)

SCHEMA_VERSION = 1


def _jsonable(value: Any) -> Any:
    """Coerce anything into something json.dumps can take.

    Enums, numpy scalars and pydantic models all turn up here. A log line
    that fails to serialise is a lost measurement, so fall back to repr()
    rather than raising.
    """
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    for attr in ("value", "item"):  # enum.Enum, numpy scalar
        if hasattr(value, attr):
            try:
                got = getattr(value, attr)
                return _jsonable(got() if callable(got) else got)
            except Exception:  # noqa: BLE001
                pass
    if hasattr(value, "model_dump"):  # pydantic v2
        try:
            return _jsonable(value.model_dump())
        except Exception:  # noqa: BLE001
            pass
    return repr(value)


class TrialLogger:
    """One instance per trial. Call `start()`, then `log_step()`, then `finish()`."""

    def __init__(self, log_dir: Path, instruction: str, config: Optional[Dict] = None) -> None:
        self.trial_id = f"{datetime.datetime.now():%Y%m%d_%H%M%S}_{uuid.uuid4().hex[:6]}"
        self.instruction = instruction
        self.path = Path(log_dir) / f"{self.trial_id}.jsonl"
        self._t0 = time.time()
        self._n_steps = 0
        self._tokens_in = 0
        self._tokens_out = 0
        self._closed = False

        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            self._write({
                "record": "trial_start",
                "schema_version": SCHEMA_VERSION,
                "trial_id": self.trial_id,
                "instruction": instruction,
                "started_at": datetime.datetime.now().isoformat(timespec="seconds"),
                # The knobs that change what an experiment MEANS. Without
                # these a log line cannot be attributed to a condition.
                "config": _jsonable(config or {}),
            })
            logger.info("Trial log: %s", self.path)
        except Exception as exc:  # noqa: BLE001
            logger.error("Could not open trial log %s: %s", self.path, exc)
            self._closed = True

    # -- public API ---------------------------------------------------

    def log_step(
        self,
        time_step: int,
        chosen_skill: Any = None,
        target_object: str = "",
        placement_object: str = "",
        chain_of_thought_reasoning: str = "",
        ltm_retrieved_scenarios: Optional[List[str]] = None,
        ltm_similarities: Optional[List[float]] = None,
        stm_len_chars: int = 0,
        exec_success: Optional[bool] = None,
        exec_message: str = "",
        vlm_is_action_successful: Optional[bool] = None,
        vlm_is_task_completed: Optional[bool] = None,
        human_gt_action_success: Optional[bool] = None,
        input_tokens: int = 0,
        output_tokens: int = 0,
        latency_s: float = 0.0,
        before_img_path: str = "",
        after_img_path: str = "",
        error: str = "",
        **extra: Any,
    ) -> None:
        """Append one step. Never raises."""
        self._n_steps = max(self._n_steps, time_step)
        self._tokens_in += int(input_tokens or 0)
        self._tokens_out += int(output_tokens or 0)

        self._write({
            "record": "step",
            "trial_id": self.trial_id,
            "time_step": time_step,
            "t_since_start_s": round(time.time() - self._t0, 3),
            "chosen_skill": _jsonable(chosen_skill),
            "target_object": target_object,
            "placement_object": placement_object,
            "chain_of_thought_reasoning": chain_of_thought_reasoning,
            # Retrieval is the graded mechanism. These two lists ARE the
            # Fig. 7 ablation: without them there is no way to show what was
            # retrieved or how close it was.
            "ltm_retrieved_scenarios": _jsonable(ltm_retrieved_scenarios or []),
            "ltm_similarities": _jsonable(ltm_similarities or []),
            "stm_len_chars": stm_len_chars,
            "exec_success": exec_success,
            "exec_message": exec_message,
            "vlm_is_action_successful": vlm_is_action_successful,
            "vlm_is_task_completed": vlm_is_task_completed,
            "human_gt_action_success": human_gt_action_success,
            "input_tokens": input_tokens,
            "output_tokens": output_tokens,
            "latency_s": round(latency_s, 3),
            "before_img_path": before_img_path,
            "after_img_path": after_img_path,
            "error": error,
            **{k: _jsonable(v) for k, v in extra.items()},
        })

    def label_step(self, time_step: int, human_gt_action_success: bool) -> None:
        """Record a human ground-truth label for an already-logged step.

        Appended as its own record rather than rewriting the step line —
        JSONL is append-only, and the analysis script merges on time_step.
        This is what makes the success-detector confusion matrix possible.
        """
        self._write({
            "record": "human_label",
            "trial_id": self.trial_id,
            "time_step": time_step,
            "human_gt_action_success": bool(human_gt_action_success),
            "labelled_at": datetime.datetime.now().isoformat(timespec="seconds"),
        })

    def finish(self, outcome: str, notes: str = "") -> None:
        """Close the trial. `outcome`: completed | max_steps | aborted | crashed."""
        if self._closed:
            return
        if outcome not in {"completed", "max_steps", "aborted", "crashed"}:
            logger.warning("Unexpected trial outcome %r", outcome)
        self._write({
            "record": "trial_end",
            "trial_id": self.trial_id,
            "outcome": outcome,
            "n_steps": self._n_steps,
            "total_input_tokens": self._tokens_in,
            "total_output_tokens": self._tokens_out,
            "total_tokens": self._tokens_in + self._tokens_out,
            "wall_time_s": round(time.time() - self._t0, 3),
            "notes": notes,
            "ended_at": datetime.datetime.now().isoformat(timespec="seconds"),
        })
        self._closed = True

    # -- internals ----------------------------------------------------

    def _write(self, record: Dict[str, Any]) -> None:
        """Append one line and force it to disk. Never raises."""
        if self._closed and record.get("record") != "trial_end":
            return
        try:
            line = json.dumps(record, ensure_ascii=False)
        except Exception as exc:  # noqa: BLE001
            logger.error("Trial log record not serialisable: %s", exc)
            return
        try:
            # Reopened per write: an append-mode handle held across a crash
            # can lose buffered lines, and this is not a hot path.
            with open(self.path, "a", encoding="utf-8") as fh:
                fh.write(line + "\n")
                fh.flush()
                os.fsync(fh.fileno())
        except Exception as exc:  # noqa: BLE001
            logger.error("Could not write trial log line: %s", exc)


def load_trials(log_dir: Path) -> "Any":
    """Load a directory of trial JSONL files into a tidy pandas DataFrame.

    One row per step, with human labels merged in and trial-level columns
    broadcast across the trial's steps — the shape the paper's Table II is
    computed from.

        df = load_trials(Path("pragmabot/data/logs"))
        df.groupby("instruction")["vlm_is_task_completed"].mean()
    """
    import pandas as pd

    steps, ends, labels = [], [], []
    for path in sorted(Path(log_dir).glob("*.jsonl")):
        for i, raw in enumerate(path.read_text(encoding="utf-8").splitlines()):
            if not raw.strip():
                continue
            try:
                rec = json.loads(raw)
            except json.JSONDecodeError:
                # A half-written final line is expected after a crash — that
                # is the cost of append-and-fsync, and it is the right cost.
                logger.warning("Skipping malformed line %s:%d", path.name, i + 1)
                continue
            kind = rec.get("record")
            if kind == "step":
                steps.append(rec)
            elif kind == "trial_end":
                ends.append(rec)
            elif kind == "human_label":
                labels.append(rec)

    df = pd.DataFrame(steps)
    if df.empty:
        return df

    if labels:
        lab = pd.DataFrame(labels)[["trial_id", "time_step", "human_gt_action_success"]]
        df = df.drop(columns=["human_gt_action_success"], errors="ignore").merge(
            lab, on=["trial_id", "time_step"], how="left")

    if ends:
        end = pd.DataFrame(ends)[["trial_id", "outcome", "n_steps", "wall_time_s", "total_tokens"]]
        df = df.merge(end, on="trial_id", how="left")

    return df
