#!/usr/bin/env python3
"""Offline checks for the per-trial JSONL logger.

No robot, no VLM, no ROS. Verifies the properties the report depends on:
a killed process still leaves usable data, retrieval similarities survive
the round trip, and human labels merge back onto the right step.

    python scripts/test_trial_logger.py
"""

import json
import sys
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "pragmabot" / "src"))

from pragmabot.trial_logger import TrialLogger, load_trials  # noqa: E402


def _mk(tmp, instruction="pick up the red cup"):
    return TrialLogger(Path(tmp), instruction, config={"activate_ltm": True, "retrieval_top_k": 5})


def test_survives_a_crash_mid_trial():
    """The whole reason for fsync-per-line: no finish(), data still there."""
    with tempfile.TemporaryDirectory() as tmp:
        log = _mk(tmp)
        log.log_step(time_step=1, chosen_skill="pick", target_object="red cup")
        log.log_step(time_step=2, chosen_skill="place", placement_object="plate")
        # Simulate SIGKILL: no finish(), no close, nothing unwound.
        del log

        lines = [json.loads(x) for x in Path(tmp).glob("*.jsonl").__next__()
                 .read_text(encoding="utf-8").splitlines() if x.strip()]
        kinds = [r["record"] for r in lines]
        assert kinds == ["trial_start", "step", "step"], kinds
        assert lines[1]["target_object"] == "red cup"


def test_retrieval_similarities_are_kept():
    """Fig. 7 is built from these. Losing them loses the graded experiment."""
    with tempfile.TemporaryDirectory() as tmp:
        log = _mk(tmp)
        log.log_step(time_step=1,
                     ltm_retrieved_scenarios=["scenario A", "scenario B"],
                     ltm_similarities=[0.83, 0.71])
        log.finish("completed")

        df = load_trials(Path(tmp))
        assert df.loc[0, "ltm_similarities"] == [0.83, 0.71]
        assert df.loc[0, "ltm_retrieved_scenarios"][0] == "scenario A"


def test_human_label_merges_onto_the_right_step():
    """Needed for the success-detector confusion matrix (paper: 5% FP, 6.67% FN)."""
    with tempfile.TemporaryDirectory() as tmp:
        log = _mk(tmp)
        log.log_step(time_step=1, vlm_is_action_successful=True)
        log.log_step(time_step=2, vlm_is_action_successful=True)
        log.label_step(time_step=2, human_gt_action_success=False)  # a false positive
        log.finish("completed")

        df = load_trials(Path(tmp)).sort_values("time_step")
        assert df.loc[df.time_step == 1, "human_gt_action_success"].isna().all()
        row = df.loc[df.time_step == 2].iloc[0]
        assert row["human_gt_action_success"] is False or row["human_gt_action_success"] == False  # noqa: E712
        assert row["vlm_is_action_successful"] == True  # noqa: E712


def test_enum_and_odd_types_do_not_break_serialisation():
    """chosen_skill arrives as an enum; a crash here would lose the step."""
    import enum

    class RobotSkill(enum.Enum):
        PICK = "pick"

    with tempfile.TemporaryDirectory() as tmp:
        log = _mk(tmp)
        log.log_step(time_step=1, chosen_skill=RobotSkill.PICK,
                     extra_object=object())  # deliberately unserialisable
        log.finish("completed")
        df = load_trials(Path(tmp))
        assert df.loc[0, "chosen_skill"] == "pick"


def test_trial_end_totals():
    with tempfile.TemporaryDirectory() as tmp:
        log = _mk(tmp)
        log.log_step(time_step=1, input_tokens=1200, output_tokens=300)
        log.log_step(time_step=2, input_tokens=1800, output_tokens=250)
        log.finish("max_steps", notes="hit the cap")

        end = [json.loads(x) for x in Path(tmp).glob("*.jsonl").__next__()
               .read_text(encoding="utf-8").splitlines() if x.strip()][-1]
        assert end["record"] == "trial_end"
        assert end["outcome"] == "max_steps"
        assert end["total_tokens"] == 3550 and end["n_steps"] == 2


def test_logging_failure_never_kills_the_trial():
    """A logging bug must not cost robot time."""
    log = TrialLogger(Path("/nonexistent/deep/path"), "x")
    log.log_step(time_step=1, chosen_skill="pick")   # must not raise
    log.label_step(time_step=1, human_gt_action_success=True)
    log.finish("aborted")


def test_malformed_trailing_line_is_skipped():
    """A half-written line after SIGKILL must not break analysis."""
    with tempfile.TemporaryDirectory() as tmp:
        log = _mk(tmp)
        log.log_step(time_step=1, chosen_skill="pick")
        log.finish("completed")
        p = next(Path(tmp).glob("*.jsonl"))
        with open(p, "a", encoding="utf-8") as fh:
            fh.write('{"record": "step", "trial_id": "trunc')  # torn write
        df = load_trials(Path(tmp))
        assert len(df) == 1


def test_two_trials_are_separate_files_and_join():
    with tempfile.TemporaryDirectory() as tmp:
        a, b = _mk(tmp, "task A"), _mk(tmp, "task B")
        a.log_step(time_step=1); a.finish("completed")
        b.log_step(time_step=1); b.finish("aborted")
        assert len(list(Path(tmp).glob("*.jsonl"))) == 2
        df = load_trials(Path(tmp))
        assert set(df["outcome"]) == {"completed", "aborted"}
        assert df["trial_id"].nunique() == 2


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
        print(f"  ok  {t.__name__}")
    print(f"\n{len(tests)} passed")
