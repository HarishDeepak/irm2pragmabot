#!/usr/bin/env python3
"""Prove the planning loop is bounded. Runs WITHOUT ROS or a VLM.

The bug this guards: handle_planning_request and handle_evaluation_request
called each other with no cap - mutual recursion, not a loop. A VLM that
never reports task completion would drive the arm indefinitely, bill every
step, and finally raise RecursionError inside a Gradio callback thread,
where main()'s `finally` never runs and the conversation log is lost.

Testing this against the real class would need rclpy, gradio, anthropic and
a robot. Instead the two methods are extracted from the source and run
against a stub - so the test exercises the ACTUAL control flow in the file,
not a paraphrase of it, while importing nothing.

Run:  python scripts/test_planning_loop.py
"""

import ast
import sys
import textwrap
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
NODE = REPO / "pragmabot" / "nodes" / "pragmabot_node.py"
CONFIG = REPO / "pragmabot" / "config" / "config.yaml"

MAX_STEPS_UNDER_TEST = 10


def _method_node(source: str, name: str) -> ast.FunctionDef:
    tree = ast.parse(source)
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef) and node.name == "PragmaBot":
            for item in node.body:
                if isinstance(item, ast.FunctionDef) and item.name == name:
                    return item
    raise AssertionError(f"PragmaBot.{name} not found in {NODE}")


def extract_method(source: str, name: str) -> str:
    """Return the source text of one method of the PragmaBot class."""
    return ast.get_source_segment(source, _method_node(source, name))


def calls_made_by(source: str, name: str) -> set:
    """Names of self.<method>() calls actually made inside a method.

    AST, not a substring search: the docstring of handle_evaluation_request
    legitimately *mentions* handle_planning_request while explaining that it
    no longer calls it. A text search flags that as a regression (it did on
    the first version of this test); only a real ast.Call should count.
    """
    return {
        node.func.attr
        for node in ast.walk(_method_node(source, name))
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and isinstance(node.func.value, ast.Name)
        and node.func.value.id == "self"
    }


class _Logger:
    def warn(self, *a, **k):
        pass

    def info(self, *a, **k):
        pass


class _Config:
    max_steps = MAX_STEPS_UNDER_TEST
    rosbag_replay = True


class FakeBot:
    """Minimal stand-in exposing only what handle_planning_request touches.

    _plan_one_step always returns False - i.e. the task NEVER completes,
    the exact condition that made the old code recurse forever.
    """

    def __init__(self):
        self.config = _Config()
        self.logger = _Logger()
        self.calls = 0
        self.stm_notes = []

    def _plan_one_step(self, chatbot):
        self.calls += 1
        if self.calls > 1000:  # would have been RecursionError territory
            raise AssertionError("planning loop is unbounded - it never stopped")
        return False

    def append_to_stm_if_activated(self, kind, value):
        self.stm_notes.append((kind, value))


def main() -> int:
    source = NODE.read_text(encoding="utf-8")

    # 1. Structural: the evaluation handler must not call back into planning.
    eval_calls = calls_made_by(source, "handle_evaluation_request")
    assert "handle_planning_request" not in eval_calls, (
        "Regression: handle_evaluation_request calls handle_planning_request "
        "again - the mutual recursion is back."
    )
    assert "_plan_one_step" not in eval_calls, (
        "Regression: handle_evaluation_request calls _plan_one_step - that is "
        "the same recursion cycle wearing the new method's name."
    )

    # 2. Behavioural: run the real loop body against a never-completing stub.
    namespace = {}
    exec(  # noqa: S102 - executing our own source, by design
        "class _Host:\n" + textwrap.indent(extract_method(source, "handle_planning_request"), "    "),
        {"getattr": getattr},
        namespace,
    )
    bot = FakeBot()
    namespace["_Host"].handle_planning_request(bot, None)

    assert bot.calls == MAX_STEPS_UNDER_TEST, (
        f"Expected exactly {MAX_STEPS_UNDER_TEST} planning steps before the cap, got {bot.calls}"
    )

    # 3. The abort must reach STM, not just the log - the summarizer needs to
    #    see that the task ran out of steps rather than failing an action.
    assert any("max_steps" in str(v) for _, v in bot.stm_notes), (
        "Hitting the cap must be recorded in STM, not only logged"
    )

    # 4. config.yaml must actually define max_steps, or the loop silently
    #    falls back to the getattr default and the config knob is a lie.
    assert "max_steps" in CONFIG.read_text(encoding="utf-8"), "config.yaml does not define max_steps"

    print(f"planning steps before cap : {bot.calls}")
    print(f"abort recorded in STM     : {bot.stm_notes[-1][1]}")
    print("PASS - planning loop is bounded and the abort is recorded.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
