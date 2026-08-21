#!/usr/bin/env python3
"""Guard the planner <-> executor field contract. Runs WITHOUT ROS.

Why this test exists
--------------------
The ROS 1 executor read three fields that do not exist on the planner's
output (`action.skill`, `action.target_location`, and a `"done"` skill).
None of it was ever caught, because config.yaml sets `rosbag_replay: true`
and pragmabot_node.py skips the executor entirely in that mode - the code
path had literally never executed. The bugs would have surfaced as an
AttributeError on the first real robot run, at exactly the moment when
robot time is scarcest.

So: this checks that every field the executor sends actually exists on
NextBestAction, and that the ExecuteSkill.action goal declares a matching
field for each. It imports NO ROS packages, so it runs on a laptop with
no ROS installed - which is the whole point, since this class of bug is
what you cannot afford to discover in the lab.

Run:  python scripts/test_executor_schema.py
"""

import ast
import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
PLANNER = REPO / "pragmabot" / "src" / "pragmabot" / "vlm_task_planner.py"
EXECUTOR = REPO / "pragmabot" / "src" / "pragmabot" / "panda_skill_executor.py"
ACTION = REPO / "ros2_ws" / "src" / "pragmabot_interfaces" / "action" / "ExecuteSkill.action"

# The three fields the old executor got wrong. Named explicitly so that if
# anyone reintroduces them, the failure message says which bug came back.
KNOWN_BAD_FIELDS = {
    "skill": "use 'chosen_skill' (the old code raised AttributeError here)",
    "target_location": "use 'placement_object' (the old code silently sent an empty string)",
}


def planner_fields() -> set:
    """Field names declared on the NextBestAction pydantic model.

    Parsed with ast rather than imported: importing vlm_task_planner pulls
    in pydantic and the VLM client stack, which defeats the goal of a test
    that runs anywhere with nothing installed.
    """
    tree = ast.parse(PLANNER.read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef) and node.name == "NextBestAction":
            return {
                stmt.target.id
                for stmt in node.body
                if isinstance(stmt, ast.AnnAssign) and isinstance(stmt.target, ast.Name)
            }
    raise AssertionError(f"NextBestAction not found in {PLANNER}")


def planner_skills() -> set:
    """Allowed values of the RobotSkill enum."""
    tree = ast.parse(PLANNER.read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef) and node.name == "RobotSkill":
            return {
                stmt.value.value
                for stmt in node.body
                if isinstance(stmt, ast.Assign) and isinstance(stmt.value, ast.Constant)
            }
    raise AssertionError(f"RobotSkill not found in {PLANNER}")


def executor_required_fields() -> set:
    """Fields the executor reads via self._require(action, "<name>")."""
    return set(re.findall(r'_require\(action,\s*"([^"]+)"\)', EXECUTOR.read_text(encoding="utf-8")))


def action_goal_fields() -> set:
    """Field names in the goal section of ExecuteSkill.action.

    A '---' separator line divides goal/result/feedback. Split on the LINE,
    not the substring: the file's header comments contain '---' inside
    prose, and a naive str.split("---") returns an empty goal section (this
    test caught exactly that bug in its own first version).
    """
    fields = set()
    for raw in ACTION.read_text(encoding="utf-8").splitlines():
        if raw.strip() == "---":
            break  # end of the goal section
        line = raw.split("#")[0].strip()
        if line:
            # "string target_object" -> "target_object"
            fields.add(line.split()[-1])
    return fields


def main() -> int:
    p_fields = planner_fields()
    p_skills = planner_skills()
    e_fields = executor_required_fields()
    a_fields = action_goal_fields()

    print(f"planner NextBestAction fields : {len(p_fields)}")
    print(f"planner RobotSkill values     : {sorted(p_skills)}")
    print(f"executor reads                : {len(e_fields)}")
    print(f"ExecuteSkill goal fields      : {len(a_fields)}")
    print()

    # 1. Every field the executor reads must exist on NextBestAction.
    #    This is the check that would have caught bugs 1 and 2.
    missing = e_fields - p_fields
    assert not missing, (
        f"Executor reads field(s) that do not exist on NextBestAction: {sorted(missing)}. "
        "Update panda_skill_executor.py to match vlm_task_planner.py - and do NOT "
        "trust PROJECT_OVERVIEW.md section 6, which documents the wrong schema."
    )

    # 2. The regressions must not come back under their old names.
    for bad, hint in KNOWN_BAD_FIELDS.items():
        assert bad not in e_fields, f"Regression: executor reads '{bad}' again - {hint}"

    # 3. Every field the executor reads must have somewhere to go in the goal.
    unsendable = e_fields - a_fields
    assert not unsendable, (
        f"Executor reads {sorted(unsendable)} but ExecuteSkill.action has no such "
        "goal field - the value would be silently dropped."
    )

    # 4. The executor's skill whitelist must equal the planner's enum, so a
    #    new RobotSkill member cannot be silently rejected at runtime.
    known = set(
        re.search(r"KNOWN_SKILLS = \{([^}]+)\}", EXECUTOR.read_text(encoding="utf-8")).group(1).replace('"', "").split(", ")
    )
    assert known == p_skills, (
        f"Executor KNOWN_SKILLS {sorted(known)} != planner RobotSkill {sorted(p_skills)}. "
        "A skill the planner can emit would be rejected as unknown."
    )

    # 5. The old dead branch: 'done' was never a RobotSkill member.
    assert "done" not in p_skills, "unexpected: 'done' is a RobotSkill member now"

    print("PASS - planner, executor and action definition agree on all fields.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
