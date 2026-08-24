#!/usr/bin/env python3
"""Offline checks for the straight-line path retry policy.

No robot, no ROS: only the step schedule is exercised. What this guards is
that a failed Cartesian plan is retried with a DIFFERENT request each time.
/compute_cartesian_path is deterministic - repeating an identical request
returns an identical fraction - so a schedule that ever repeats a value, or
that never terminates, is the bug this file exists to catch.

    python scripts/test_cartesian_retry.py
"""

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "ros2_ws" / "src" / "pragmabot_bridge"))

from pragmabot_bridge.cartesian_path import cartesian_step_schedule  # noqa: E402

_passed, _failed = 0, 0


def check(name, cond):
    global _passed, _failed
    if cond:
        _passed += 1
        print(f"  ok  {name}")
    else:
        _failed += 1
        print(f"  FAIL {name}")


def main():
    print("cartesian_step_schedule")

    s = cartesian_step_schedule(0.01, 0.001, 5)
    check("starts at the configured eef_step", s[0] == 0.01)
    check("halves each retry", s[:4] == [0.01, 0.005, 0.0025, 0.00125])
    check("never goes below min_step", all(v >= 0.001 for v in s))
    check("never repeats a step", len(set(s)) == len(s))
    check("honours max_tries", len(s) <= 5)

    # The floor must terminate the sequence, not be hit repeatedly: once
    # the finest step has been asked, asking it again learns nothing.
    s = cartesian_step_schedule(0.01, 0.005, 10)
    check("stops at the floor rather than repeating it", s == [0.01, 0.005])

    check("a single try is one attempt", cartesian_step_schedule(0.01, 0.001, 1) == [0.01])
    check("max_tries below 1 still tries once",
          cartesian_step_schedule(0.01, 0.001, 0) == [0.01])

    # Misconfiguration must degrade to a usable request, never to an empty
    # schedule - an empty one would skip the service call entirely and the
    # caller would read that as a planning failure.
    check("min_step above max_step collapses to one attempt",
          cartesian_step_schedule(0.001, 0.01, 5) == [0.001])
    check("zero max_step falls back to min_step",
          cartesian_step_schedule(0.0, 0.002, 5) == [0.002])
    check("negative inputs never yield an empty schedule",
          len(cartesian_step_schedule(-1.0, -1.0, 5)) >= 1)

    print("\n" + "=" * 60)
    print(f"{_passed} passed, {_failed} failed")
    return 1 if _failed else 0


if __name__ == "__main__":
    sys.exit(main())
