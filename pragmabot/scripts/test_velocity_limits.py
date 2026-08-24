#!/usr/bin/env python3
"""Offline checks for the FR3 position-dependent velocity limit gate.

No robot, no ROS: JointTrajectory is duck-typed below, and fr3_limits.py
imports nothing but numpy. What this guards is the decision _execute_trajectory
makes before a trajectory reaches the controller - legal through, illegal
slowed, hopeless refused.

The concrete case is the one that actually faulted the arm: joint 2 at
-1.816 rad, 0.0201 rad from its -1.8361 limit, where 0.57 rad/s is permitted
and MoveIt had planned 2.62.

    python scripts/test_velocity_limits.py
"""

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "ros2_ws" / "src" / "pragmabot_bridge"))

from pragmabot_bridge.fr3_limits import FR3Limits  # noqa: E402

_passed, _failed = 0, 0


def check(name, cond):
    global _passed, _failed
    if cond:
        _passed += 1
        print(f"  ok  {name}")
    else:
        _failed += 1
        print(f"  FAIL {name}")


class _Pt:
    def __init__(self, positions, velocities):
        self.positions = positions
        self.velocities = velocities


class _Traj:
    """Minimal stand-in for trajectory_msgs/JointTrajectory."""

    def __init__(self, names, points):
        self.joint_names = names
        self.points = points


NAMES = [f"fr3_joint{i}" for i in range(1, 8)]


def _traj(q2, dq2):
    """One waypoint with joint 2 at `q2` moving at `dq2`, others parked."""
    pos = [0.0, q2, 0.0, -1.5, 0.0, 1.5, 0.0]
    vel = [0.0, dq2, 0.0, 0.0, 0.0, 0.0, 0.0]
    return _Traj(NAMES, [_Pt(pos, vel)])


def main():
    L = FR3Limits.load()
    print("FR3 position-dependent velocity limit")

    # The real fault. 0.0201 rad from the limit -> 0.57 rad/s permitted.
    permitted = L.max_velocity_at("fr3_joint2", -1.816)
    check("the faulting configuration permits ~0.57 rad/s",
          0.56 < permitted < 0.58)
    check("MoveIt's nominal 2.62 rad/s is >4x over that",
          2.62 / permitted > 4.0)

    ok, reason, worst = L.check_trajectory(_traj(-1.816, 2.62), safety=0.9)
    check("the faulting trajectory is rejected", not ok)
    check("the reason names the joint", "fr3_joint2" in reason)
    check("the reason carries real numbers, not just 'rejected'",
          "0.57" in reason or "0.572" in reason)

    factor = L.retime_factor_for(_traj(-1.816, 2.62), safety=0.9)
    check("a slow-down factor is offered", factor is not None and 0 < factor < 1)
    # Re-timing scales velocity linearly, so the slowed trajectory must pass.
    slowed = _traj(-1.816, 2.62 * factor)
    ok_after, _, _ = L.check_trajectory(slowed, safety=0.9)
    check("the trajectory is legal once slowed by that factor", ok_after)

    # Mid-range motion must not be touched - a gate that slows everything
    # would make every pick crawl and would hide the real problem.
    ok, _, _ = L.check_trajectory(_traj(0.0, 2.0), safety=0.9)
    check("a mid-range trajectory passes untouched", ok)
    check("and asks for no slow-down",
          L.retime_factor_for(_traj(0.0, 2.0), safety=0.9) == 1.0)

    # Global velocity scaling is NOT a substitute: 0.2 scaling still
    # commands 0.52 rad/s against a 0.57 budget.
    ok_scaled, _, _ = L.check_trajectory(_traj(-1.816, 2.62 * 0.2), safety=0.9)
    check("global 0.2 velocity scaling does NOT make it legal", not ok_scaled)

    # Zero-velocity waypoints (trajectory endpoints) must not divide by zero
    # or be reported as violations.
    ok, _, _ = L.check_trajectory(_traj(-1.816, 0.0), safety=0.9)
    check("a stationary waypoint at the same position is legal", ok)

    # Asymmetric joints: 4 and 6 never contain zero, so a symmetric
    # Panda-era assumption would compute the wrong margin here.
    lo6, hi6 = L.bounds()["fr3_joint6"]
    check("joint 6's range excludes zero (asymmetric)", lo6 > 0.0)
    check("distance_to_limit respects the asymmetry",
          abs(L.distance_to_limit("fr3_joint6", lo6 + 0.01) - 0.01) < 1e-9)

    print("\n" + "=" * 60)
    print(f"{_passed} passed, {_failed} failed")
    return 1 if _failed else 0


if __name__ == "__main__":
    sys.exit(main())
