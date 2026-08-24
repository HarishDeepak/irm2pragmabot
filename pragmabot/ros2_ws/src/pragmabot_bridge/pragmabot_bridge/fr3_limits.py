"""FR3 joint limits, including the position-dependent velocity limit.

WHY THIS MODULE EXISTS
----------------------
MoveIt plans against the URDF, and a URDF joint can carry only a CONSTANT
`velocity` limit. The FR3's real velocity limit is not constant: it shrinks
as a joint approaches its position limit, so that the joint can always brake
before hitting the stop. MoveIt therefore cannot represent this limit, does
not know it exists, and will happily produce a trajectory that libfranka
then refuses at 1 kHz with a velocity violation.

That is not a hypothetical. On this robot, a pick sent the arm to a
configuration with joint 2 at -1.816 rad, 0.0201 rad from its -1.8361
limit. Its permitted velocity there is 0.57 rad/s; MoveIt had planned
against the nominal 2.62 rad/s - a 4.6x overspeed - and the controller
faulted with "speed limits reached".

Global velocity scaling does NOT fix this. Scaling to 0.2 still commands
0.52 rad/s against a 0.57 rad/s budget: one slightly worse configuration
and it trips again. The limit has to be modelled, not approximated.

THE MODEL
---------
From franka_description/robots/fr3/joint_limits.yaml, every joint carries
`velocity_offset` and `deceleration_limit` alongside its nominal limits.
These are the two constants of a braking-distance law - a joint `d` radians
from its limit, able to decelerate at `a`, may travel at most

    qd_max(d) = min(qd_nominal, velocity_offset + sqrt(2 * a * d))

Self-consistency check, which is why this reading is trustworthy: solving
qd_max(d) = qd_nominal gives the distance at which the position-based limit
stops binding. For joint 2 that is 1.07 rad; joint 1, 0.32; joint 4, 0.64;
joint 5, 0.65. Every joint saturates smoothly at its own nominal limit at a
sane distance. A wrong formula would not do that for all seven.

Hard per-joint limits from libfranka's rate_limiting.h (constants, not
position dependent): max acceleration 10 rad/s^2, max jerk 5000 rad/s^3.
"""

from pathlib import Path

import numpy as np

# libfranka/include/franka/rate_limiting.h - kMaxJointAcceleration, kMaxJointJerk.
FR3_MAX_JOINT_ACCEL = 10.0     # rad/s^2, all joints
FR3_MAX_JOINT_JERK = 5000.0    # rad/s^3, all joints

# Fallback if franka_description is not on disk. Same numbers as
# franka_description/robots/fr3/joint_limits.yaml, kept here so the guard
# still works rather than silently disabling itself.
_FR3_FALLBACK = {
    "fr3_joint1": (-2.9007, 2.9007, 2.62, 0.6520, 6.000),
    "fr3_joint2": (-1.8361, 1.8361, 2.62, 0.2500, 2.585),
    "fr3_joint3": (-2.9007, 2.9007, 2.62, 0.2005, 3.500),
    "fr3_joint4": (-3.0770, -0.1169, 2.62, 0.3542, 4.000),
    "fr3_joint5": (-2.8763, 2.8763, 5.26, 0.5738, 17.00),
    "fr3_joint6": (0.4398, 4.6216, 4.18, 0.4885, 5.500),
    "fr3_joint7": (-3.0508, 3.0508, 5.26, 0.4592, 17.00),
}

_SEARCH = [
    Path.home() / "ros2_ws/franka_ros2/franka_description/robots/fr3/joint_limits.yaml",
    Path("/ros2_ws/src/franka_description/robots/fr3/joint_limits.yaml"),
]


class FR3Limits:
    """Position, velocity and position-dependent velocity limits per joint."""

    def __init__(self, table=None):
        # name -> (lower, upper, nominal_velocity, velocity_offset, decel_limit)
        self.table = dict(table or _FR3_FALLBACK)

    @classmethod
    def load(cls, path=None):
        """Read franka_description's joint_limits.yaml, else use the fallback.

        Parsed with a tiny hand-rolled reader rather than PyYAML: this module
        is imported by the bridge in a ROS 2 environment where PyYAML is
        present, but also by offline tests that must not need it, and the
        file's structure is a fixed two-level indent.
        """
        candidates = [Path(path)] if path else _SEARCH
        for p in candidates:
            try:
                if p.is_file():
                    return cls(_parse_joint_limits(p.read_text()))
            except OSError:
                continue
        return cls()

    def names(self):
        return list(self.table)

    def bounds(self):
        """name -> (lower, upper), the form joint_limit_margin() wants."""
        return {k: (v[0], v[1]) for k, v in self.table.items()}

    def distance_to_limit(self, name: str, q: float) -> float:
        lo, hi, *_ = self.table[name]
        return min(q - lo, hi - q)

    def max_velocity_at(self, name: str, q: float) -> float:
        """Permitted |velocity| for `name` at position `q`, in rad/s.

        This is the number MoveIt does not know. Negative distance (already
        past the limit) yields the bare offset rather than a NaN from the
        square root - a joint outside its range may still creep back.
        """
        lo, hi, v_nom, v_off, decel = self.table[name]
        d = max(0.0, min(q - lo, hi - q))
        return float(min(v_nom, v_off + np.sqrt(2.0 * decel * d)))

    def velocity_headroom(self, names, positions, velocities):
        """Worst ratio |velocity| / permitted over the given point.

        < 1.0 is legal. >= 1.0 is what libfranka rejects. Returns
        (ratio, joint_name, permitted, commanded) so the caller can put real
        numbers in the reason string rather than "trajectory rejected".
        """
        worst, who, perm, cmd = 0.0, "", 0.0, 0.0
        for name, q, dq in zip(names, positions, velocities):
            if name not in self.table:
                continue
            allowed = self.max_velocity_at(name, q)
            ratio = abs(dq) / allowed if allowed > 0 else float("inf")
            if ratio > worst:
                worst, who, perm, cmd = ratio, name, allowed, abs(dq)
        return worst, who, perm, cmd

    def check_trajectory(self, joint_traj, safety: float = 0.9):
        """Validate a JointTrajectory against the position-based limits.

        `safety` is the fraction of the permitted velocity we allow, leaving
        headroom for the controller's own tracking error - libfranka rejects
        at the limit, not near it.

        Returns (ok, reason, worst_ratio). `reason` names the joint, its
        position, what it was permitted and what was commanded, because that
        text ends up in the ExecuteSkill message the planner reflects on.
        """
        names = list(joint_traj.joint_names)
        worst, who, perm, cmd, at_q, idx = 0.0, "", 0.0, 0.0, 0.0, -1

        for i, pt in enumerate(joint_traj.points):
            if not pt.velocities:
                continue
            r, w, p, c = self.velocity_headroom(names, pt.positions, pt.velocities)
            if r > worst:
                worst, who, perm, cmd, idx = r, w, p, c
                at_q = pt.positions[names.index(w)] if w in names else 0.0

        if worst <= safety:
            return True, (f"trajectory respects the FR3 position-based velocity "
                          f"limits (worst {worst * 100:.0f}% of permitted)"), worst

        return False, (
            f"trajectory would violate the FR3's position-dependent velocity "
            f"limit: at waypoint {idx}, {who} sits at {at_q:+.3f} rad, "
            f"{self.distance_to_limit(who, at_q):.3f} rad from its limit, where "
            f"only {perm:.3f} rad/s is permitted - but {cmd:.3f} rad/s is "
            f"commanded ({worst * 100:.0f}% of the limit). MoveIt plans against "
            f"the constant URDF velocity and cannot see this limit, so the "
            f"controller would fault rather than the planner"), worst

    def retime_factor_for(self, joint_traj, safety: float = 0.9):
        """Slow-down factor that would bring `joint_traj` inside the limits.

        Velocity scales linearly with the time reparameterisation, so the
        needed factor is just safety/worst_ratio. Returns 1.0 when the
        trajectory is already legal, and None when no slow-down helps -
        which happens when a joint is so close to its limit that even the
        offset term cannot cover the motion, i.e. the CONFIGURATION is
        wrong and re-timing cannot rescue it.
        """
        ok, _, worst = self.check_trajectory(joint_traj, safety)
        if ok:
            return 1.0
        if not np.isfinite(worst) or worst <= 0.0:
            return None
        return float(safety / worst)


def _parse_joint_limits(text: str):
    """Minimal reader for franka_description's joint_limits.yaml layout."""
    table, cur, vals = {}, None, {}

    def flush():
        if cur and {"lower", "upper", "velocity"} <= vals.keys():
            table[f"fr3_{cur}"] = (
                vals["lower"], vals["upper"], vals["velocity"],
                vals.get("velocity_offset", 0.0),
                vals.get("deceleration_limit", FR3_MAX_JOINT_ACCEL),
            )

    for raw in text.splitlines():
        line = raw.split("#")[0].rstrip()
        if not line.strip():
            continue
        if not line.startswith(" ") and line.rstrip().endswith(":"):
            flush()
            cur, vals = line.strip().rstrip(":"), {}
            continue
        if ":" in line:
            key, _, val = line.strip().partition(":")
            val = val.strip()
            if val:
                try:
                    vals[key.strip()] = float(val)
                except ValueError:
                    pass
    flush()
    return table or None
