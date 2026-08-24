"""Step-size policy for straight-line (Cartesian) path planning.

Separate from bridge_node, and free of every ROS import, so the policy can
be checked without a robot or a sourced ROS environment
(scripts/test_cartesian_retry.py). bridge_node holds the service call; this
holds the decision about what to ask next when the call comes back short.
"""


def cartesian_step_schedule(max_step: float, min_step: float, max_tries: int):
    """eef_step values to try for one straight-line plan, coarsest first.

    Kept module-level and free of ROS so it can be checked without a robot
    (scripts/test_cartesian_retry.py). Each entry halves the previous one
    and the sequence stops at min_step - retrying with a step already at
    the floor would re-ask a question already answered.
    """
    max_step = max(float(max_step), 0.0)
    min_step = max(float(min_step), 0.0)
    if max_step <= 0.0:
        max_step = min_step
    if min_step > max_step:
        min_step = max_step

    steps = []
    step = max_step
    for _ in range(max(1, int(max_tries))):
        steps.append(step)
        if step <= min_step:
            break
        step = max(min_step, step / 2.0)
    return steps
