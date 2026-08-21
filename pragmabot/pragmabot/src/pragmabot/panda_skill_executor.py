"""Skill executor: planner decision -> ROS 2 action goal.

Rewritten from the ROS 1 actionlib version. Sends one ExecuteSkill goal to
the bridge (pragmabot_bridge), which owns the MoveIt/gripper sequencing.

The class name is unchanged (`PandaSkillExecutor`) only so
pragmabot_node.py's import keeps working; this robot is an FR3, not a
Panda - see CLAUDE.md "Robot identity".

THREE REAL BUGS FIXED HERE. None of them were ever caught, because
config.yaml sets `rosbag_replay: true` and pragmabot_node.py:139-141 skips
the executor entirely in that mode - this code path had literally never
executed:

  1. `action.skill` -> AttributeError. NextBestAction has no `skill` field;
     it is `chosen_skill` (vlm_task_planner.py:107). The very first line of
     execute() would have crashed on the first real robot run.
  2. `getattr(action, "target_location", "")` -> silently "". There is no
     `target_location` field; it is `placement_object` (:116). The getattr
     default meant place/push would have sent an empty target and failed
     mysteriously rather than loudly.
  3. `elif skill == "done"` -> dead branch. RobotSkill only defines
     PUSH|PICK|PLACE (:73-79), so "done" was unreachable.

PROJECT_OVERVIEW.md section 6 documents the WRONG schema and is what led
to bugs 1 and 2. Code against vlm_task_planner.py, never against that doc.

Guard against recurrence: _require() below reads fields with getattr(...)
and NO silent default, raising if a field is missing. If the upstream
schema ever changes again, this fails on the first call with the field
name in the message instead of sending an empty string to the robot.
"""

import logging

import rclpy
from pragmabot_interfaces.action import ExecuteSkill
from rclpy.action import ActionClient
from rclpy.node import Node

logger = logging.getLogger(__name__)

ACTION_TIMEOUT_S = 120.0
SERVER_WAIT_S = 30.0

# RobotSkill (vlm_task_planner.py:73-79). Kept as a literal set so an
# unexpected value is reported rather than forwarded to the robot.
KNOWN_SKILLS = {"pick", "place", "push"}


def _result(success: bool, message: str) -> dict:
    """Build the dict pragmabot_node.py expects back from execute()."""
    return {"success": success, "pre_image": None, "post_image": None, "message": message}


class PandaSkillExecutor:
    """Sends the planner's chosen skill to the bridge as a ROS 2 action goal."""

    def __init__(self, node: Node) -> None:
        """Connect to the bridge's ExecuteSkill action server.

        Args:
            node: The live rclpy node. Required - rclpy has no implicit
                global node the way rospy did.

        Raises:
            RuntimeError: if the bridge server does not appear within
                SERVER_WAIT_S. Raised at construction rather than deferred
                to the first goal, so a missing bridge is caught at startup
                instead of mid-experiment.
        """
        self.node = node
        self._client = ActionClient(node, ExecuteSkill, "/pragmabot/execute_skill")

        logger.info("Waiting for /pragmabot/execute_skill action server...")
        if not self._client.wait_for_server(timeout_sec=SERVER_WAIT_S):
            raise RuntimeError(
                f"/pragmabot/execute_skill unavailable after {SERVER_WAIT_S}s. "
                "Is pragmabot_bridge running, on the same ROS_DOMAIN_ID=7? "
                "Check with: ros2 action list | grep execute_skill"
            )
        logger.info("Connected to /pragmabot/execute_skill")

    @staticmethod
    def _require(action, field: str):
        """Read a NextBestAction field, failing loudly if it is absent.

        Deliberately NOT getattr(action, field, "") - that silent default is
        exactly what turned bug 2 into an empty target instead of an error.
        """
        if not hasattr(action, field):
            raise AttributeError(
                f"NextBestAction has no field '{field}'. The upstream schema in "
                "vlm_task_planner.py changed - update this executor to match it "
                "(and do NOT trust PROJECT_OVERVIEW.md section 6, which is stale)."
            )
        return getattr(action, field)

    @staticmethod
    def _as_text(value) -> str:
        """Flatten an Optional[str] / Optional[Enum] field to a plain string.

        The planner leaves non-applicable fields as None (e.g. placement_object
        on a pick), and enum fields arrive as RobotSkill/PushDirection whose
        .value carries the wire string. ROS 2 string fields accept neither None
        nor an Enum, so both are normalised here.
        """
        if value is None:
            return ""
        return str(getattr(value, "value", value))

    def execute(self, action) -> dict:
        """Execute one planner action on the robot, blocking until it finishes.

        Args:
            action: NextBestAction pydantic object from VLMTaskPlanner.

        Returns:
            {"success": bool, "pre_image": None, "post_image": None,
             "message": str} - the shape pragmabot_node.py expects.
        """
        try:
            # `chosen_skill`, NOT `skill` - bug 1.
            skill = self._as_text(self._require(action, "chosen_skill")).lower()
        except AttributeError as exc:
            logger.error("%s", exc)
            return _result(False, str(exc))

        if skill not in KNOWN_SKILLS:
            # Reachable only if RobotSkill gains a member; "done" never was
            # a member, which is what made the old branch dead code (bug 3).
            return _result(False, f"unknown skill: {skill!r} (expected one of {sorted(KNOWN_SKILLS)})")

        goal = ExecuteSkill.Goal()
        goal.chosen_skill = skill
        goal.target_object = self._as_text(self._require(action, "target_object"))
        # `placement_object`, NOT `target_location` - bug 2.
        goal.placement_object = self._as_text(self._require(action, "placement_object"))
        goal.push_direction = self._as_text(self._require(action, "push_direction"))
        goal.should_grasp_at_specific_section = bool(
            self._require(action, "should_grasp_at_specific_section")
        )
        goal.should_place_at_specific_section = bool(
            self._require(action, "should_place_at_specific_section")
        )

        # Fail before moving the robot rather than after: a place with no
        # placement_object, or a push with no direction, is an underspecified
        # goal the bridge cannot act on.
        if skill == "place" and not goal.placement_object:
            return _result(False, "place action has empty placement_object - planner did not set it")
        if skill == "push" and not goal.push_direction:
            return _result(False, "push action has empty push_direction - planner did not set it")
        if not goal.target_object:
            return _result(False, f"{skill} action has empty target_object - planner did not set it")

        logger.info(
            "Sending %s goal: target=%r placement=%r push_dir=%r",
            skill,
            goal.target_object,
            goal.placement_object,
            goal.push_direction,
        )
        return self._send_and_wait(goal, skill)

    def _send_and_wait(self, goal, skill: str) -> dict:
        """Send one goal and block until result, rejection, or timeout."""
        send_future = self._client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        rclpy.spin_until_future_complete(self.node, send_future, timeout_sec=ACTION_TIMEOUT_S)

        goal_handle = send_future.result()
        if goal_handle is None:
            return _result(False, f"{skill}: no response to goal request within {ACTION_TIMEOUT_S}s")
        if not goal_handle.accepted:
            return _result(False, f"{skill}: goal rejected by the bridge")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=ACTION_TIMEOUT_S)

        wrapper = result_future.result()
        if wrapper is None:
            # Cancel so the arm does not keep executing a goal we stopped
            # waiting on - the ROS 1 version's cancel_goal() equivalent.
            goal_handle.cancel_goal_async()
            return _result(False, f"{skill} timed out after {ACTION_TIMEOUT_S}s (goal cancelled)")

        return _result(bool(wrapper.result.success), wrapper.result.message)

    @staticmethod
    def _feedback_cb(feedback_msg) -> None:
        logger.info("[skill] %s", feedback_msg.feedback.status)
