#!/usr/bin/env python3
"""Record the arm's current joint configuration as "home", and return to it.

Standalone - talks directly to /joint_states and /move_action, independent
of pragmabot_bridge and the planner's skill vocabulary (pick/place/push are
the only skills the protected VLM prompt files know about; this is a manual
between-attempts reset, not a fourth skill). Useful for putting the arm back
somewhere known after a pick that left it lifted holding (or not holding)
the object, without restarting Terminal A.

    python3 scripts/home_pose.py --record     # save the current joint pose
    python3 scripts/home_pose.py --go          # plan+execute back to it

Needs the same sourced environment as Terminal A (ROS_DOMAIN_ID, ROS 2
Humble, the pragmabot_bridge_ws overlay) and the robot stack (MoveGroup)
already up. Uses a JOINT-space goal, not a pose goal - unlike the standoff
move in bridge_node.py, there is no IK ambiguity to resolve here (the exact
joint target is already known from the recording), so this is not
susceptible to the "weird trajectory" failure mode a pose-space OMPL plan
can hit.
"""

import argparse
import json
import sys
from pathlib import Path

import rclpy
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState

FR3_JOINTS = [f"fr3_joint{i}" for i in range(1, 8)]
DEFAULT_FILE = Path.home() / ".ros2" / "pragmabot" / "home_pose.json"
GROUP_NAME = "fr3_arm"
JOINT_TOLERANCE_RAD = 0.01
VELOCITY_SCALING = 0.15
ACCEL_SCALING = 0.15
SERVER_TIMEOUT_S = 10.0
RESULT_TIMEOUT_S = 60.0
JOINT_STATE_TIMEOUT_S = 10.0


class HomePoseTool(Node):
    def __init__(self):
        super().__init__("pragmabot_home_pose_tool")
        self._move_client = ActionClient(self, MoveGroup, "/move_action")

    def record(self, out_file: Path) -> bool:
        """Block until one /joint_states message has all 7 fr3 joints, save it."""
        received = {}

        def cb(msg: JointState):
            for name, pos in zip(msg.name, msg.position):
                if name in FR3_JOINTS:
                    received[name] = pos

        sub = self.create_subscription(JointState, "/joint_states", cb, 10)
        self.get_logger().info("Waiting for /joint_states ...")
        deadline = self.get_clock().now().nanoseconds + int(JOINT_STATE_TIMEOUT_S * 1e9)
        try:
            while rclpy.ok() and len(received) < len(FR3_JOINTS):
                rclpy.spin_once(self, timeout_sec=0.5)
                if self.get_clock().now().nanoseconds > deadline:
                    missing = sorted(set(FR3_JOINTS) - set(received))
                    self.get_logger().error(
                        f"Only saw {sorted(received)} on /joint_states within "
                        f"{JOINT_STATE_TIMEOUT_S:.0f}s - missing {missing}. "
                        "Is the robot stack (ros2_control_node) running?"
                    )
                    return False
        finally:
            self.destroy_subscription(sub)

        positions = {name: received[name] for name in FR3_JOINTS}
        out_file.parent.mkdir(parents=True, exist_ok=True)
        out_file.write_text(json.dumps(positions, indent=2))
        self.get_logger().info(f"Saved home pose to {out_file}:")
        for name in FR3_JOINTS:
            self.get_logger().info(f"  {name}: {positions[name]:+.4f} rad")
        return True

    def go_home(self, in_file: Path) -> bool:
        if not in_file.exists():
            self.get_logger().error(f"{in_file} does not exist - run with --record first")
            return False
        positions = json.loads(in_file.read_text())
        missing = [n for n in FR3_JOINTS if n not in positions]
        if missing:
            self.get_logger().error(f"{in_file} is missing joints: {missing}")
            return False

        constraints = Constraints()
        for name in FR3_JOINTS:
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(positions[name])
            jc.tolerance_above = JOINT_TOLERANCE_RAD
            jc.tolerance_below = JOINT_TOLERANCE_RAD
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        goal = MoveGroup.Goal()
        goal.request.group_name = GROUP_NAME
        goal.request.goal_constraints.append(constraints)
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = VELOCITY_SCALING
        goal.request.max_acceleration_scaling_factor = ACCEL_SCALING
        goal.planning_options.plan_only = False

        if not self._move_client.wait_for_server(timeout_sec=SERVER_TIMEOUT_S):
            self.get_logger().error(
                f"/move_action did not appear within {SERVER_TIMEOUT_S:.0f}s - "
                "is MoveIt running on this ROS_DOMAIN_ID?"
            )
            return False

        send_future = self._move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=SERVER_TIMEOUT_S)
        goal_handle = send_future.result() if send_future.done() else None
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("MoveGroup goal rejected or not acknowledged")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=RESULT_TIMEOUT_S)
        if not result_future.done():
            self.get_logger().error(f"No result within {RESULT_TIMEOUT_S:.0f}s - cancelling")
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=5.0)
            return False

        result = result_future.result()
        ok = result.result.error_code.val == MoveItErrorCodes.SUCCESS
        if not ok:
            self.get_logger().error(f"MoveGroup failed, error_code={result.result.error_code.val}")
        else:
            self.get_logger().info("Reached home pose")
        return ok


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--record", action="store_true", help="save the current joint pose as home")
    mode.add_argument("--go", action="store_true", help="plan+execute to the saved home pose")
    parser.add_argument(
        "--file", type=Path, default=DEFAULT_FILE,
        help=f"home pose file (default: {DEFAULT_FILE})",
    )
    args = parser.parse_args()

    rclpy.init()
    node = HomePoseTool()
    try:
        ok = node.record(args.file) if args.record else node.go_home(args.file)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
