"""
pragmabot_bridge.bridge_node

Action-client bridge: receives a skill decision from the PragmaBot planner
and dispatches it against Container 1's live, confirmed interfaces:
  - /move_action              (moveit_msgs/action/MoveGroup)
  - /compute_cartesian_path   (moveit_msgs/srv/GetCartesianPath)
  - /franka_gripper/grasp     (franka_msgs/action/Grasp)

Never targets /fr3_gripper/gripper_action — dead stub, silently hangs.

STATUS: structural skeleton only. The actual goal-construction logic
(grasp pose -> MoveGroup goal, etc.) depends on GraspGen/GroundedSAM output,
which isn't available until those tools are built on Alonnisos. This file
is meant to be extended there, not completed here.

This node can be built and launched tonight against ROS2 Humble alone —
it will simply sit waiting for the action servers (expected, since
Container 1 isn't running on a laptop). That waiting behavior is itself
a useful smoke test: if the node starts and logs "waiting for server"
without crashing, the ROS2 plumbing is correct.
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from franka_msgs.action import Grasp
from moveit_msgs.action import MoveGroup

# TODO: once the open decision in session_prompt.md is settled, import the
# actual interface here, e.g.:
# from pragmabot_interfaces.action import ExecuteSkill


class PragmabotBridge(Node):
    """Routes planner skill decisions to Container 1's action servers."""

    def __init__(self):
        super().__init__("pragmabot_bridge")

        self._move_client = ActionClient(self, MoveGroup, "/move_action")
        self._gripper_client = ActionClient(self, Grasp, "/franka_gripper/grasp")

        self.get_logger().info(
            "pragmabot_bridge started — waiting for /move_action and "
            "/franka_gripper/grasp servers (expected to be unavailable "
            "unless Container 1 is running on the same ROS_DOMAIN_ID)."
        )

    # ------------------------------------------------------------------
    # TODO (fill in on Alonnisos, once GraspGen/GroundedSAM are wired up):
    #
    # def execute_pick(self, target_object: str):
    #     """
    #     1. Trigger observation-pose move via self._move_client
    #     2. Capture RGBD (subscribe to ZED topics from Container 2)
    #     3. Call GroundedSAM -> mask
    #     4. mask + depth -> point cloud -> call GraspGen -> SE(3) grasp pose
    #     5. Build MoveGroup goal for the grasp pose, send via self._move_client
    #     6. On success, send Grasp goal via self._gripper_client
    #     7. Lift, return result to planner
    #     """
    #     raise NotImplementedError
    #
    # def execute_place(self, target_location: str):
    #     raise NotImplementedError
    #
    # def execute_push(self, target_object: str, goal_region: str):
    #     raise NotImplementedError
    # ------------------------------------------------------------------


def main(args=None):
    rclpy.init(args=args)
    node = PragmabotBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
