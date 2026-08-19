"""
pragmabot_bridge.bridge_node

Action-client bridge: receives a skill decision from the PragmaBot planner
and dispatches it against Container 1's live interfaces:
  - /move_action              (moveit_msgs/action/MoveGroup)
  - /compute_cartesian_path   (moveit_msgs/srv/GetCartesianPath)
  - /execute_trajectory       (moveit_msgs/action/ExecuteTrajectory)
  - /franka_gripper/grasp     (franka_msgs/action/Grasp)
  - /franka_gripper/homing    (franka_msgs/action/Homing)
  - /franka_gripper/move      (franka_msgs/action/Move)

Never targets /fr3_gripper/gripper_action - dead stub, silently hangs.

The Franka Hand driver requires a Homing call after connecting (or after
any Grasp/Move fault) before it will reliably respond again - Desk shows
this as "End Effector: Not connected". execute_pick() homes once up front
and again (with one retry) if a Grasp call fails, rather than leaving the
gripper stuck for the next attempt.

execute_pick() implements: load top-confidence GraspGen pose (camera
frame, saved via graspgen_client.py --save_grasps) -> build a pre-grasp
standoff along GraspGen's approach axis -> transform both into fr3_link0
via the live TF tree -> MoveGroup to the standoff -> Cartesian approach
into the grasp -> close the gripper -> Cartesian lift retreat -> if
`place_after_s` > 0, wait, then lower back to the same grasp pose, open
the gripper, and retreat again (place-in-place, for repeatable demo runs).

Frame chain assumed live (verify with `ros2 run tf2_ros tf2_echo
fr3_link0 zed_left_camera_frame_optical` before trusting any transformed
pose): fr3_link0 -> zed_camera_link [easy_handeye2] -> zed_camera_center
-> zed_left_camera_frame -> zed_left_camera_frame_optical [ZED wrapper].
tf2 composes this in a single lookup_transform call.

execute_place() / execute_push() are not part of this pass - still
NotImplementedError stubs.
"""

import time

import numpy as np
import rclpy
from franka_msgs.action import Grasp, Homing, Move
from franka_msgs.msg import GraspEpsilon
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
)
from moveit_msgs.srv import GetCartesianPath
from rclpy.action import ActionClient
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener

from pragmabot_bridge import grasp_transform


class PragmabotBridge(Node):
    """Routes planner skill decisions to Container 1's action servers."""

    def __init__(self):
        super().__init__("pragmabot_bridge")

        self._move_client = ActionClient(self, MoveGroup, "/move_action")
        self._execute_client = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")
        self._gripper_client = ActionClient(self, Grasp, "/franka_gripper/grasp")
        self._homing_client = ActionClient(self, Homing, "/franka_gripper/homing")
        self._gripper_move_client = ActionClient(self, Move, "/franka_gripper/move")
        self._cartesian_client = self.create_client(
            GetCartesianPath, "/compute_cartesian_path"
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.declare_parameter("grasp_file", "")
        self.declare_parameter("group_name", "fr3_arm")
        self.declare_parameter("eef_link", "fr3_hand")
        self.declare_parameter("standoff_m", 0.12)
        self.declare_parameter("lift_m", 0.12)
        self.declare_parameter("gripper_width", 0.0)
        self.declare_parameter("gripper_speed", 0.05)
        self.declare_parameter("gripper_force", 20.0)
        self.declare_parameter("gripper_epsilon", 0.02)
        self.declare_parameter("camera_frame", "zed_left_camera_frame_optical")
        self.declare_parameter("object_pcd_file", "")
        self.declare_parameter("grasp_index", -1)
        self.declare_parameter("revolute_jump_threshold", 0.2)
        self.declare_parameter("place_after_s", 0.0)
        self.declare_parameter("gripper_open_width", 0.08)
        self.declare_parameter("home_gripper_first", True)

        self.get_logger().info(
            "pragmabot_bridge started - waiting for /move_action, "
            "/execute_trajectory, /compute_cartesian_path and "
            "/franka_gripper/grasp servers (expected to be unavailable "
            "unless Container 1's MoveIt is launched on the same "
            "ROS_DOMAIN_ID)."
        )

    # ------------------------------------------------------------------
    # execute_pick
    # ------------------------------------------------------------------

    def execute_pick(
        self,
        grasp_file: str,
        group_name: str = "fr3_arm",
        eef_link: str = "fr3_hand",
        standoff_m: float = 0.12,
        lift_m: float = 0.12,
        gripper_width: float = 0.0,
        gripper_speed: float = 0.05,
        gripper_force: float = 20.0,
        gripper_epsilon: float = 0.02,
        camera_frame: str = "zed_left_camera_frame_optical",
        object_pcd_file: str = "",
        grasp_index: int = -1,
        place_after_s: float = 0.0,
        gripper_open_width: float = 0.08,
        home_gripper_first: bool = True,
    ) -> bool:
        """Run one pick attempt using a grasp from `grasp_file`.

        `grasp_index` selects which of the saved grasps to use. -1 (default)
        auto-picks the most top-down candidate: the one whose approach axis
        is closest to straight down / perpendicular to the table in
        fr3_link0 (see grasp_transform.select_topdown_index) - this is a
        robot-frame comparison, done AFTER transforming all candidates, not
        in camera frame, since the camera's own tilt is arbitrary. Pass a
        non-negative index (0..N-1, N = grasps saved in grasp_file) to
        override with a specific one instead.

        `eef_link` must match whatever link GraspGen's origin convention
        (gripper base/root link, not fingertip/TCP) corresponds to in this
        MoveIt setup - NOT verified live for this project. Confirm the
        planning group's actual tip link (RViz MotionPlanning panel, or
        `ros2 param get /move_group ...` once MoveIt is up) before trusting
        `fr3_hand` here; some franka_ros2 setups use `fr3_hand_tcp` instead.

        `gripper_width` is a real closed-gripper target width in meters.
        If left at the 0.0 placeholder AND `object_pcd_file` is given (the
        same object_pcd.npy passed as --pcd_file to graspgen_client.py),
        it's auto-estimated from the point cloud at the actual grasp
        contact location via grasp_transform.estimate_gripper_width() -
        this matters because a non-uniform object (e.g. a cup) can be a
        very different width at the rim than at the body, and the grasp
        pose determines which one the fingers will actually land on, not
        a single whole-object measurement. If `object_pcd_file` isn't
        given either, 0.0 falls through as "close fully", correct only
        for an object that fully occludes the fingers before they meet.

        `place_after_s`: if > 0, after the pick+lift succeeds, wait this
        many seconds, then lower back to the exact grasp pose, open the
        gripper (Move action, not Grasp - no object-contact force
        expected on release), and retreat again. Places the object back
        where it was picked up, for repeatable demo/test cycles.

        `home_gripper_first`: call Homing before anything else. The
        Franka Hand driver needs this after connecting or after any prior
        Grasp/Move fault, or it goes unresponsive (Desk shows "End
        Effector: Not connected") - this is the fix for that, not a
        network/Docker issue. Adds a few seconds (the hand fully opens
        and closes to calibrate) - set False to skip if you've already
        homed this session and want faster iteration.
        """
        if home_gripper_first and not self._home_gripper():
            self.get_logger().error(
                "Gripper homing failed - Desk likely still shows the end "
                "effector as not connected/faulted. Check Desk directly "
                "before retrying; nothing past this point will work."
            )
            return False

        grasps_T_cam = grasp_transform.load_all_grasps(grasp_file)

        try:
            self._wait_for_transform("fr3_link0", camera_frame)
            stamped = self._tf_buffer.lookup_transform(
                "fr3_link0", camera_frame, rclpy.time.Time()
            )
            T_base_from_cam = grasp_transform.transform_to_matrix(stamped)
        except Exception as exc:  # noqa: BLE001 - report and abort, don't guess
            self.get_logger().error(f"TF lookup fr3_link0 <- {camera_frame} failed: {exc}")
            return False
        grasps_T_base = T_base_from_cam @ grasps_T_cam

        if grasp_index < 0:
            grasp_index = grasp_transform.select_topdown_index(grasps_T_base)
            self.get_logger().info(
                f"Auto-selected grasp index {grasp_index}/{len(grasps_T_cam)} "
                "as the most top-down candidate"
            )
        elif grasp_index >= len(grasps_T_cam):
            self.get_logger().error(
                f"grasp_index={grasp_index} out of range (grasp_file has "
                f"{len(grasps_T_cam)} grasps) - aborting"
            )
            return False

        grasp_T_cam = grasps_T_cam[grasp_index]
        grasp_T_base = grasps_T_base[grasp_index]
        standoff_T_base = grasp_transform.standoff_pose(grasp_T_base, standoff_m)

        if gripper_width <= 0.0 and object_pcd_file:
            pcd_cam = np.load(object_pcd_file).astype(np.float64)[:, :3]
            gripper_width = grasp_transform.estimate_gripper_width(pcd_cam, grasp_T_cam)
            self.get_logger().info(
                f"Auto-estimated gripper_width={gripper_width:.4f} m from "
                f"{object_pcd_file} at the grasp contact location"
            )
        elif gripper_width <= 0.0:
            self.get_logger().warn(
                "gripper_width is 0.0 and no object_pcd_file given - this "
                "will close the gripper fully rather than to the object's "
                "actual width."
            )

        self.get_logger().info(f"standoff pose (fr3_link0):\n{standoff_T_base}")
        self.get_logger().info(f"grasp pose (fr3_link0):\n{grasp_T_base}")

        # Step 1: MoveGroup to the pre-grasp standoff.
        standoff_pose_msg = grasp_transform.matrix_to_pose(standoff_T_base)
        if not self._move_to_pose(group_name, eef_link, standoff_pose_msg):
            self.get_logger().error("Failed to reach standoff pose - aborting")
            return False

        # Step 2: straight-line Cartesian approach into the grasp pose.
        grasp_pose_msg = grasp_transform.matrix_to_pose(grasp_T_base)
        cart = self._compute_cartesian_path(group_name, eef_link, [grasp_pose_msg])
        if cart is None or cart.fraction < 1.0:
            frac = None if cart is None else cart.fraction
            self.get_logger().error(
                f"Cartesian approach incomplete (fraction={frac}) - aborting. "
                "A fraction well below 1.0 (rather than exactly 0.0 from a "
                "service failure) usually means revolute_jump_threshold "
                "truncated the path due to a large single-joint jump "
                "between steps - a likely sign the straight-line approach "
                "passes near a singularity for this grasp pose."
            )
            return False
        if not self._execute_trajectory(cart.solution):
            self.get_logger().error("Cartesian approach execution failed - aborting")
            return False

        # Step 3: close the gripper. One retry after re-homing, since a
        # failed Grasp is exactly what leaves the driver unresponsive for
        # the next call (Desk: "End Effector: Not connected") until homed
        # again.
        if not self._grasp(gripper_width, gripper_speed, gripper_force, gripper_epsilon):
            self.get_logger().warn("Grasp failed - re-homing and retrying once")
            if not self._home_gripper() or not self._grasp(
                gripper_width, gripper_speed, gripper_force, gripper_epsilon
            ):
                self.get_logger().error("Gripper grasp failed after retry - aborting retreat")
                return False

        # Step 4: straight-line lift along fr3_link0's own +Z (not the
        # grasp's own -Z) - less likely to re-collide with the table than
        # retracing the approach axis.
        retreat_T_base = grasp_T_base.copy()
        retreat_T_base[2, 3] += lift_m
        retreat_pose_msg = grasp_transform.matrix_to_pose(retreat_T_base)
        cart = self._compute_cartesian_path(group_name, eef_link, [retreat_pose_msg])
        if cart is None or cart.fraction < 1.0:
            frac = None if cart is None else cart.fraction
            self.get_logger().error(f"Retreat path incomplete (fraction={frac})")
            return False
        if not self._execute_trajectory(cart.solution):
            self.get_logger().error("Retreat execution failed")
            return False

        self.get_logger().info("Pick sequence complete")

        if place_after_s > 0.0:
            self.get_logger().info(f"Waiting {place_after_s:.1f}s before placing back down")
            time.sleep(place_after_s)

            # Step 5: straight-line descend back to the exact grasp pose.
            cart = self._compute_cartesian_path(group_name, eef_link, [grasp_pose_msg])
            if cart is None or cart.fraction < 1.0:
                frac = None if cart is None else cart.fraction
                self.get_logger().error(f"Place descent incomplete (fraction={frac})")
                return False
            if not self._execute_trajectory(cart.solution):
                self.get_logger().error("Place descent execution failed")
                return False

            # Step 6: open the gripper to release - Move, not Grasp, since
            # no object-contact force is expected on release.
            if not self._open_gripper(gripper_open_width, gripper_speed):
                self.get_logger().error("Gripper open (place) failed")
                return False

            # Step 7: retreat again, same as step 4.
            cart = self._compute_cartesian_path(group_name, eef_link, [retreat_pose_msg])
            if cart is None or cart.fraction < 1.0:
                frac = None if cart is None else cart.fraction
                self.get_logger().error(f"Post-place retreat incomplete (fraction={frac})")
                return False
            if not self._execute_trajectory(cart.solution):
                self.get_logger().error("Post-place retreat execution failed")
                return False

            self.get_logger().info("Place-back-in-place sequence complete")

        return True

    def execute_place(self, target_location: str):
        raise NotImplementedError

    def execute_push(self, target_object: str, goal_region: str):
        raise NotImplementedError

    # ------------------------------------------------------------------
    # MoveIt goal helpers
    # ------------------------------------------------------------------

    def _wait_for_transform(self, target_frame: str, source_frame: str, timeout_sec: float = 10.0):
        start = time.time()
        while not self._tf_buffer.can_transform(target_frame, source_frame, rclpy.time.Time()):
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > timeout_sec:
                raise RuntimeError(
                    f"TF {target_frame} <- {source_frame} not available after "
                    f"{timeout_sec}s (is easy_handeye2 publish.launch.py running, "
                    "and is the ZED wrapper's robot_state_publisher up?)"
                )

    def _pose_goal_constraints(
        self, link_name: str, pose: Pose, pos_tol: float = 0.01, ori_tol: float = 0.02
    ) -> Constraints:
        header = Header()
        header.frame_id = "fr3_link0"
        header.stamp = self.get_clock().now().to_msg()

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [pos_tol]

        region_pose = Pose()
        region_pose.position = pose.position
        region_pose.orientation.w = 1.0

        bounding_volume = BoundingVolume()
        bounding_volume.primitives.append(primitive)
        bounding_volume.primitive_poses.append(region_pose)

        position_constraint = PositionConstraint()
        position_constraint.header = header
        position_constraint.link_name = link_name
        position_constraint.constraint_region = bounding_volume
        position_constraint.weight = 1.0

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header = header
        orientation_constraint.link_name = link_name
        orientation_constraint.orientation = pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = ori_tol
        orientation_constraint.absolute_y_axis_tolerance = ori_tol
        orientation_constraint.absolute_z_axis_tolerance = ori_tol
        orientation_constraint.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(position_constraint)
        constraints.orientation_constraints.append(orientation_constraint)
        return constraints

    def _move_to_pose(self, group_name: str, link_name: str, pose: Pose) -> bool:
        goal = MoveGroup.Goal()
        goal.request.group_name = group_name
        goal.request.goal_constraints.append(self._pose_goal_constraints(link_name, pose))
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.2
        goal.request.max_acceleration_scaling_factor = 0.2
        goal.planning_options.plan_only = False

        result = self._send_goal_blocking(self._move_client, goal, "MoveGroup")
        if result is None:
            return False
        ok = result.result.error_code.val == MoveItErrorCodes.SUCCESS
        if not ok:
            self.get_logger().error(
                f"MoveGroup failed, error_code={result.result.error_code.val}"
            )
        return ok

    def _compute_cartesian_path(self, group_name: str, link_name: str, waypoints: list[Pose]):
        request = GetCartesianPath.Request()
        request.header.frame_id = "fr3_link0"
        request.header.stamp = self.get_clock().now().to_msg()
        request.group_name = group_name
        request.link_name = link_name
        request.waypoints = waypoints
        request.max_step = 0.01
        request.avoid_collisions = True
        # Truncate the path (rather than execute through it) if any single
        # max_step-sized Cartesian step would require an unusually large
        # single-joint angle change -- the direct symptom of the IK
        # solution passing near a singularity. 0.2 rad (~11 deg) per 1cm
        # Cartesian step is a conservative starting point, not empirically
        # tuned for this robot/workspace -- loosen or tighten via the
        # revolute_jump_threshold parameter if it aborts on fine motions
        # or doesn't catch a real one.
        request.revolute_jump_threshold = self.get_parameter("revolute_jump_threshold").value

        self._cartesian_client.wait_for_service()
        future = self._cartesian_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def _execute_trajectory(self, robot_trajectory) -> bool:
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = robot_trajectory
        result = self._send_goal_blocking(self._execute_client, goal, "ExecuteTrajectory")
        if result is None:
            return False
        return result.result.error_code.val == MoveItErrorCodes.SUCCESS

    def _grasp(self, width: float, speed: float, force: float, epsilon: float) -> bool:
        goal = Grasp.Goal()
        goal.width = width
        goal.speed = speed
        goal.force = force
        goal.epsilon = GraspEpsilon(inner=epsilon, outer=epsilon)

        result = self._send_goal_blocking(self._gripper_client, goal, "Grasp")
        if result is None:
            return False
        if not result.result.success:
            self.get_logger().error(f"Grasp reported failure: {result.result.error}")
            return False
        return True

    def _home_gripper(self) -> bool:
        result = self._send_goal_blocking(self._homing_client, Homing.Goal(), "Homing")
        if result is None:
            return False
        if not result.result.success:
            self.get_logger().error(f"Homing reported failure: {result.result.error}")
            return False
        return True

    def _open_gripper(self, width: float, speed: float) -> bool:
        goal = Move.Goal()
        goal.width = width
        goal.speed = speed
        result = self._send_goal_blocking(self._gripper_move_client, goal, "Move")
        if result is None:
            return False
        if not result.result.success:
            self.get_logger().error(f"Gripper open (Move) reported failure: {result.result.error}")
            return False
        return True

    def _send_goal_blocking(self, client: ActionClient, goal_msg, name: str):
        client.wait_for_server()
        send_future = client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"{name} goal rejected")
            return None
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result()


def main(args=None):
    rclpy.init(args=args)
    node = PragmabotBridge()

    grasp_file = node.get_parameter("grasp_file").get_parameter_value().string_value
    if grasp_file:
        success = node.execute_pick(
            grasp_file,
            group_name=node.get_parameter("group_name").value,
            eef_link=node.get_parameter("eef_link").value,
            standoff_m=node.get_parameter("standoff_m").value,
            lift_m=node.get_parameter("lift_m").value,
            gripper_width=node.get_parameter("gripper_width").value,
            gripper_speed=node.get_parameter("gripper_speed").value,
            gripper_force=node.get_parameter("gripper_force").value,
            gripper_epsilon=node.get_parameter("gripper_epsilon").value,
            camera_frame=node.get_parameter("camera_frame").value,
            object_pcd_file=node.get_parameter("object_pcd_file").get_parameter_value().string_value,
            grasp_index=node.get_parameter("grasp_index").value,
            place_after_s=node.get_parameter("place_after_s").value,
            gripper_open_width=node.get_parameter("gripper_open_width").value,
            home_gripper_first=node.get_parameter("home_gripper_first").value,
        )
        node.get_logger().info(f"execute_pick finished, success={success}")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
