#!/usr/bin/env python3
import sys
import numpy as np
import rospy
import actionlib
import ros_numpy
import moveit_commander
from franka_gripper.msg import MoveAction, MoveGoal, GraspAction, GraspGoal
from sensor_msgs.msg import Image
from message_filters import Subscriber, ApproximateTimeSynchronizer

sys.path.insert(0, "/catkin_ws/src/pragmabot/pragmabot/src")
from pragmabot.msg import PandaPickAction, PandaPickResult, PandaPickFeedback
from pragmabot.grounded_sam import GroundedSAM
from pragmabot.graspgen_client import GraspGenClient

# -- Constants (update OBSERVATION_JOINT_CONFIG after physical measurement) --
OBSERVATION_JOINT_CONFIG = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
GRIPPER_MAX_WIDTH = 0.08    # metres
GRIPPER_GRASP_FORCE = 20.0  # Newtons
LIFT_HEIGHT = 0.10          # metres
SYNC_SLOP = 1.0             # seconds for image sync


class PandaPickServer:

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.arm = moveit_commander.MoveGroupCommander("panda_arm")
        self.arm.set_max_velocity_scaling_factor(0.3)
        self.arm.set_max_acceleration_scaling_factor(0.2)

        self.move_client = actionlib.SimpleActionClient("/franka_gripper/move", MoveAction)
        self.grasp_client = actionlib.SimpleActionClient("/franka_gripper/grasp", GraspAction)
        rospy.loginfo("Waiting for gripper action servers...")
        self.move_client.wait_for_server()
        self.grasp_client.wait_for_server()

        self.gsam = GroundedSAM()
        self.graspgen = GraspGenClient()

        self._rgb_msg = None
        self._depth_msg = None
        rgb_sub = Subscriber("/zed2/zed_node/rgb/image_rect_color", Image)
        depth_sub = Subscriber("/zed2/zed_node/depth/depth_registered", Image)
        self._sync = ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=5, slop=SYNC_SLOP)
        self._sync.registerCallback(self._image_cb)

        self.server = actionlib.SimpleActionServer(
            "/panda/pick", PandaPickAction, execute_cb=self.execute_cb, auto_start=False
        )
        self.server.start()
        rospy.loginfo("PandaPickServer ready")

    def _image_cb(self, rgb_msg, depth_msg):
        self._rgb_msg = rgb_msg
        self._depth_msg = depth_msg

    def execute_cb(self, goal):
        try:
            self._run(goal)
        except RuntimeError as e:
            rospy.logerr("Pick failed: %s", e)
            self.server.set_aborted(PandaPickResult(success=False, message=str(e)))
        except Exception as e:
            rospy.logerr("Pick unexpected error: %s", e)
            self.server.set_aborted(PandaPickResult(success=False, message=str(e)))

    def _run(self, goal):
        self._send_feedback("Opening gripper")
        self._open_gripper()

        self._send_feedback("Moving to observation pose")
        self._move_to_observation_pose()

        self._send_feedback("Capturing RGBD")
        rgb_msg, depth_msg = self._capture_rgbd()

        self._send_feedback("Segmenting object")
        rgb_np = ros_numpy.numpify(rgb_msg)[..., :3]   # H×W×3 BGR via cv_bridge convention
        import cv2
        rgb_bgr = cv2.cvtColor(rgb_np, cv2.COLOR_RGB2BGR)
        mask, conf = self.gsam.segment(rgb_bgr, goal.target_object)
        if conf < 0.25:
            raise RuntimeError("GroundedSAM confidence too low (%.2f) for '%s'" % (conf, goal.target_object))
        rospy.loginfo("SAM confidence: %.3f, mask pixels: %d", conf, mask.sum())

        self._send_feedback("Building point cloud")
        pc = self._mask_to_pointcloud(mask, depth_msg)
        if len(pc) < 10:
            raise RuntimeError("Point cloud too sparse (%d points)" % len(pc))

        self._send_feedback("Generating grasps")
        grasps = self.graspgen.generate(pc - pc.mean(axis=0), topk=8)
        if not grasps:
            raise RuntimeError("GraspGen returned no grasps")

        self._send_feedback("Planning and executing grasp")
        executed = False
        for g in grasps:
            traj = self._plan_to_pose(g["pose"])
            if traj is not None:
                self.arm.execute(traj, wait=True)
                executed = True
                break
        if not executed:
            raise RuntimeError("No IK-feasible grasp found")

        self._send_feedback("Closing gripper")
        self._close_gripper()

        self._send_feedback("Lifting")
        self._lift()

        result = PandaPickResult(success=True, message="pick succeeded")
        self.server.set_succeeded(result)

    # ------------------------------------------------------------------ helpers

    def _open_gripper(self):
        goal = MoveGoal(width=GRIPPER_MAX_WIDTH, speed=0.05)
        self.move_client.send_goal_and_wait(goal, rospy.Duration(10.0))

    def _close_gripper(self):
        goal = GraspGoal(width=0.0, speed=0.03, force=GRIPPER_GRASP_FORCE)
        goal.epsilon.inner = 0.03
        goal.epsilon.outer = 0.03
        self.grasp_client.send_goal_and_wait(goal, rospy.Duration(10.0))

    def _move_to_observation_pose(self):
        self.arm.set_joint_value_target(OBSERVATION_JOINT_CONFIG)
        plan = self.arm.plan()
        traj = plan[1] if isinstance(plan, tuple) else plan
        if not self.arm.execute(traj, wait=True):
            raise RuntimeError("Failed to move to observation pose")
        self.arm.stop()

    def _capture_rgbd(self, timeout=5.0):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(20)
        while rospy.Time.now() < deadline:
            if self._rgb_msg is not None and self._depth_msg is not None:
                return self._rgb_msg, self._depth_msg
            rate.sleep()
        raise RuntimeError("Timed out waiting for RGBD image pair")

    def _mask_to_pointcloud(self, mask, depth_msg):
        depth = ros_numpy.numpify(depth_msg)  # H×W float32, metres
        ys, xs = np.where(mask & np.isfinite(depth) & (depth > 0.1) & (depth < 2.0))
        if len(xs) == 0:
            return np.zeros((0, 3), dtype=np.float32)
        zs = depth[ys, xs]
        # Back-project using ZED2 default approx intrinsics; replaced by real TF in Phase 7
        fx = fy = 700.0
        cx = depth.shape[1] / 2.0
        cy = depth.shape[0] / 2.0
        xs_m = (xs - cx) * zs / fx
        ys_m = (ys - cy) * zs / fy
        return np.stack([xs_m, ys_m, zs], axis=1).astype(np.float32)

    def _plan_to_pose(self, pose_stamped):
        self.arm.set_pose_target(pose_stamped)
        plan = self.arm.plan()
        traj = plan[1] if isinstance(plan, tuple) else plan
        self.arm.clear_pose_targets()
        if traj and len(traj.joint_trajectory.points) > 0:
            return traj
        return None

    def _lift(self):
        waypoints = []
        pose = self.arm.get_current_pose().pose
        pose.position.z += LIFT_HEIGHT
        waypoints.append(pose)
        traj, fraction = self.arm.compute_cartesian_path(waypoints, 0.01, 0.0)
        if fraction > 0.8:
            self.arm.execute(traj, wait=True)
        self.arm.stop()

    def _send_feedback(self, text):
        rospy.loginfo("[pick] %s", text)
        self.server.publish_feedback(PandaPickFeedback(status=text))


if __name__ == "__main__":
    rospy.init_node("panda_pick_server")
    PandaPickServer()
    rospy.spin()
