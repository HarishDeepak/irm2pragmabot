#!/usr/bin/env python3
import sys
import numpy as np
import rospy
import actionlib
import ros_numpy
import moveit_commander
from franka_gripper.msg import MoveAction, MoveGoal
from sensor_msgs.msg import Image
from message_filters import Subscriber, ApproximateTimeSynchronizer

sys.path.insert(0, "/catkin_ws/src/pragmabot/pragmabot/src")
from pragmabot.msg import PandaPlaceAction, PandaPlaceResult, PandaPlaceFeedback
from pragmabot.grounded_sam import GroundedSAM

OBSERVATION_JOINT_CONFIG = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
GRIPPER_MAX_WIDTH = 0.08
DESCEND_HEIGHT = 0.05   # metres to descend for place
RETREAT_HEIGHT = 0.10   # metres to retreat after place
SYNC_SLOP = 1.0


class PandaPlaceServer:

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.arm = moveit_commander.MoveGroupCommander("panda_arm")
        self.arm.set_max_velocity_scaling_factor(0.3)
        self.arm.set_max_acceleration_scaling_factor(0.2)

        self.move_client = actionlib.SimpleActionClient("/franka_gripper/move", MoveAction)
        rospy.loginfo("Waiting for gripper move server...")
        self.move_client.wait_for_server()

        self.gsam = GroundedSAM()

        self._rgb_msg = None
        self._depth_msg = None
        rgb_sub = Subscriber("/zed2/zed_node/rgb/image_rect_color", Image)
        depth_sub = Subscriber("/zed2/zed_node/depth/depth_registered", Image)
        self._sync = ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=5, slop=SYNC_SLOP)
        self._sync.registerCallback(self._image_cb)

        self.server = actionlib.SimpleActionServer(
            "/panda/place", PandaPlaceAction, execute_cb=self.execute_cb, auto_start=False
        )
        self.server.start()
        rospy.loginfo("PandaPlaceServer ready")

    def _image_cb(self, rgb_msg, depth_msg):
        self._rgb_msg = rgb_msg
        self._depth_msg = depth_msg

    def execute_cb(self, goal):
        try:
            self._run(goal)
        except RuntimeError as e:
            rospy.logerr("Place failed: %s", e)
            self.server.set_aborted(PandaPlaceResult(success=False, message=str(e)))
        except Exception as e:
            rospy.logerr("Place unexpected error: %s", e)
            self.server.set_aborted(PandaPlaceResult(success=False, message=str(e)))

    def _run(self, goal):
        self._send_feedback("Moving to observation pose")
        self._move_to_observation_pose()

        self._send_feedback("Capturing RGBD")
        rgb_msg, depth_msg = self._capture_rgbd()

        self._send_feedback("Segmenting receptacle")
        import cv2
        rgb_np = ros_numpy.numpify(rgb_msg)[..., :3]
        rgb_bgr = cv2.cvtColor(rgb_np, cv2.COLOR_RGB2BGR)
        location_label = goal.target_location or "plate"
        mask, conf = self.gsam.segment(rgb_bgr, location_label)
        if conf < 0.20:
            raise RuntimeError("GroundedSAM confidence too low (%.2f) for location '%s'" % (conf, location_label))

        self._send_feedback("Computing place position")
        place_point = self._mask_centroid_3d(mask, depth_msg)
        if place_point is None:
            raise RuntimeError("Could not compute 3D centroid for place location")

        from geometry_msgs.msg import PoseStamped
        approach_pose = PoseStamped()
        approach_pose.header.frame_id = "panda_link0"
        approach_pose.header.stamp = rospy.Time.now()
        approach_pose.pose.position.x = place_point[0]
        approach_pose.pose.position.y = place_point[1]
        approach_pose.pose.position.z = place_point[2] + 0.15  # approach from above
        # Fixed downward orientation
        approach_pose.pose.orientation.x = 1.0
        approach_pose.pose.orientation.y = 0.0
        approach_pose.pose.orientation.z = 0.0
        approach_pose.pose.orientation.w = 0.0

        self._send_feedback("Moving to approach pose")
        self.arm.set_pose_target(approach_pose)
        plan = self.arm.plan()
        traj = plan[1] if isinstance(plan, tuple) else plan
        self.arm.clear_pose_targets()
        if not traj or len(traj.joint_trajectory.points) == 0:
            raise RuntimeError("No IK solution for place approach pose")
        self.arm.execute(traj, wait=True)
        self.arm.stop()

        self._send_feedback("Descending to place")
        self._move_cartesian_z(-DESCEND_HEIGHT)

        self._send_feedback("Opening gripper")
        self.move_client.send_goal_and_wait(
            MoveGoal(width=GRIPPER_MAX_WIDTH, speed=0.05), rospy.Duration(10.0)
        )

        self._send_feedback("Retreating")
        self._move_cartesian_z(RETREAT_HEIGHT)

        self.server.set_succeeded(PandaPlaceResult(success=True, message="place succeeded"))

    # ------------------------------------------------------------------ helpers

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

    def _mask_centroid_3d(self, mask, depth_msg):
        depth = ros_numpy.numpify(depth_msg)
        ys, xs = np.where(mask & np.isfinite(depth) & (depth > 0.1) & (depth < 2.0))
        if len(xs) == 0:
            return None
        cx_px = float(xs.mean())
        cy_px = float(ys.mean())
        z = float(np.median(depth[ys, xs]))
        fx = fy = 700.0
        cx = depth.shape[1] / 2.0
        cy = depth.shape[0] / 2.0
        x = (cx_px - cx) * z / fx
        y = (cy_px - cy) * z / fy
        return np.array([x, y, z], dtype=np.float32)

    def _move_cartesian_z(self, dz):
        pose = self.arm.get_current_pose().pose
        pose.position.z += dz
        traj, fraction = self.arm.compute_cartesian_path([pose], 0.01, 0.0)
        if fraction > 0.8:
            self.arm.execute(traj, wait=True)
        self.arm.stop()

    def _send_feedback(self, text):
        rospy.loginfo("[place] %s", text)
        self.server.publish_feedback(PandaPlaceFeedback(status=text))


if __name__ == "__main__":
    rospy.init_node("panda_place_server")
    PandaPlaceServer()
    rospy.spin()
