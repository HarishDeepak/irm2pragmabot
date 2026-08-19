#!/usr/bin/env python3
import sys
import numpy as np
import rospy
import actionlib
import ros_numpy
import moveit_commander
from sensor_msgs.msg import Image
from message_filters import Subscriber, ApproximateTimeSynchronizer
from geometry_msgs.msg import PoseStamped

sys.path.insert(0, "/catkin_ws/src/pragmabot/pragmabot/src")
from pragmabot.msg import PandaPushAction, PandaPushResult, PandaPushFeedback
from pragmabot.grounded_sam import GroundedSAM

OBSERVATION_JOINT_CONFIG = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
PUSH_APPROACH_OFFSET = 0.05  # metres behind object
PUSH_HEIGHT = 0.02           # metres above table during push (wrist height)
RETREAT_HEIGHT = 0.15
SYNC_SLOP = 1.0


class PandaPushServer:

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.arm = moveit_commander.MoveGroupCommander("panda_arm")
        self.arm.set_max_velocity_scaling_factor(0.2)
        self.arm.set_max_acceleration_scaling_factor(0.15)

        self.gsam = GroundedSAM()

        self._rgb_msg = None
        self._depth_msg = None
        rgb_sub = Subscriber("/zed2/zed_node/rgb/image_rect_color", Image)
        depth_sub = Subscriber("/zed2/zed_node/depth/depth_registered", Image)
        self._sync = ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=5, slop=SYNC_SLOP)
        self._sync.registerCallback(self._image_cb)

        self.server = actionlib.SimpleActionServer(
            "/panda/push", PandaPushAction, execute_cb=self.execute_cb, auto_start=False
        )
        self.server.start()
        rospy.loginfo("PandaPushServer ready")

    def _image_cb(self, rgb_msg, depth_msg):
        self._rgb_msg = rgb_msg
        self._depth_msg = depth_msg

    def execute_cb(self, goal):
        try:
            self._run(goal)
        except RuntimeError as e:
            rospy.logerr("Push failed: %s", e)
            self.server.set_aborted(PandaPushResult(success=False, message=str(e)))
        except Exception as e:
            rospy.logerr("Push unexpected error: %s", e)
            self.server.set_aborted(PandaPushResult(success=False, message=str(e)))

    def _run(self, goal):
        self._send_feedback("Moving to observation pose")
        self._move_to_observation_pose()

        self._send_feedback("Capturing RGBD")
        rgb_msg, depth_msg = self._capture_rgbd()

        import cv2
        rgb_np = ros_numpy.numpify(rgb_msg)[..., :3]
        rgb_bgr = cv2.cvtColor(rgb_np, cv2.COLOR_RGB2BGR)
        depth = ros_numpy.numpify(depth_msg)

        self._send_feedback("Segmenting object and goal region")
        mask_obj, conf_obj = self.gsam.segment(rgb_bgr, goal.target_object)
        if conf_obj < 0.25:
            raise RuntimeError("Low confidence (%.2f) for object '%s'" % (conf_obj, goal.target_object))

        obj_pt = self._mask_centroid_3d(mask_obj, depth)
        if obj_pt is None:
            raise RuntimeError("Could not compute 3D centroid for object")

        mask_goal, conf_goal = self.gsam.segment(rgb_bgr, goal.goal_region)
        if conf_goal < 0.20:
            raise RuntimeError("Low confidence (%.2f) for goal region '%s'" % (conf_goal, goal.goal_region))

        goal_pt = self._mask_centroid_3d(mask_goal, depth)
        if goal_pt is None:
            raise RuntimeError("Could not compute 3D centroid for goal region")

        self._send_feedback("Computing push trajectory")
        push_vec = goal_pt - obj_pt
        push_dist = np.linalg.norm(push_vec)
        if push_dist < 0.01:
            raise RuntimeError("Object and goal region too close (%.3fm)" % push_dist)
        push_dir = push_vec / push_dist

        approach_pt = obj_pt - push_dir * PUSH_APPROACH_OFFSET
        approach_pt[2] = obj_pt[2] + PUSH_HEIGHT

        approach_pose = self._make_pose(approach_pt, push_dir)

        self._send_feedback("Moving to push approach")
        self.arm.set_pose_target(approach_pose)
        plan = self.arm.plan()
        traj = plan[1] if isinstance(plan, tuple) else plan
        self.arm.clear_pose_targets()
        if not traj or len(traj.joint_trajectory.points) == 0:
            raise RuntimeError("No IK solution for push approach")
        self.arm.execute(traj, wait=True)
        self.arm.stop()

        self._send_feedback("Executing push")
        push_end = goal_pt.copy()
        push_end[2] = obj_pt[2] + PUSH_HEIGHT
        waypoints = []
        for alpha in [0.33, 0.66, 1.0]:
            pt = approach_pt + (push_end - approach_pt) * alpha
            waypoints.append(self._make_pose(pt, push_dir).pose)

        traj, fraction = self.arm.compute_cartesian_path(waypoints, 0.01, 0.0)
        if fraction < 0.5:
            raise RuntimeError("Cartesian push path only %.0f%% feasible" % (fraction * 100))
        self.arm.execute(traj, wait=True)
        self.arm.stop()

        self._send_feedback("Retreating")
        self._move_cartesian_z(RETREAT_HEIGHT)

        self.server.set_succeeded(PandaPushResult(success=True, message="push succeeded"))

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

    def _mask_centroid_3d(self, mask, depth):
        ys, xs = np.where(mask & np.isfinite(depth) & (depth > 0.1) & (depth < 2.0))
        if len(xs) == 0:
            return None
        z = float(np.median(depth[ys, xs]))
        fx = fy = 700.0
        cx = depth.shape[1] / 2.0
        cy = depth.shape[0] / 2.0
        x = (float(xs.mean()) - cx) * z / fx
        y = (float(ys.mean()) - cy) * z / fy
        return np.array([x, y, z], dtype=np.float32)

    def _make_pose(self, position, push_dir):
        from tf.transformations import quaternion_from_euler
        import math
        yaw = math.atan2(push_dir[1], push_dir[0])
        q = quaternion_from_euler(math.pi, 0.0, yaw)  # gripper pointing horizontally in push direction
        ps = PoseStamped()
        ps.header.frame_id = "panda_link0"
        ps.header.stamp = rospy.Time.now()
        ps.pose.position.x = float(position[0])
        ps.pose.position.y = float(position[1])
        ps.pose.position.z = float(position[2])
        ps.pose.orientation.x = q[0]
        ps.pose.orientation.y = q[1]
        ps.pose.orientation.z = q[2]
        ps.pose.orientation.w = q[3]
        return ps

    def _move_cartesian_z(self, dz):
        pose = self.arm.get_current_pose().pose
        pose.position.z += dz
        traj, fraction = self.arm.compute_cartesian_path([pose], 0.01, 0.0)
        if fraction > 0.8:
            self.arm.execute(traj, wait=True)
        self.arm.stop()

    def _send_feedback(self, text):
        rospy.loginfo("[push] %s", text)
        self.server.publish_feedback(PandaPushFeedback(status=text))


if __name__ == "__main__":
    rospy.init_node("panda_push_server")
    PandaPushServer()
    rospy.spin()
