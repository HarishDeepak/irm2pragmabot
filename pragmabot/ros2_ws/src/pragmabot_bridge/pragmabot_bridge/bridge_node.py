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

execute_push() is a non-prehensile skill for clearing an obstruction
before a pick: perceive the object, close the hand, and shove it "left" or
"right" (camera-relative) along the table with a slow lateral Cartesian
move. No grasp, so none of the tilt/width/confidence gates apply - it
works on objects the two-finger hand cannot pick. Added 2026-08-27 once
pick-and-place was working on real hardware.

ACTION SERVER
-------------
This node now also serves `/pragmabot/execute_skill`
(pragmabot_interfaces/action/ExecuteSkill), which is what connects the VLM
planner to the robot - the gap the released PragmaBot code leaves as
NotImplementedError. PandaSkillExecutor (planner side) sends one goal per
planner decision; _execute_skill_cb dispatches on `chosen_skill`.

The server runs in a ReentrantCallbackGroup behind a MultiThreadedExecutor
because the skill callbacks are BLOCKING: execute_pick()/execute_place()
wait on MoveIt futures internally. With the default single-threaded
executor and a mutually-exclusive callback group that is an immediate
deadlock - the callback waits for a future that only the executor it is
blocking could ever complete. The internal waits go through
_spin_until_done(), which polls future.done() and lets the OTHER executor
threads do the servicing - NOT the module-level
rclpy.spin_until_future_complete(), whose throwaway executor detaches the
node from the main one after the first goal and freezes the next
(a `place` after a `pick`).

PERCEPTION IS WIRED IN (use_live_perception, default true). Each pick
resolves the action's `target_object` live: PerceptionClient (ZMQ :5557,
GroundedSAM -> mask -> filtered cloud) then GraspGen (ZMQ :5556), via
live_perception.LivePerception. Previously the bridge read a pre-computed
`grasp_file` parameter and ignored `target_object` entirely, so "pick the
red cup" picked whatever was segmented in the last offline run - and a
success on the wrong object writes a fabricated entry into the long-term
memory this project is graded on.

PLACEMENT IS ALSO PERCEIVED. When the planner names a `placement_object`,
execute_place() runs the SAME detect -> mask -> depth pipeline against that
surface (PerceptionClient :5557), picks a point well inside its mask via
farthest-point sampling (calibration/mask_sampling.py), back-projects it
with mask_to_pointcloud's own pinhole code and transforms it into
fr3_link0. GraspGen is deliberately NOT called for this: a table or plate
is not being grasped, and a grasp pose on it would mean nothing. The fixed
place_offset_xyz survives only as the fallback.

Set use_live_perception:=false to fall back to the `grasp_file` /
`object_pcd_file` parameters for offline replay against a saved npz.

REQUIRES `self._scene_source` to be set to a callable returning
(rgb, depth, intrinsics). This node deliberately does not subscribe to the
camera itself - the planner's SceneObserver already reads those
BEST_EFFORT topics, and a second subscriber here would compete with it.
Until it is injected, a pick fails with a reason rather than guessing.
"""

import tempfile
import time
from pathlib import Path

import numpy as np
import rclpy
from franka_msgs.action import Grasp, Homing, Move
from franka_msgs.msg import GraspEpsilon
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    CollisionObject,
    Constraints,
    MoveItErrorCodes,
    OrientationConstraint,
    PlanningScene,
    PositionConstraint,
)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetPositionIK
from pragmabot_interfaces.action import ExecuteSkill
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener

from pragmabot_bridge import grasp_transform
from pragmabot_bridge.cartesian_path import cartesian_step_schedule
from pragmabot_bridge.fr3_limits import FR3Limits


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
        # Used only to SCORE grasp candidates before committing to motion
        # (see the joint-motion/limit prefilter in execute_pick) - never
        # for execution itself, which stays on _compute_cartesian_path and
        # _move_to_pose with their own collision/jump guards.
        self._ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self._latest_joint_state: JointState | None = None
        self._joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 10
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._scene_client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self._table_in_scene = False

        # Reentrant group + MultiThreadedExecutor (see module docstring):
        # the skill callbacks block on spin_until_future_complete, which
        # deadlocks under the default single-threaded/mutually-exclusive
        # setup.
        self._skill_cb_group = ReentrantCallbackGroup()
        self._skill_server = ActionServer(
            self,
            ExecuteSkill,
            "/pragmabot/execute_skill",
            execute_callback=self._execute_skill_cb,
            callback_group=self._skill_cb_group,
        )

        # Where the last successful pick left the object, in fr3_link0.
        # execute_place() needs a pose to descend to, and the natural one
        # is "the pose we picked from, moved to the new location". Set by
        # execute_pick(), consumed by execute_place().
        self._last_grasp_T_base = None
        # The held object's points in the fr3_hand frame at the moment of the
        # grasp (visible surface + its footprint dropped to the table), so a
        # push with it as the tool knows where its lowest point and leading
        # face are for any hand orientation. Set/cleared with the grasp pose.
        self._held_cloud_hand = None

        # How many obstacle objects have been placed into the clearing zone
        # so far this session. Used to fan successive obstacles out across
        # the zone (clearing_zone_step_xyz per object) so a second object
        # is not dropped on top of the first. Reset with
        # reset_clearing_zone().
        self._cleared_count = 0

        # Callable returning (rgb, depth, intrinsics) for live perception.
        #
        # The original objection to setting this here was that a permanent
        # camera subscription on the bridge would compete with the
        # planner's SceneObserver on the same BEST_EFFORT sensor topics.
        # capture_scene.capture() does not create one: it builds a
        # throwaway node, takes ONE frame, and destroys the subscription
        # again. Nothing is subscribed between picks, so there is nothing
        # to compete with - and injection still overrides it, so replay and
        # tests can substitute a canned frame.
        self._scene_source = self._capture_scene

        # Read once: franka_description's joint_limits.yaml if present,
        # else the fallback table baked into fr3_limits.py.
        self._limits = FR3Limits.load()

        # The most recent failure reason from a skill. The action result used
        # to say only "see the node log", so the planner never learned WHY a
        # pick failed and invented physical explanations for what were
        # actually crashed ROS nodes - it reasoned about gripper span and cup
        # geometry while the real cause was `Action servers: 0`. Self-
        # reflection on a wrong cause is worse than none.
        self._last_failure = ""

        # grasp_file is now a FALLBACK, used only when use_live_perception
        # is false. Live perception resolves the planner's target_object
        # per pick; the parameter is kept so offline replay against a saved
        # npz still works.
        self.declare_parameter("grasp_file", "")
        self.declare_parameter("use_live_perception", True)
        self.declare_parameter("perception_host", "127.0.0.1")
        self.declare_parameter("perception_port", 5557)
        self.declare_parameter("graspgen_port", 5556)
        self.declare_parameter("group_name", "fr3_arm")
        self.declare_parameter("eef_link", "fr3_hand")
        self.declare_parameter("standoff_m", 0.12)
        self.declare_parameter("lift_m", 0.12)
        self.declare_parameter("gripper_width", 0.0)
        self.declare_parameter("gripper_speed", 0.05)
        self.declare_parameter("gripper_force", 20.0)
        self.declare_parameter("gripper_epsilon", 0.02)
        # Asymmetric on purpose - see _grasp(). inner small so an empty
        # close can never report success; outer generous because the width
        # estimate errs low on single-view clouds.
        self.declare_parameter("gripper_epsilon_inner", 0.008)
        self.declare_parameter("gripper_epsilon_outer", 0.045)
        # A compressible object (sponge, towel) squeezes far below its
        # estimated width, which epsilon_inner alone reports as an empty
        # close. If > 0, any stop with the fingers still at least this far
        # apart counts as holding something; a true empty close ends near 0.
        # <= 0 restores the plain epsilon_inner band.
        self.declare_parameter("gripper_min_held_width", 0.004)
        self.declare_parameter("camera_frame", "zed_left_camera_frame_optical")
        self.declare_parameter("color_topic", "/zed/zed_node/rgb/color/rect/image")
        self.declare_parameter("depth_topic", "/zed/zed_node/depth/depth_registered")
        self.declare_parameter("camera_info_topic",
                               "/zed/zed_node/rgb/color/rect/image/camera_info")
        self.declare_parameter("capture_timeout_s", 15.0)
        self.declare_parameter("object_pcd_file", "")
        self.declare_parameter("grasp_index", -1)
        self.declare_parameter("revolute_jump_threshold", 0.2)

        # --- straight-line (Cartesian) path quality ---------------------
        # eef_step: the Cartesian distance between successive IK samples
        # along the straight line. Smaller means the line is followed more
        # tightly and each step asks less of the IK solver, at the cost of
        # more samples. 1cm is the starting value; a failed plan is retried
        # with progressively finer steps down to cartesian_min_step.
        self.declare_parameter("cartesian_max_step", 0.01)
        self.declare_parameter("cartesian_min_step", 0.001)
        self.declare_parameter("cartesian_max_tries", 5)

        # --- FR3 position-dependent velocity limit -----------------------
        # Every trajectory is checked against the real limit before it is
        # sent, and slowed down if it would violate it. See fr3_limits.py
        # for why MoveIt cannot do this itself.
        #
        # velocity_safety_margin: fraction of the permitted velocity we
        # allow. libfranka rejects AT the limit, not near it, so leave
        # headroom for the controller's own tracking error.
        self.declare_parameter("velocity_safety_margin", 0.9)
        self.declare_parameter("enforce_velocity_limits", True)
        # A near-singular Cartesian segment shows up as a big joint step
        # between waypoints only 1cm apart. GetCartesianPath accepts
        # revolute_jump_threshold but does not forward it to the
        # interpolator (moveit2 #2404), so the guard has to be applied
        # here instead of trusted to the service.
        self.declare_parameter("max_joint_jump_rad", 0.5)

        # --- joint-motion / limit-proximity candidate prefilter -----------
        # WHY THIS EXISTS. kinematics.yaml (franka_fr3_moveit_config) uses
        # lma_kinematics_plugin - a single-solution numeric IK solver with
        # NO secondary objective. The FR3 is 7-DoF against a 6-DoF pose
        # goal, so a whole null space of joint configs (elbow/wrist swing)
        # reaches the identical gripper pose, and LMA has nothing telling
        # it to prefer the one closest to where the arm already is or
        # furthest from a limit - it just returns whatever the
        # Levenberg-Marquardt descent converges to from the seed, which can
        # be an unnecessary near-full rotation of joint 7 (or any other
        # joint) even when a much smaller move reaches the same pose. This
        # runs /compute_ik once per grasp candidate's standoff pose BEFORE
        # any motion is attempted, purely to reject candidates whose
        # standoff config is a large joint-space jump from the live state
        # or crowds a limit - grasp_transform.config_distance/
        # joint_limit_margin already existed for exactly this but were
        # never wired in until now. Real fix for the underlying redundancy
        # is a null-space-aware IK plugin (pick_ik); this is a same-solver
        # mitigation that needs no MoveIt reconfigure.
        # 0.0 disables the corresponding check.
        self.declare_parameter("max_candidate_config_distance_rad", 2.5)
        self.declare_parameter("min_candidate_limit_margin_rad", 0.05)

        # --- action call timeouts ----------------------------------------
        # Bounds every wait in _send_goal_blocking. Without these a crashed
        # action server (whose name lingers in the DDS graph) hangs the
        # bridge silently instead of failing with a reason.
        # result timeout covers the slowest legitimate call: a full MoveIt
        # plan + execute, or a gripper homing cycle.
        self.declare_parameter("action_server_timeout_s", 10.0)
        # Grasps below this GraspGen confidence are skipped when a better one
        # exists, as a soft preference during ranking (grasp_transform.
        # rank_grasp_indices - ignored there if it would leave no tilt/width
        # survivor at all). FAILS CLOSED at the end, though (see the
        # explicit check after grasp_index is finalised below): if the
        # candidate that actually survives the tilt+width gates still can't
        # reach this bar, execute_pick aborts rather than running it anyway.
        # Measured 2026-08-26: a real pick executed a tilt/width-legal but
        # 0.634-confidence grasp (best available overall was 0.968, but that
        # one was outside the tilt gate) and failed to close on the cube -
        # this is the exact "confident enough to be worth the risk" case the
        # soft ranking alone does not catch, since NO candidate that frame
        # cleared 0.80 and the ranking just returned the best of a weak set.
        # 2026-08-27: lowered 0.80 -> 0.70. On the wooden cubes in the
        # obstacle-clearing scene the only grasps that clear the tilt+width
        # gates score ~0.74-0.76 (the 0.90+ candidates are all too tilted),
        # so 0.80 aborted every pick. 0.70 still rejects the genuinely-weak
        # ~0.63 case that motivated the hard gate. Raise back toward 0.80
        # if picks start closing on nothing again.
        self.declare_parameter("min_grasp_confidence", 0.70)
        # Grasps whose approach axis is more than this many degrees off
        # straight-down are excluded entirely, before confidence is even
        # considered - a tilted approach risks the fingers clipping the
        # table. UNLIKE min_grasp_confidence, this gate fails CLOSED: if
        # nothing qualifies, execute_pick aborts (see the grasp_index < 0
        # check below) rather than falling back to the least-tilted
        # candidate anyway. 20 deg is a starting point (a two-finger
        # top-down grasp on a small tabletop object), not empirically
        # tuned; 0 disables the gate. Briefly defaulted to 0 on
        # 2026-08-24 to isolate a motion-planning issue from grasp
        # quality - reverted after confirming with live data that every
        # "best confidence" failure since was this gate's exact target
        # (one measured case: the top-confidence grasp's target was
        # 7cm BELOW the robot's own base origin - physically under the
        # table, not a false alarm).
        # 2026-08-27: widened 20 -> 30. GraspGen's high-confidence (0.90+)
        # grasps for the wooden cubes sit at ~22-28 deg off vertical; at
        # 20 deg they were all excluded and only weak near-vertical ones
        # remained. 30 deg still keeps the approach clearly top-down.
        self.declare_parameter("max_grasp_tilt_deg", 12.0)
        # Keep only the top-K GraspGen candidates by confidence before the
        # tilt gate/selection ever sees them. 0 = no cap (~100 candidates,
        # as GraspGen returns by default) - DEFAULT, after 2026-08-24
        # testing: a top-6 cap caused a real pick to abort ("no candidate
        # within 20 deg") on the SAME cube that succeeded moments earlier
        # with the full ~100 - the one near-vertical candidate (tilt 2.7
        # deg) ranked #9 by confidence, outside the top-6 cutoff. Confirms
        # the tradeoff isn't worth it: fewer candidates only shrinks the
        # tilt gate's chances of finding a valid one, no upside. Client-
        # side cap, see LivePerception.grasp_topk, if a future reason to
        # use a nonzero value comes up.
        self.declare_parameter("grasp_topk", 0)
        # Number of grasps GraspGen samples per request. Raised from its
        # 200/top-100 default because that confidence cut was discarding
        # every near-vertical candidate - see LivePerception.num_grasps.
        self.declare_parameter("graspgen_num_grasps", 1200)
        # Minimum finger span for a grasp to be considered real. A
        # single-view cube face is a ~1 cm-thick shell; a grasp closing
        # across only that is pinching air next to the object. 0 disables.
        self.declare_parameter("min_gripper_width", 0.015)
        # Widest finger span the hand can actually close around, with
        # margin. The Franka Hand opens to 0.080 m, but a grasp measured at
        # the very limit leaves no room for the approach: on a real cup
        # (83 mm across) a 66 mm candidate was selected, the fingers caught
        # the outside on the way down and knocked it over instead of
        # closing on it. Capping at 0.070 m keeps >=5 mm clearance per
        # finger and pushes selection toward the narrow parts of a wide
        # object - a cup's rim wall or handle - which the hand CAN close
        # on, instead of refusing the object outright.
        self.declare_parameter("max_gripper_width", 0.070)
        # Widest the OBJECT may be across the finger axis for the hand to
        # get around it at all. The Franka Hand opens to 0.080 m; 0.075
        # leaves a little for approach error. Distinct from
        # max_gripper_width, which is about where the fingers CLOSE - this
        # is about whether they can get around the object in the first
        # place, and it is the one that rules out an 88 mm cup.
        self.declare_parameter("max_object_span", 0.075)
        # Longest side (any axis) the perceived object cloud may have before
        # execute_pick treats it as a contaminated detection - mask bled
        # onto the table, or bad stereo depth on the object - and aborts
        # asking for a re-capture instead of letting the width gate call it
        # "too large for the gripper". 0.20 m is well above any object this
        # hand grasps and well below the tens-of-cm blob a bad mask makes.
        self.declare_parameter("max_object_extent_m", 0.20)
        # CROSS-AXIS GRASP PREFERENCE (for elongated objects: banana,
        # carrot, pen). GraspGen's highest-confidence candidate on a long
        # thin object is often an end-to-end pinch the hand physically
        # cannot make - or a diagonal one that clips. When the perceived
        # cloud's horizontal footprint is at least `crossaxis_min_elongation`
        # times longer one way than the other, the ranker multiplies each
        # candidate's confidence by up to (1 - crossaxis_grasp_weight) for
        # closing ALONG the length, pulling crosswise grasps to the top.
        # Soft re-rank, never a filter. Weight 0.0 disables it entirely;
        # the cube path is untouched because a cube's footprint is not
        # elongated enough to trip the threshold.
        self.declare_parameter("crossaxis_grasp_weight", 0.6)
        self.declare_parameter("crossaxis_min_elongation", 1.6)
        # How many grasp candidates to try before giving up on the pick.
        # A pose can be a fine grasp yet have no IK solution for this arm;
        # the next candidate usually does.
        self.declare_parameter("max_grasp_attempts", 5)
        # Slide each grasp onto the object's true centre along the
        # finger-closing axis (single-view centroid bias). Set false to
        # execute GraspGen's raw poses.
        self.declare_parameter("center_grasp_on_object", True)
        # Fingertip height above the table, as a fraction of the object's
        # height. 0.35 puts the fingers well into the object's lower half
        # while keeping the tips clear of the tabletop.
        self.declare_parameter("grip_height_fraction", 0.35)
        # Hard cap on how far BELOW the perceived object top the fingertips
        # may be driven, regardless of the table-anchored target above.
        # The table anchor assumes the whole cloud is one object; when
        # segmentation merges a stacked pair into one tall column (two
        # touching cubes -> a 10 cm "object"), the table anchor aims into
        # the LOWER item and the gripper drives through the top one. This
        # bound keeps the target within one item-height of the top, so a
        # merged/tall cloud is still grasped near its top. 45 mm ~= one
        # cube. Also lifts genuine tall-object grasps (bottle, pepper) off
        # their base, which is the safer place to hold them anyway.
        self.declare_parameter("max_grip_depth_m", 0.045)
        # --- Empirical calibration correction (see execute_pick, applied to
        # T_base_from_cam right after the TF lookup) - measured 2026-08-26
        # via a hand-guided touch-test against fr3_zed_right.calib: bias was
        # x=-3mm, y=-30mm, z=+6mm, dominated by Y. Zero disables (raw
        # calibration, uncorrected). Re-measure and update these if picks
        # keep missing in a consistent direction after this correction.
        self.declare_parameter("calib_correction_x", -0.003)
        self.declare_parameter("calib_correction_y", -0.030)
        # z bumped 0.006 -> 0.008 (2026-08-28): grasps were landing ~2 mm
        # low across all objects by visual inspection. Still one-sample
        # eyeballed, not a touch-test number.
        self.declare_parameter("calib_correction_z", 0.008)
        # --- Table collision object -------------------------------------
        # MoveIt starts with an EMPTY world: no table, no obstacles. Every
        # plan to the standoff pose was therefore free to route the arm
        # straight through the tabletop, which is what had to be stopped by
        # hand on run after run, for every object. Pushing one box in fixes
        # it for every future plan. `table_z` is the tabletop height in
        # fr3_link0 - measured 0.010 m from the object clouds (a cube's
        # lowest points sit on it), set slightly below that so the box
        # never swallows the object we are trying to grasp.
        self.declare_parameter("add_table_collision", True)
        self.declare_parameter("table_z", 0.005)
        # Box spans only the workspace IN FRONT of the robot: a slab
        # covering the origin would intersect fr3_link0/link1 and every
        # plan would fail on "start state in collision" instead.
        self.declare_parameter("table_center_x", 0.60)
        self.declare_parameter("table_size_x", 0.70)
        self.declare_parameter("table_size_y", 1.00)
        self.declare_parameter("table_thickness", 0.10)
        self.declare_parameter("action_result_timeout_s", 120.0)
        self.declare_parameter("place_after_s", 0.0)
        self.declare_parameter("gripper_open_width", 0.08)
        self.declare_parameter("home_gripper_first", True)

        # FALLBACK drop location: an [x, y, z] offset in fr3_link0 applied
        # to the pose the object was picked from. A pure offset (rather than
        # an absolute pose) keeps the approach orientation that already
        # worked for this object, and only moves where it lands. Default:
        # 20cm to the robot's left, same height.
        #
        # This is no longer the only path. When the planner names a
        # `placement_object`, execute_place() perceives that surface and
        # places on it (see _resolve_placement). The offset below is used
        # only when no placement object was named, or when perceiving it
        # failed - and in that case the returned message says a perceived
        # pose was attempted and why it was not used, because the planner
        # reflects on that text.
        self.declare_parameter("place_offset_xyz", [0.0, 0.20, 0.0])

        # --- clear-obstacle-by-picking --------------------------------
        # Preferred way to get a graspable obstacle out of the way of a
        # target object: PICK it and PLACE it aside, instead of PUSH (a
        # closed-gripper shove that can clip the obstacle instead of
        # moving it - the failure this path was added for). Works for any
        # graspable object regardless of colour or size: the object is
        # named by the planner's free-text target_object (fed to the
        # detector as a prompt) and its height is measured per object by
        # _hold_height, not assumed.
        #
        # When execute_place() runs with NO placement_object (the planner
        # asks to "put it aside" / "move it out of the way"):
        #   - if `clearing_zone_xyz` is set to "x,y,z" (metres, in
        #     fr3_link0), the object is released at that ABSOLUTE point (a
        #     fixed spot you have chosen to be clear of the working area
        #     and reachable on YOUR table). z is the bare surface height;
        #     _hold_height + place_clearance_m are added on top at release.
        #   - if it is left "" (the default), the object is released at
        #     `place_offset_xyz` RELATIVE to where it was picked (no table
        #     geometry assumed).
        # No default location is baked in - set this for your table, e.g.
        #   --ros-args -p clearing_zone_xyz:="0.40,0.30,0.01"
        self.declare_parameter("clearing_zone_xyz", "")
        # Per-obstacle fan-out: the Nth obstacle placed aside this session
        # lands at its base point + N * this, so successive obstacles do
        # not stack. Only applied when clearing_zone_xyz is set. Size the
        # step to your largest obstacle and to which axis has room on your
        # table. Default [0,0,0] = no fan-out (every obstacle to the same
        # point - fine for a single obstacle).
        self.declare_parameter("clearing_zone_step_xyz", [0.0, 0.0, 0.0])

        # --- perceived placement ---------------------------------------
        # Gap left between the held object's lowest point and the perceived
        # surface at release. The object is dropped this far, not lowered
        # onto contact: the arm has no force feedback in this path, so
        # descending until it touches would press the object into the table.
        # 4 cm, not 1: the perceived tray-floor depth carries a cm or two of
        # stereo error, and a raised tray rim the cube can catch on is not
        # modelled - so the object is released into free air a safe gap
        # above the surface and drops the rest of the way. A light cube
        # tolerates that fine; raise it further for a fragile object.
        self.declare_parameter("place_clearance_m", 0.04)
        # The final straight-line descent onto the surface is retimed to
        # this fraction of its planned speed (0.25 = 4x slower). Cartesian
        # paths come back timed at full speed with no scaling field in
        # Humble (see grasp_transform.retime_trajectory); a slow descent
        # means any unmodelled contact trips the FR3 reflex gently instead
        # of at speed.
        self.declare_parameter("place_descent_speed_factor", 0.25)
        # How many farthest-point candidates to consider on the placement
        # mask, and how far from the mask's edge the winner must be. See
        # calibration/mask_sampling.py for why plain FPS alone picks corners.
        self.declare_parameter("placement_fps_candidates", 16)
        self.declare_parameter("placement_interior_px", 8)
        # Side of the pixel patch whose median depth becomes the placement
        # point. One pixel is one stereo match; a single bad match would
        # move the release point by tens of centimetres.
        self.declare_parameter("placement_patch_px", 5)
        # Mask-area ceiling for a PLACEMENT surface. The pick path keeps the
        # object-sized 40% bound; a table measured 51% of this rig's frame,
        # so reusing the object bound would refuse every real surface.
        self.declare_parameter("placement_max_mask_frac", 0.85)

        # --- push ------------------------------------------------------
        # How far to shove the object, measured from its centroid along the
        # push direction (in the ground plane).
        self.declare_parameter("push_distance_m", 0.12)
        # Where on the object's side the closed fingers contact it, as a
        # fraction of the object's height above the table. Low = stable
        # (less tipping); clamped to a sane absolute band.
        self.declare_parameter("push_height_frac", 0.35)
        self.declare_parameter("push_height_min_m", 0.015)
        self.declare_parameter("push_height_max_m", 0.06)
        # Gap left between the object's edge and the fingers at the start of
        # the push, so the descent lands beside the object, not on it.
        self.declare_parameter("push_contact_margin_m", 0.03)
        # Descent and lateral-push trajectories retimed to this fraction of
        # planned speed (same rationale as place_descent_speed_factor).
        self.declare_parameter("push_speed_factor", 0.25)
        # fr3_hand origin -> fingertip plane along the (downward) approach
        # axis. Same constant grasp_transform uses (gripper_depth).
        self.declare_parameter("push_fingertip_offset_m", 0.10527314)
        # Mask-area ceiling for a PUSH target. Looser than the pick path's
        # 40% (a push target is often bigger and closer than a pick one),
        # tighter than a placement SURFACE's 85%.
        self.declare_parameter("push_max_mask_frac", 0.60)
        # Finger gap while pushing. Not fully closed (0.0) - commanding the
        # fingers into each other can stall the Move and fault the hand.
        self.declare_parameter("push_gripper_width", 0.005)
        # Pushing WITH a held object (e.g. a sponge): height of that object's
        # lowest point above table_z during the shove. Small, so it reaches
        # flat objects the fingertips (push_height_min_m) sail over; not
        # zero, so calibration error does not grind the tool into the table.
        self.declare_parameter("tool_push_clearance_m", 0.004)

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
        # Make sure the world has a table in it before ANY motion is
        # planned. Cheap, idempotent, and the first plan is exactly the
        # one that used to dive at the tabletop.
        self._add_table_collision()

        if home_gripper_first and not self._home_gripper():
            self._fail_log(
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
            self._fail_log(f"TF lookup fr3_link0 <- {camera_frame} failed: {exc}")
            return False

        # EMPIRICAL CALIBRATION CORRECTION (measured 2026-08-26). Repeated
        # real picks missed to one side / clipped the object's top edge
        # even though the arm always reached its OWN computed target within
        # ~2mm (see ARRIVAL CHECK) - i.e. self-consistent but wrong. A
        # touch-test compared the computed grasp's fingertip contact point
        # (projected depth mm along the grasp's own approach axis, since
        # GraspGen's origin is the gripper base, not the fingertip) against
        # a hand-guided pose centred by eye on the same real cube: the bias
        # was x=-3mm, y=-30mm, z=+6mm - dominated by a single ~30mm Y
        # offset, on an object only ~40-45mm wide. That's the exact size of
        # "lands on the object's edge instead of straddling its centre"
        # failures we were seeing. Applied here as a fixed post-calibration
        # translation correction (in fr3_link0), not by editing
        # ~/.ros2/easy_handeye2/calibrations/fr3_zed_right.calib directly,
        # so the raw calibration output stays untouched and this stays
        # visible/tunable/revertible from one place. ONE noisy sample (a
        # human eyeballing "centred") - treat as a starting point, not
        # gospel; re-measure and adjust if picks still miss consistently in
        # some direction after this.
        calib_correction = np.array([
            float(self.get_parameter("calib_correction_x").value),
            float(self.get_parameter("calib_correction_y").value),
            float(self.get_parameter("calib_correction_z").value),
        ])
        T_base_from_cam[:3, 3] += calib_correction
        # Correct the single-view centroid bias BEFORE ranking, so tilt,
        # width and reachability are all judged on the pose that will
        # actually be executed. See center_grasp_on_object().
        if object_pcd_file and bool(self.get_parameter("center_grasp_on_object").value):
            try:
                _pcd = np.load(object_pcd_file).astype(np.float64)[:, :3]
                _shifted = np.array([
                    grasp_transform.center_grasp_on_object(g, _pcd)
                    for g in grasps_T_cam])
                # Report the CLOUD's own extent, not a median over all
                # candidates: most of the 1200 are side/edge grasps whose
                # shift is legitimately large, so that median said nothing
                # about the one that gets executed. The per-grasp shift for
                # the winner is logged after selection instead.
                _ext = (_pcd.max(axis=0) - _pcd.min(axis=0)) * 100
                self.get_logger().info(
                    "Centred grasps on the object's finger-axis midpoint "
                    f"(object extent {_ext[0]:.1f} x {_ext[1]:.1f} x "
                    f"{_ext[2]:.1f} cm) - corrects the single-view centroid "
                    "bias toward the camera"
                )
                grasps_T_cam = _shifted

                # CONTAMINATED-CLOUD GATE. A segmented tabletop object that
                # is genuinely graspable is at most ~15-20 cm on its
                # longest side. When the mask bleeds onto the table or the
                # stereo depth on the object is poor, the back-projected
                # cloud balloons to tens of cm across and only a cm or two
                # "tall" - and every downstream gate then reports the
                # object as "too wide for the gripper", which reads to the
                # planner as a real property of the object and makes it
                # escalate to PUSH forever. Catch it here and say what is
                # actually wrong: the PERCEPTION needs redoing, not the
                # skill. (Fires before the width gate so the message the
                # planner reflects on is the right one.)
                _longest_cm = float(np.sort(_ext)[-1])
                _max_cm = float(self.get_parameter("max_object_extent_m").value) * 100
                if _longest_cm > _max_cm:
                    self._fail_log(
                        f"perceived object cloud spans {_ext[0]:.0f} x "
                        f"{_ext[1]:.0f} x {_ext[2]:.0f} cm - larger than any "
                        f"object this gripper handles ({_max_cm:.0f} cm max). "
                        "The segmentation mask or the depth on this object is "
                        "bad (mask bleeding onto the table, or poor stereo "
                        "depth on the surface). RE-CAPTURE the scene / try a "
                        "cleaner view of this same object - do NOT switch to "
                        "PUSH, the object is not actually oversized."
                    )
                    return False
            except Exception as exc:  # noqa: BLE001 - never block a pick on this
                self.get_logger().warn(f"Grasp centring skipped: {exc}")

        grasps_T_base = T_base_from_cam @ grasps_T_cam

        # ANCHOR GRASP DEPTH TO THE TABLE, NOT TO THE CLOUD.
        # The camera-frame correction above sets depth from the object
        # cloud's own extent along the approach axis - and that extent is
        # NOT stable between captures. Three consecutive captures of the
        # same stationary cube put its top face at z=0.0560, 0.0470 and
        # gave visible heights of 42.2, 41.2 and 51.9 mm. A 9 mm swing in
        # the perceived top swings the grasp depth by the same 9 mm, which
        # is the difference between a 20 mm grip and clipping the top
        # edge - exactly the "works, then hits the top" intermittency.
        #
        # The tabletop does not move and its height is already known
        # (table_z, the same value the collision box uses). Placing the
        # fingertips a fixed fraction of the object's height ABOVE THE
        # TABLE is therefore stable across captures in a way that
        # anchoring to the noisy upper surface can never be.
        if object_pcd_file and bool(self.get_parameter("center_grasp_on_object").value):
            try:
                _p = np.load(object_pcd_file).astype(np.float64)[:, :3]
                _pb = (T_base_from_cam[:3, :3] @ _p.T).T + T_base_from_cam[:3, 3]
                _table = float(self.get_parameter("table_z").value)
                # 98th percentile, not max: the top face carries stereo
                # noise and a single flyer would raise the whole target.
                _top = float(np.percentile(_pb[:, 2], 98))
                _height = max(_top - _table, 0.0)
                _frac = float(self.get_parameter("grip_height_fraction").value)
                _tip_target = _table + float(np.clip(_frac * _height, 0.008, 0.022))

                # Never drive the fingertips more than one item-height below
                # the perceived top. Guards the merged-stack case where the
                # table anchor would aim into the cube underneath.
                _max_depth = float(self.get_parameter("max_grip_depth_m").value)
                _top_bounded = _top - _max_depth
                _capped = _top_bounded > _tip_target
                if _capped:
                    _tip_target = _top_bounded

                _depth = 0.10527314
                _fixed = grasps_T_base.copy()
                for _i in range(len(_fixed)):
                    _a = _fixed[_i, :3, 2]
                    if abs(_a[2]) < 0.3:      # near-horizontal: no sane tip height
                        continue
                    _tip_z = _fixed[_i, 2, 3] + _depth * _a[2]
                    _s = float(np.clip((_tip_target - _tip_z) / _a[2], -0.05, 0.05))
                    _fixed[_i, :3, 3] = _fixed[_i, :3, 3] + _a * _s
                grasps_T_base = _fixed
                self.get_logger().info(
                    f"Grasp depth anchored to the {'object top (merged/tall cloud)' if _capped else 'table'}: "
                    f"object top z={_top:.4f}, table z={_table:.4f}, "
                    f"fingertips targeted at z={_tip_target:.4f} "
                    f"({(_top - _tip_target) * 1000:.0f} mm of grip)"
                )
            except Exception as exc:  # noqa: BLE001 - never block a pick
                self.get_logger().warn(f"Table-anchored depth skipped: {exc}")

        if grasp_index < 0:
            # GraspGen's own confidences travel in the same npz. Selecting on
            # geometry alone let a 0.66-confidence grasp win over a 0.95 one
            # purely for being a few degrees more vertical.
            confidences = None
            try:
                with np.load(grasp_file) as _npz:
                    if "confidences" in _npz:
                        confidences = _npz["confidences"]
            except Exception as exc:  # noqa: BLE001 - fall back to geometry only
                self.get_logger().warn(
                    f"Could not read confidences from {grasp_file}: {exc} - "
                    "selecting on top-down alignment alone"
                )
            max_tilt_deg = float(self.get_parameter("max_grasp_tilt_deg").value)
            max_attempts = int(self.get_parameter("max_grasp_attempts").value)
            viable: list[int] = []

            # Cross-axis preference for elongated objects. Compute the
            # cloud's horizontal long axis in BASE frame (rank_grasp_indices
            # works in base frame); only engage it when the footprint is
            # actually elongated, so cubes/round objects are unaffected.
            _long_axis = None
            _xw = float(self.get_parameter("crossaxis_grasp_weight").value)
            if _xw > 0.0 and object_pcd_file:
                try:
                    _pc = np.load(object_pcd_file).astype(np.float64)[:, :3]
                    _pb = (T_base_from_cam[:3, :3] @ _pc.T).T + T_base_from_cam[:3, 3]
                    _axis, _elong = grasp_transform.principal_axis_xy(_pb)
                    if _elong >= float(self.get_parameter("crossaxis_min_elongation").value):
                        _long_axis = _axis
                        self.get_logger().info(
                            f"Elongated object (footprint {_elong:.1f}:1) - "
                            "preferring grasps that close across its long axis"
                        )
                except Exception as exc:  # noqa: BLE001 - never block a pick
                    self.get_logger().warn(f"Cross-axis grasp preference skipped: {exc}")

            ranked = grasp_transform.rank_grasp_indices(
                grasps_T_base, confidences,
                min_confidence=float(self.get_parameter("min_grasp_confidence").value),
                max_tilt_deg=max_tilt_deg,
                object_long_axis=_long_axis,
                crossaxis_weight=_xw,
            )
            grasp_index = int(ranked[0]) if len(ranked) else -1

            # WIDTH GATE (measured 2026-08-26). Passing the tilt gate only
            # means the approach points down; it says nothing about whether
            # the fingers span the object. A single camera view of a cube
            # is one flat face, which looks like a thin plate, and GraspGen
            # will happily propose pinching it edge-on: on a real 6 cm cube
            # the top-ranked candidate measured 0.0058 m across the fingers.
            # Commanding that closes the gripper almost fully and shoves the
            # object aside. Walk down the ranked list and take the first
            # candidate that also spans a plausible width.
            min_width = float(self.get_parameter("min_gripper_width").value)
            max_width = float(self.get_parameter("max_gripper_width").value)
            max_span = float(self.get_parameter("max_object_span").value)
            if len(ranked) and gripper_width <= 0.0 and object_pcd_file and min_width > 0.0:
                pcd_cam_probe = np.load(object_pcd_file).astype(np.float64)[:, :3]
                for rank, cand in enumerate(ranked):
                    try:
                        w = grasp_transform.estimate_gripper_width(
                            pcd_cam_probe, grasps_T_cam[cand])
                    except ValueError:
                        continue  # too few points in the contact band
                    # FEASIBILITY, not just contact width. `w` is measured
                    # in the fingertip band and under-reads badly on a
                    # single-view cloud: on a real cup it returned 39.8 mm
                    # while the cup actually spanned 87.6 mm across the
                    # finger axis - wider than the hand opens (80 mm), so
                    # the fingers stopped on the outside and shoved it.
                    # The object's FULL span along the finger axis is what
                    # decides whether the hand can go around it at all.
                    _lx = (pcd_cam_probe - grasps_T_cam[cand][:3, 3]) @ grasps_T_cam[cand][:3, 0]
                    span = float(_lx.max() - _lx.min())
                    if span > max_span:
                        continue
                    if min_width <= w <= max_width:
                        viable.append(int(cand))
                        if len(viable) == 1 and rank:
                            self.get_logger().info(
                                f"Skipped {rank} higher-ranked candidate(s): "
                                f"finger span outside [{min_width:.3f}, "
                                f"{max_width:.3f}] m - too thin to be the "
                                "object, or too wide for the hand to close"
                            )
                        if len(viable) >= max_attempts:
                            break
                if viable:
                    grasp_index = viable[0]
                else:
                    self._fail_log(
                        f"No usable grasp: every candidate either closes on "
                        f"less than {min_width * 1000:.0f} mm (the visible "
                        "shell edge-on, not the object), or the object spans "
                        f"more than {max_span * 1000:.0f} mm across the "
                        "fingers - the hand opens to 80 mm and cannot get "
                        "around it. THIS OBJECT IS TOO LARGE FOR THIS "
                        "GRIPPER; pick a smaller one rather than retrying."
                    )
                    return False

            if not viable and len(ranked):
                viable = [int(i) for i in ranked[:max_attempts]]
                grasp_index = viable[0]

            if grasp_index < 0:
                # select_grasp_index's tilt gate fails CLOSED (see its
                # docstring): -1 means every candidate was more than
                # max_tilt_deg off perpendicular, not "pick the least-bad
                # one anyway". This message goes into the planner's
                # self-reflection the same way every other execute_pick
                # failure does (_fail_log), so it needs to suggest
                # something the planner can actually act on next - here,
                # that the OBJECT's pose is the problem, not the grasp/
                # trajectory/gripper, since no near-vertical approach
                # existed for it in this orientation at all.
                self._fail_log(
                    f"No grasp candidate within {max_tilt_deg:.0f} deg of "
                    "perpendicular to the table - GraspGen did not offer a "
                    "safe top-down approach for this object in its current "
                    "pose. Try reorienting or repositioning the object "
                    "(e.g. standing it more upright, or turning it so a "
                    "flatter surface faces up) before retrying, rather "
                    "than repeating the same pick."
                )
                return False

            # HARD CONFIDENCE GATE (measured 2026-08-26). rank_grasp_indices'
            # min_confidence filter is soft - it falls back to the best
            # tilt/width survivor even if that survivor is nowhere near
            # min_grasp_confidence, because a single low-scoring frame must
            # not silently pass min_confidence just for lacking alternatives.
            # That is exactly what happened on a real cube: the executed
            # grasp was tilt/width-legal but only 0.634 confidence (best
            # available overall was 0.968, excluded by the tilt gate) and
            # the gripper failed to close on the object. Check the FINAL
            # selection here and abort rather than execute a grasp the
            # model itself rated as unreliable - same fail-closed contract
            # as the tilt gate above, just on confidence instead of angle.
            min_grasp_confidence = float(self.get_parameter("min_grasp_confidence").value)
            if confidences is not None and confidences[grasp_index] < min_grasp_confidence:
                self._fail_log(
                    f"No usable grasp reaches min_grasp_confidence "
                    f"({min_grasp_confidence:.2f}): the best candidate that "
                    f"passed the tilt/width gates only scored "
                    f"{confidences[grasp_index]:.3f} (best available overall "
                    f"was {np.max(confidences):.3f}, but it did not pass the "
                    "tilt or width gate). Aborting rather than executing a "
                    "grasp GraspGen itself rated unreliable - try "
                    "reorienting or repositioning the object, or "
                    "re-capturing the scene, before retrying."
                )
                return False

            approach_axis = grasps_T_base[grasp_index, :3, :3] @ np.array([0.0, 0.0, 1.0])
            tilt_deg = float(np.degrees(np.arccos(np.clip(-approach_axis[2], -1.0, 1.0))))
            if confidences is not None:
                self.get_logger().info(
                    f"Auto-selected grasp {grasp_index}/{len(grasps_T_cam)} "
                    f"(confidence {confidences[grasp_index]:.3f}, best available "
                    f"{np.max(confidences):.3f}, tilt {tilt_deg:.1f} deg from "
                    "vertical) by confidence within the tilt gate"
                )
            else:
                self.get_logger().info(
                    f"Auto-selected grasp index {grasp_index}/{len(grasps_T_cam)} "
                    f"as the most top-down candidate (no confidences available, "
                    f"tilt {tilt_deg:.1f} deg from vertical)"
                )
        elif grasp_index >= len(grasps_T_cam):
            self._fail_log(
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

        if object_pcd_file:
            try:
                _p = np.load(object_pcd_file).astype(np.float64)[:, :3]
                _lx = (_p - grasp_T_cam[:3, 3]) @ grasp_T_cam[:3, 0]
                _resid = 0.5 * (_lx.min() + _lx.max())
                _half = 0.5 * (_lx.max() - _lx.min())
                self.get_logger().info(
                    f"Chosen grasp is {_resid * 1000:+.1f} mm off the object's "
                    f"centre along the finger axis (object half-width "
                    f"{_half * 1000:.1f} mm)"
                )
            except Exception:  # noqa: BLE001 - diagnostics only
                pass

        self.get_logger().info(f"standoff pose (fr3_link0):\n{standoff_T_base}")
        self.get_logger().info(f"grasp pose (fr3_link0):\n{grasp_T_base}")

        # Steps 1+2: reach the pre-grasp standoff, then approach into the
        # grasp pose via a HOVER waypoint, not a single straight line from
        # standoff - both attempted per-candidate, in the SAME retry loop.
        #
        # WHY THEY'RE IN ONE LOOP (not two separate ones). Originally the
        # standoff reach (this step) retried across `attempts` but the
        # hover-then-descend approach below did not: it only ever tried the
        # ONE candidate whichever standoff attempt happened to reach. A
        # standoff being reachable says nothing about the final approach
        # also being reachable - measured 2026-09-23, a real 'green cube'
        # pick reached standoff fine on the FIRST candidate both times, then
        # the hover+grasp Cartesian path failed at fraction 0.97-0.99 (a
        # near-singular joint flip on the final descent) and the whole pick
        # aborted outright, even though ~48 other tilt/width-legal
        # candidates were sitting right there unused. Folding both stages
        # into one loop means a candidate whose STANDOFF is reachable but
        # whose FINAL APPROACH isn't gets skipped in favour of the next
        # candidate, instead of failing the whole pick.
        #
        # Standoff: try a straight-line Cartesian path first - deterministic,
        # and already collision- and jump-guarded by
        # _compute_cartesian_path/_execute_trajectory, the same machinery
        # Steps 2 and 4 use. Fall back to free-space OMPL planning
        # (_move_to_pose) only if the straight line itself isn't reachable.
        # An unconstrained OMPL plan from wherever the arm currently is has
        # no reason to prefer a short or direct path, and with no collision
        # scene in MoveIt yet, nothing else biases it toward one either -
        # this is the "weird trajectory" symptom (unnecessary joint
        # rotation, including the wrist) seen in the 2026-08-24 evening run.
        #
        # RETRY ACROSS CANDIDATES. A single grasp pose can be perfectly good
        # yet have no IK solution for this arm (OMPL reports "Unable to
        # sample any valid states for goal tree"), which is a property of
        # THAT pose, not of the object - the other ~48 candidates that
        # passed the tilt and width gates are still there. Aborting the
        # whole pick on the first unreachable one is what made a working
        # pipeline look intermittent. Only PLANNING failures fall through
        # to the next candidate; a failure while the arm is actually
        # moving still aborts, since the arm is then somewhere unknown.
        #
        # Hover waypoint (measured 2026-08-26, real cube, hit its top
        # surface twice in a row). standoff_pose() backs the standoff off
        # along the grasp's OWN approach axis (grasp_transform.
        # standoff_pose) - for a tilted grasp that axis has a horizontal
        # component, so standoff and grasp differ in x/y AND z at once. A
        # single straight line between them therefore descends and slides
        # sideways simultaneously: one real trial measured (dx=+15mm,
        # dy=-27mm, dz=-116mm) over a 120mm move (matches the reported 15
        # deg tilt: sin(15)*120mm = 31mm of sideways drift). The object's
        # own footprint was only ~40-50mm across, so for part of that
        # descent the gripper was still sliding into its final x/y position
        # while already BELOW the object's top-surface height - a geometric
        # collision with the cube's top, not a perception or calibration
        # error. This gets WORSE at higher tilt (more sideways drift per mm
        # of descent), which is exactly the direction things got worse when
        # testing steeper tilts. THE FIX: insert a waypoint directly above
        # the grasp target - same x/y as the grasp, but at the standoff's
        # (safely elevated) z - so horizontal centering happens first, at a
        # height well clear of the object, and the final descent is a pure
        # vertical drop straight down onto the already-centred position.
        # This decouples "get over the object" from "go down onto it"
        # instead of doing both at once along a diagonal.
        attempts = viable if viable else [grasp_index]

        # JOINT-MOTION / LIMIT-PROXIMITY PREFILTER. See the
        # max_candidate_config_distance_rad/min_candidate_limit_margin_rad
        # declare_parameter comments for why this exists (lma_kinematics_
        # plugin has no null-space secondary objective). Runs /compute_ik
        # for each candidate's STANDOFF pose only (cheap - `attempts` is
        # capped by max_grasp_attempts, a handful of candidates, not
        # hundreds) and drops any whose solution is a large joint-space
        # jump from the live state or crowds a limit, before the retry loop
        # below ever tries to move there. Fails OPEN: if the live joint
        # state or /compute_ik isn't available, or every candidate gets
        # filtered out, this falls back to the unfiltered list rather than
        # blocking a pick on a diagnostic check.
        max_dist = float(self.get_parameter("max_candidate_config_distance_rad").value)
        min_margin = float(self.get_parameter("min_candidate_limit_margin_rad").value)
        if (max_dist > 0.0 or min_margin > 0.0) and self._latest_joint_state is not None:
            current_by_name = dict(zip(
                self._latest_joint_state.name, self._latest_joint_state.position))
            filtered = []
            for cand in attempts:
                cand_standoff_T = grasp_transform.standoff_pose(
                    grasps_T_base[int(cand)], standoff_m)
                ik = self._ik_joint_config(
                    group_name, eef_link, grasp_transform.matrix_to_pose(cand_standoff_T))
                if ik is None:
                    filtered.append(cand)  # couldn't check - don't block on it
                    continue
                names, positions = ik
                cand_by_name = dict(zip(names, positions))
                common = [n for n in names if n in current_by_name]
                if common and max_dist > 0.0:
                    dist = grasp_transform.config_distance(
                        [cand_by_name[n] for n in common],
                        [current_by_name[n] for n in common],
                    )
                    if dist > max_dist:
                        self.get_logger().warn(
                            f"Candidate {cand}: standoff IK needs "
                            f"{np.degrees(dist):.0f} deg of joint travel "
                            f"(limit {np.degrees(max_dist):.0f} deg) - "
                            "skipping to avoid an unnecessarily large "
                            "joint move"
                        )
                        continue
                if min_margin > 0.0:
                    margin, who = grasp_transform.joint_limit_margin(
                        names, positions, self._limits.bounds())
                    if who and margin < min_margin:
                        self.get_logger().warn(
                            f"Candidate {cand}: standoff IK leaves only "
                            f"{np.degrees(margin):.1f} deg margin on {who} "
                            f"(limit {np.degrees(min_margin):.1f} deg) - "
                            "skipping"
                        )
                        continue
                filtered.append(cand)
            if filtered:
                if len(filtered) < len(attempts):
                    self.get_logger().info(
                        f"Joint-motion/limit prefilter kept {len(filtered)}/"
                        f"{len(attempts)} candidate(s)"
                    )
                attempts = filtered
            else:
                self.get_logger().warn(
                    "Joint-motion/limit prefilter rejected every candidate "
                    "- falling back to the unfiltered list rather than "
                    "aborting the pick"
                )

        reached = False
        hover_pose_msg = None
        grasp_pose_msg = None
        cart = None
        for attempt_n, cand in enumerate(attempts):
            # Recompute from `cand` on EVERY iteration, including the
            # first: the joint-motion/limit prefilter above may have
            # reordered or dropped the original grasp_index, so
            # attempts[0] is no longer guaranteed to be whatever
            # grasp_index/grasp_T_base/standoff_T_base already hold from
            # before this loop.
            grasp_index = int(cand)
            grasp_T_cam = grasps_T_cam[grasp_index]
            grasp_T_base = grasps_T_base[grasp_index]
            standoff_T_base = grasp_transform.standoff_pose(grasp_T_base, standoff_m)
            if attempt_n:
                self.get_logger().warn(
                    f"Trying candidate {attempt_n + 1}/{len(attempts)} "
                    f"(grasp {grasp_index})"
                )
            self.get_logger().info(f"standoff pose (fr3_link0):\n{standoff_T_base}")

            standoff_pose_msg = grasp_transform.matrix_to_pose(standoff_T_base)
            at_standoff = False
            standoff_cart = self._compute_cartesian_path(
                group_name, eef_link, [standoff_pose_msg])
            if standoff_cart is not None and standoff_cart.fraction >= 1.0:
                if not self._execute_trajectory(standoff_cart.solution):
                    self._fail_log("Cartesian path to standoff pose failed to execute - aborting")
                    return False
                at_standoff = True
            elif self._move_to_pose(group_name, eef_link, standoff_pose_msg):
                at_standoff = True

            if not at_standoff:
                continue  # standoff itself unreachable - try next candidate

            hover_T_base = grasp_T_base.copy()
            hover_T_base[2, 3] = standoff_T_base[2, 3]
            hover_pose_msg = grasp_transform.matrix_to_pose(hover_T_base)
            grasp_pose_msg = grasp_transform.matrix_to_pose(grasp_T_base)
            cart = self._compute_cartesian_path(
                group_name, eef_link, [hover_pose_msg, grasp_pose_msg])
            if cart is not None and cart.fraction >= 1.0:
                reached = True
                break

            frac = None if cart is None else cart.fraction
            self.get_logger().warn(
                f"Standoff for candidate {attempt_n + 1}/{len(attempts)} "
                f"(grasp {grasp_index}) was reachable but the hover-then-"
                f"descend approach was not (fraction={frac}) - the arm is "
                "safely parked at standoff; trying the next candidate "
                "rather than aborting the pick"
            )

        if not reached:
            self._fail_log(
                f"Failed to reach standoff AND complete the hover-then-"
                f"descend approach for any of {len(attempts)} grasp "
                "candidates - every near-vertical, correctly-sized grasp "
                "for this object is either outside the arm's reachable "
                "workspace or forces a near-singular final approach from "
                "its current configuration. Move the object closer to the "
                "centre of the table, or move the arm to a different "
                "starting pose, before retrying."
            )
            return False

        # The retry loop may have advanced past the candidate the width was
        # estimated for, so recompute it for whichever grasp actually won.
        if object_pcd_file and attempts and grasp_index != int(attempts[0]):
            try:
                pcd_cam = np.load(object_pcd_file).astype(np.float64)[:, :3]
                gripper_width = grasp_transform.estimate_gripper_width(pcd_cam, grasp_T_cam)
                self.get_logger().info(
                    f"Re-estimated gripper_width={gripper_width:.4f} m for the "
                    "candidate that was actually reachable"
                )
            except ValueError as exc:
                self.get_logger().warn(f"Could not re-estimate gripper width: {exc}")

        if self._execute_trajectory(cart.solution):
            # VERIFY WE ACTUALLY ARRIVED. MoveIt reporting SUCCESS is not
            # evidence that the arm is at the pose: a run measured 62 mm of
            # residual error in x after a "successful" approach, which is
            # why the gripper kept closing on air next to a correctly
            # located object. Comparing the commanded pose against live TF
            # in the same breath removes the guesswork - and the operator
            # timing problem of reading tf2_echo by hand afterwards.
            try:
                _now = self._tf_buffer.lookup_transform(
                    "fr3_link0", eef_link, rclpy.time.Time())
                _a = np.array([_now.transform.translation.x,
                               _now.transform.translation.y,
                               _now.transform.translation.z])
                _err = _a - grasp_T_base[:3, 3]
                _mag = float(np.linalg.norm(_err)) * 1000.0
                self.get_logger().info(
                    f"ARRIVAL CHECK: commanded {np.round(grasp_T_base[:3, 3], 4)}, "
                    f"actual {np.round(_a, 4)}, error "
                    f"[{_err[0] * 1000:+.1f} {_err[1] * 1000:+.1f} "
                    f"{_err[2] * 1000:+.1f}] mm (|{_mag:.1f}| mm)"
                )
                if _mag > 15.0:
                    self.get_logger().error(
                        f"Arm is {_mag:.0f} mm from the commanded grasp pose "
                        "after a 'successful' approach. The gripper will close "
                        "on empty space. This is an EXECUTION fault, not a "
                        "perception or grasp-selection fault."
                    )
            except Exception as exc:  # noqa: BLE001 - diagnostics only
                self.get_logger().warn(f"Arrival check skipped: {exc}")
        else:
            self._fail_log("Cartesian approach execution failed - aborting")
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
                self._fail_log("Gripper grasp failed after retry - aborting retreat")
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
            self._fail_log(f"Retreat path incomplete (fraction={frac})")
            return False
        if not self._execute_trajectory(cart.solution):
            self._fail_log("Retreat execution failed")
            return False

        self.get_logger().info("Pick sequence complete")

        # Remember where this object was grasped, so a follow-up place goal
        # can descend to the same pose translated to the drop location.
        self._last_grasp_T_base = grasp_T_base.copy()
        self._held_cloud_hand = self._held_geometry_in_hand(
            object_pcd_file, T_base_from_cam, grasp_T_base)

        if place_after_s > 0.0:
            self.get_logger().info(f"Waiting {place_after_s:.1f}s before placing back down")
            time.sleep(place_after_s)

            # Step 5: straight-line descend back to the exact grasp pose.
            cart = self._compute_cartesian_path(group_name, eef_link, [grasp_pose_msg])
            if cart is None or cart.fraction < 1.0:
                frac = None if cart is None else cart.fraction
                self._fail_log(f"Place descent incomplete (fraction={frac})")
                return False
            if not self._execute_trajectory(cart.solution):
                self._fail_log("Place descent execution failed")
                return False

            # Step 6: open the gripper to release - Move, not Grasp, since
            # no object-contact force is expected on release.
            if not self._open_gripper(gripper_open_width, gripper_speed):
                self._fail_log("Gripper open (place) failed")
                return False

            # Step 7: retreat again, same as step 4.
            cart = self._compute_cartesian_path(group_name, eef_link, [retreat_pose_msg])
            if cart is None or cart.fraction < 1.0:
                frac = None if cart is None else cart.fraction
                self._fail_log(f"Post-place retreat incomplete (fraction={frac})")
                return False
            if not self._execute_trajectory(cart.solution):
                self._fail_log("Post-place retreat execution failed")
                return False

            self.get_logger().info("Place-back-in-place sequence complete")

        return True

    @staticmethod
    def _level_place_pose(place_T_base: np.ndarray):
        """A straight-down version of `place_T_base`, keeping its yaw.

        Grasp frame convention (grasp_transform): approach axis is column 2,
        finger axis is column 0. This rebuilds the rotation with the
        approach axis pointing straight down (base -Z) and the finger axis
        set to the horizontal projection of the grasp's finger axis, so the
        gripper still opens across the object with the least wrist motion.
        Translation is unchanged. Returns None if the finger axis is almost
        vertical (no well-defined yaw to keep).
        """
        R = place_T_base[:3, :3]
        f_horiz = np.array([R[0, 0], R[1, 0], 0.0])
        n = np.linalg.norm(f_horiz)
        if n < 1e-3:
            return None
        x_new = f_horiz / n
        z_new = np.array([0.0, 0.0, -1.0])
        y_new = np.cross(z_new, x_new)
        y_new /= np.linalg.norm(y_new)
        x_new = np.cross(y_new, z_new)

        out = place_T_base.copy()
        out[:3, :3] = np.column_stack([x_new, y_new, z_new])
        return out

    def execute_place(
        self,
        placement_object: str = "",
        group_name: str = "fr3_arm",
        eef_link: str = "fr3_hand",
        offset_xyz=(0.0, 0.20, 0.0),
        standoff_m: float = 0.12,
        lift_m: float = 0.12,
        gripper_open_width: float = 0.08,
        gripper_speed: float = 0.05,
        camera_frame: str = "zed_left_camera_frame_optical",
    ) -> tuple[bool, str]:
        """Place the currently-held object at an offset from where it was picked.

        Structurally this is the second half of execute_pick() run at a
        different location: descend to a target pose, open the gripper,
        retreat. The reused machinery is why place is cheap to add once
        pick works.

        Requires a preceding successful execute_pick() in this same node
        process - it needs `_last_grasp_T_base` to know the object's
        grasp pose and, more importantly, the approach ORIENTATION that
        was already proven reachable for this object. Re-deriving an
        orientation from scratch would risk a pose MoveIt cannot reach.

        `placement_object` is the planner's semantic target ("the plate").
        It IS resolved to a real pose now: _resolve_placement() detects that
        surface with the same GroundedSAM pipeline used for picking, chooses
        a point well inside its mask, and returns it in fr3_link0. Only the
        translation is taken from perception - the rotation stays the one
        that was already proven reachable on this object, for the same
        reason the fixed-offset path kept it.

        The height is not the bare surface point: the object hangs below the
        gripper, so the release pose is the perceived surface z, plus how far
        the gripper origin sat above the object's own lowest visible point
        when it grasped it (_hold_height), plus `place_clearance_m`.

        When no placement object is named, or perceiving it fails, the fixed
        `offset_xyz` is used instead - and the returned message says a
        perceived pose was attempted and why it was not used. That text is a
        deliverable: the planner writes it into STM and self-reflects on it,
        so "fell back to a fixed 20 cm offset because the plate was not
        detected" changes the next plan, while a bare False does not.

        Returns:
            (success, message) - message names the failing step, so the
            planner's success detector and the STM both get a real reason
            rather than a bare False.
        """
        if self._last_grasp_T_base is None:
            return False, (
                "place requested with no prior successful pick in this session - "
                "nothing is held, and there is no known grasp pose to place from"
            )

        # Target = the proven grasp pose with a new translation. Rotation
        # untouched either way: it is the one MoveIt already reached for this
        # object, and re-deriving one risks a pose it cannot.
        place_T_base = self._last_grasp_T_base.copy()

        point_base, placement_reason = self._resolve_placement(
            placement_object, camera_frame)

        zone = self._clearing_zone_xyz()
        used_clearing_zone = False
        if point_base is None and zone is not None and (
            not placement_object or not placement_object.strip()
        ):
            # No surface was named: this is a "move the obstacle out of the
            # way" place. Drop it at the configured clearing zone, fanned
            # out by how many obstacles have already gone there this
            # session, so a second object does not land on the first.
            step = np.asarray(
                self.get_parameter("clearing_zone_step_xyz").value, dtype=np.float64)
            target = zone + self._cleared_count * step

            hold_m, hold_reason = self._hold_height(camera_frame)
            clearance = float(self.get_parameter("place_clearance_m").value)
            place_T_base[:3, 3] = target
            place_T_base[2, 3] += hold_m + clearance
            used_clearing_zone = True
            placement_reason = (
                f"no placement surface named - treating this as clearing an "
                f"obstacle; releasing obstacle #{self._cleared_count + 1} at the "
                f"clearing zone [{target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}] m "
                f"in fr3_link0, {(hold_m + clearance) * 100:.1f} cm up ({hold_reason})"
            )
            self.get_logger().info(placement_reason)
        elif point_base is None:
            place_T_base[:3, 3] += np.asarray(offset_xyz, dtype=np.float64)
            self.get_logger().warn(placement_reason)
        else:
            hold_m, hold_reason = self._hold_height(camera_frame)
            clearance = float(self.get_parameter("place_clearance_m").value)
            place_T_base[:3, 3] = point_base
            place_T_base[2, 3] += hold_m + clearance
            placement_reason = (
                f"{placement_reason}; {hold_reason}; releasing "
                f"{(hold_m + clearance) * 100:.1f} cm above the perceived surface"
            )
            self.get_logger().info(placement_reason)

        self.get_logger().info(f"place pose (fr3_link0):\n{place_T_base}")

        # Step 1: MoveGroup to a pose directly above the drop point. Free
        # (non-Cartesian) motion here, same as pick's standoff step - the
        # arm may need to travel a long way and reorient.
        #
        # First try with the grasp's own orientation (proven reachable at
        # the PICK location). If MoveGroup cannot get there, the drop point
        # is usually just far enough from the pick that the wristy grasp
        # orientation is unreachable THERE - retry straight-down, keeping
        # only the grasp's yaw so the fingers still open across the object.
        # Placing is a release, not a grasp: the exact tilt does not matter.
        candidates = [("grasp orientation", place_T_base)]
        level_T_base = self._level_place_pose(place_T_base)
        if level_T_base is not None:
            candidates.append(("straight-down", level_T_base))

        chosen_T_base = None
        for label, cand_T in candidates:
            approach_T = cand_T.copy()
            approach_T[2, 3] += standoff_m
            if self._move_to_pose(
                group_name, eef_link, grasp_transform.matrix_to_pose(approach_T)
            ):
                chosen_T_base = cand_T
                if label != "grasp orientation":
                    self.get_logger().info(
                        f"place: reached the pre-place pose with a {label} "
                        "orientation (grasp orientation was unreachable there)")
                break

        if chosen_T_base is None:
            return False, f"place: failed to reach the pre-place approach pose ({placement_reason})"

        place_T_base = chosen_T_base

        # Step 2: straight-line descent to the place pose, slowed down so a
        # contact the perception did not predict (rim, misjudged floor
        # depth) trips the reflex gently rather than at full speed.
        place_pose_msg = grasp_transform.matrix_to_pose(place_T_base)
        cart = self._compute_cartesian_path(group_name, eef_link, [place_pose_msg])
        if cart is None or cart.fraction < 1.0:
            frac = None if cart is None else cart.fraction
            return False, (f"place: descent path incomplete (fraction={frac}) "
                           f"({placement_reason})")
        descent_factor = float(self.get_parameter("place_descent_speed_factor").value)
        if 0.0 < descent_factor < 1.0:
            grasp_transform.retime_trajectory(cart.solution.joint_trajectory, descent_factor)
        if not self._execute_trajectory(cart.solution):
            return False, f"place: descent execution failed ({placement_reason})"

        # Step 3: open to release. Move, not Grasp - no contact force wanted.
        if not self._open_gripper(gripper_open_width, gripper_speed):
            return False, "place: gripper open failed - object may still be held"

        # Step 4: lift clear of the placed object before anything else moves.
        retreat_T_base = place_T_base.copy()
        retreat_T_base[2, 3] += lift_m
        cart = self._compute_cartesian_path(
            group_name, eef_link, [grasp_transform.matrix_to_pose(retreat_T_base)]
        )
        if cart is None or cart.fraction < 1.0:
            frac = None if cart is None else cart.fraction
            # The object IS released at this point, so the place itself
            # succeeded - only the retreat failed. Report it as a failure
            # anyway: the arm is sitting on top of the placed object and a
            # human should look before the next goal runs.
            return False, f"place: object released but retreat incomplete (fraction={frac})"
        if not self._execute_trajectory(cart.solution):
            return False, "place: object released but retreat execution failed"

        # Held object is gone; a further place has nothing to place.
        self._last_grasp_T_base = None
        self._held_cloud_hand = None
        if used_clearing_zone:
            self._cleared_count += 1
        self.get_logger().info("Place sequence complete")
        return True, f"place completed - {placement_reason}"

    def _clearing_zone_xyz(self):
        """Parsed `clearing_zone_xyz` as a (3,) array, or None if unset.

        Accepts "x,y,z" (metres, fr3_link0). Anything that is not exactly
        three finite numbers is treated as unset - the caller then falls
        back to the relative `place_offset_xyz`, so a typo degrades to the
        old behaviour rather than flinging the arm at a garbage pose.
        """
        raw = str(self.get_parameter("clearing_zone_xyz").value or "").strip()
        if not raw:
            return None
        try:
            parts = [float(p) for p in raw.replace(" ", "").split(",")]
        except ValueError:
            self.get_logger().warn(
                f"clearing_zone_xyz={raw!r} is not 'x,y,z' - ignoring it and "
                "using place_offset_xyz instead")
            return None
        if len(parts) != 3 or not all(np.isfinite(parts)):
            self.get_logger().warn(
                f"clearing_zone_xyz={raw!r} must be three finite numbers - "
                "ignoring it and using place_offset_xyz instead")
            return None
        return np.asarray(parts, dtype=np.float64)

    def reset_clearing_zone(self) -> None:
        """Forget how many obstacles were placed aside this session.

        Call between independent tasks (or after a human resets the scene)
        so the next cleared obstacle lands at clearing_zone_xyz again
        rather than fanned far out from a previous run's count.
        """
        self._cleared_count = 0

    def execute_push(
        self,
        target_object: str,
        push_direction: str,
        group_name: str = "fr3_arm",
        eef_link: str = "fr3_hand",
        standoff_m: float = 0.12,
        lift_m: float = 0.12,
        gripper_speed: float = 0.05,
        camera_frame: str = "zed_left_camera_frame_optical",
        home_gripper_first: bool = True,
        with_held_object: bool = False,
    ) -> tuple[bool, str]:
        """Shove `target_object` `push_direction` ("left"/"right") along the table.

        `with_held_object`: the hand still holds the last picked object; push
        with it as the tool. The grip is kept (no homing, no closing), and the
        contact height and start point come from the held object's recorded
        geometry, so its lowest point runs tool_push_clearance_m above the
        table instead of the fingertips' push_height_min_m.

        A non-prehensile skill: no grasp, so none of the tilt/width/
        confidence gates apply and it works on objects a two-finger hand
        cannot pick (a wide cup, a sponge, a ball). The planner uses it to
        clear an obstruction before a pick.

        The direction is camera-relative, matching how the scene describer
        and vlm_task_planner talk about the world ("left"/"right" of the
        image). It is resolved to a base-frame ground-plane vector via the
        same TF lookup + calib correction the pick path uses.

        Sequence: perceive the object -> closed gripper, pointed straight
        down -> free move to a standoff beside the object on the side the
        push comes FROM -> slow straight-line descent to contact height ->
        slow straight-line lateral push through the object to
        centroid + push_distance -> lift clear.

        Returns (success, message); message names the failing step so the
        planner's self-reflection gets a real reason.
        """
        direction = (push_direction or "").strip().lower()
        if direction not in ("left", "right"):
            return False, (
                f"push aborted: push_direction must be 'left' or 'right', got "
                f"{push_direction!r}")

        self._add_table_collision()

        tool_hand = None
        if with_held_object:
            tool_hand = self._held_cloud_hand
            if self._last_grasp_T_base is None or tool_hand is None:
                return False, (
                    "push aborted: the hand is holding an object whose shape was not "
                    "recorded at pick time, so it cannot be used to push safely - "
                    "PLACE it first")
        else:
            if home_gripper_first and not self._home_gripper():
                return False, ("push aborted: gripper homing failed - check Desk "
                               "shows the end effector connected before retrying")
            # A near-closed hand is the pushing tool. Move, not Grasp - no
            # contact force wanted while closing in free space.
            push_w = float(self.get_parameter("push_gripper_width").value)
            if not self._open_gripper(push_w, gripper_speed):
                return False, "push aborted: could not close the gripper for pushing"

        # --- perceive the object ------------------------------------------
        if not self.get_parameter("use_live_perception").value or self._scene_source is None:
            return False, ("push aborted: live perception is disabled or no "
                           "RGB-D source is configured")
        try:
            rgb, depth, intrinsics = self._scene_source()
        except Exception as exc:  # noqa: BLE001
            return False, f"push aborted: could not capture an RGB-D frame ({type(exc).__name__}: {exc})"

        from pragmabot_bridge.live_perception import LivePerception
        perception = LivePerception(
            host=self.get_parameter("perception_host").value,
            perception_port=self.get_parameter("perception_port").value,
            graspgen_port=self.get_parameter("graspgen_port").value,
        )
        push_prompt, push_selector = self._split_spatial(target_object)
        cloud_cam, reason = perception.object_cloud_for(
            push_prompt, rgb, depth, intrinsics,
            max_mask_frac=float(self.get_parameter("push_max_mask_frac").value),
            disambiguate=push_selector)
        if cloud_cam is None:
            return False, f"push aborted: {reason}"

        try:
            self._wait_for_transform("fr3_link0", camera_frame)
            stamped = self._tf_buffer.lookup_transform(
                "fr3_link0", camera_frame, rclpy.time.Time())
            T_base_from_cam = grasp_transform.transform_to_matrix(stamped)
        except Exception as exc:  # noqa: BLE001
            return False, f"push aborted: TF lookup fr3_link0 <- {camera_frame} failed ({exc})"

        calib_correction = np.array([
            float(self.get_parameter("calib_correction_x").value),
            float(self.get_parameter("calib_correction_y").value),
            float(self.get_parameter("calib_correction_z").value),
        ])
        T_base_from_cam[:3, 3] += calib_correction

        cloud_base = (T_base_from_cam[:3, :3] @ np.asarray(cloud_cam, float).T).T + T_base_from_cam[:3, 3]
        centroid = cloud_base.mean(axis=0)
        obj_top_z = float(cloud_base[:, 2].max())

        # --- push direction in the base ground plane --------------------
        # Camera optical frame: +x points right in the image, so "left" is
        # -x_cam. Rotate that into the base frame, flatten to the ground
        # plane, normalise.
        dir_cam = np.array([-1.0, 0.0, 0.0]) if direction == "left" else np.array([1.0, 0.0, 0.0])
        dir_base = T_base_from_cam[:3, :3] @ dir_cam
        dir_base[2] = 0.0
        n = np.linalg.norm(dir_base)
        if n < 1e-6:
            return False, ("push aborted: the camera is looking straight down "
                           "the push axis, so 'left'/'right' has no ground-plane "
                           "direction")
        dir_base /= n

        # Object half-extent along the push axis, for where to start.
        half_extent = float(np.max(np.abs((cloud_base - centroid) @ dir_base)))
        margin = float(self.get_parameter("push_contact_margin_m").value)
        distance = float(self.get_parameter("push_distance_m").value)

        table_z = float(self.get_parameter("table_z").value)
        fingertip_offset = float(self.get_parameter("push_fingertip_offset_m").value)

        # Gripper pointed straight down, finger axis perpendicular to the
        # push so the broad side of the closed hand leads.
        z_axis = np.array([0.0, 0.0, -1.0])
        x_axis = np.array([-dir_base[1], dir_base[0], 0.0])
        x_axis /= np.linalg.norm(x_axis)
        y_axis = np.cross(z_axis, x_axis)
        R = np.column_stack([x_axis, y_axis, z_axis])

        if tool_hand is None:
            obj_height = obj_top_z - table_z
            frac = float(self.get_parameter("push_height_frac").value)
            tip_z = table_z + float(np.clip(
                frac * obj_height,
                float(self.get_parameter("push_height_min_m").value),
                float(self.get_parameter("push_height_max_m").value)))
            hand_z = tip_z + fingertip_offset
            start_xy = centroid - dir_base * (half_extent + margin)
            end_xy = centroid + dir_base * distance
            contact_desc = f"contacting at fingertip z={tip_z:.3f} m"
        else:
            # Held object's points relative to the hand origin, in base axes,
            # with the hand in the push orientation R.
            tool_off = (R @ tool_hand.T).T
            clearance = float(self.get_parameter("tool_push_clearance_m").value)
            tip_z = table_z + clearance
            # Never lower the fingertips below the tool's own clearance.
            hand_z = max(tip_z - float(tool_off[:, 2].min()), tip_z + fingertip_offset)
            lead = float((tool_off @ dir_base).max())
            lateral = tool_off @ x_axis
            lateral_mid = 0.5 * float(lateral.max() + lateral.min())
            # Tool's leading face starts `margin` behind the object's near
            # side, centred on the object across the push; travel is the
            # same as a bare-hand push.
            start_xy = (centroid - dir_base * (half_extent + margin + lead)
                        - x_axis * lateral_mid)
            end_xy = start_xy + dir_base * (half_extent + margin + distance)
            contact_desc = (f"pushing with the held object, its lowest point at "
                            f"z={tip_z:.3f} m")

        contact_T = np.eye(4)
        contact_T[:3, :3] = R
        contact_T[:3, 3] = [start_xy[0], start_xy[1], hand_z]

        approach_T = contact_T.copy()
        approach_T[2, 3] += standoff_m

        end_T = contact_T.copy()
        end_T[:3, 3] = [end_xy[0], end_xy[1], hand_z]

        note = (f"push {target_object!r} {direction}: object centroid "
                f"[{centroid[0]:.3f}, {centroid[1]:.3f}, {centroid[2]:.3f}] m, "
                f"{contact_desc}, shoving "
                f"{distance * 100:.0f} cm along "
                f"[{dir_base[0]:.2f}, {dir_base[1]:.2f}]")
        self.get_logger().info(note)
        self.get_logger().info(f"push contact pose (fr3_link0):\n{contact_T}")

        speed_factor = float(self.get_parameter("push_speed_factor").value)

        def _slow(cart):
            if 0.0 < speed_factor < 1.0:
                grasp_transform.retime_trajectory(cart.solution.joint_trajectory, speed_factor)
            return cart

        # Step 1: free move to the standoff beside the object.
        if not self._move_to_pose(
            group_name, eef_link, grasp_transform.matrix_to_pose(approach_T)):
            return False, f"push: failed to reach the pre-push standoff ({note})"

        # Step 2: slow straight-line descent to contact height.
        cart = self._compute_cartesian_path(
            group_name, eef_link, [grasp_transform.matrix_to_pose(contact_T)])
        if cart is None or cart.fraction < 1.0:
            frac_got = None if cart is None else cart.fraction
            return False, f"push: descent path incomplete (fraction={frac_got}) ({note})"
        if not self._execute_trajectory(_slow(cart).solution):
            return False, f"push: descent execution failed ({note})"

        # Step 3: slow straight-line lateral push through the object.
        cart = self._compute_cartesian_path(
            group_name, eef_link, [grasp_transform.matrix_to_pose(end_T)])
        if cart is None or cart.fraction < 1.0:
            frac_got = None if cart is None else cart.fraction
            # Retreat up before reporting - do not leave the hand down by
            # the object.
            up = contact_T.copy()
            up[2, 3] += lift_m
            retr = self._compute_cartesian_path(
                group_name, eef_link, [grasp_transform.matrix_to_pose(up)])
            if retr is not None and retr.fraction >= 1.0:
                self._execute_trajectory(retr.solution)
            return False, f"push: lateral path incomplete (fraction={frac_got}) ({note})"
        if not self._execute_trajectory(_slow(cart).solution):
            return False, f"push: lateral push execution failed ({note})"

        # Step 4: lift clear.
        up = end_T.copy()
        up[2, 3] += lift_m
        cart = self._compute_cartesian_path(
            group_name, eef_link, [grasp_transform.matrix_to_pose(up)])
        if cart is None or cart.fraction < 1.0:
            frac_got = None if cart is None else cart.fraction
            return False, f"push: object moved but retreat incomplete (fraction={frac_got}) ({note})"
        if not self._execute_trajectory(cart.solution):
            return False, f"push: object moved but retreat execution failed ({note})"

        self.get_logger().info("Push sequence complete")
        return True, f"push completed - {note}"

    # ------------------------------------------------------------------
    # ExecuteSkill action server - the planner <-> robot connection
    # ------------------------------------------------------------------

    # Spatial qualifiers the planner may prefix onto target_object to pick
    # one of several identical objects. Mapped to the selector the
    # perception server understands. Kept in sync with _SPATIAL_SELECTORS /
    # _SPATIAL_ALIASES in calibration/perception_server.py.
    _SPATIAL_PREFIXES = {
        "left": "left", "leftmost": "left",
        "right": "right", "rightmost": "right",
        "front": "near", "frontmost": "near", "nearest": "near",
        "near": "near", "closest": "near",
        "back": "far", "rear": "far", "backmost": "far",
        "far": "far", "farthest": "far", "furthest": "far",
        "top": "top", "topmost": "top",
        "bottom": "bottom", "bottommost": "bottom",
        "largest": "largest", "biggest": "largest", "smallest": "smallest",
    }

    @classmethod
    def _split_spatial(cls, target_object: str):
        """(clean_prompt, selector) from a planner target like "left wooden cube".

        Strips leading articles then a single leading spatial word. The
        spatial word is NOT sent to the detector (GroundingDINO handles
        "left" poorly as a noun-phrase modifier); it goes to the server as
        a `disambiguate` selector instead. No qualifier -> (original, "").
        """
        words = (target_object or "").strip().split()
        while words and words[0].lower() in ("the", "a", "an"):
            words = words[1:]
        if len(words) >= 2 and words[0].lower() in cls._SPATIAL_PREFIXES:
            return " ".join(words[1:]), cls._SPATIAL_PREFIXES[words[0].lower()]
        return target_object, ""

    def _resolve_grasps(self, target_object: str):
        """Turn the planner's target_object into a grasp file, live.

        Runs GroundedSAM -> cloud -> GraspGen for THIS object and writes the
        result in the same .npz layout graspgen_client.py --save_grasps
        produces, so execute_pick() and grasp_transform.load_all_grasps()
        are reused unchanged - including the centroid convention, which is
        the easiest thing to get silently wrong here.

        Falls back to the legacy `grasp_file` parameter when
        `use_live_perception` is false, so offline replay against a saved
        npz still works.

        Returns:
            (grasp_file_path, reason). path is "" on failure, and reason is
            the text the planner will reflect on.
        """
        if not self.get_parameter("use_live_perception").value:
            legacy = self.get_parameter("grasp_file").get_parameter_value().string_value
            if not legacy:
                return "", (
                    "use_live_perception is false and the grasp_file parameter "
                    "is empty - either start the perception servers or point "
                    "grasp_file at a saved .npz")
            return legacy, f"using pre-computed {legacy} (live perception disabled)"

        if self._scene_source is None:
            return "", (
                "live perception is enabled but no RGB-D source is configured; "
                "set the rgb/depth/camera_info topics or disable "
                "use_live_perception")

        try:
            rgb, depth, intrinsics = self._scene_source()
        except Exception as exc:  # noqa: BLE001
            return "", f"could not capture an RGB-D frame: {type(exc).__name__}: {exc}"

        from pragmabot_bridge.live_perception import LivePerception

        perception = LivePerception(
            host=self.get_parameter("perception_host").value,
            perception_port=self.get_parameter("perception_port").value,
            graspgen_port=self.get_parameter("graspgen_port").value,
            grasp_topk=int(self.get_parameter("grasp_topk").value),
            num_grasps=int(self.get_parameter("graspgen_num_grasps").value),
        )
        prompt, selector = self._split_spatial(target_object)
        if selector:
            self.get_logger().info(
                f"target {target_object!r} -> prompt {prompt!r}, "
                f"disambiguate={selector!r}")
        grasps, confidences, cloud, reason = perception.grasps_for(
            prompt, rgb, depth, intrinsics, disambiguate=selector)
        if grasps is None:
            return "", reason

        # Re-centre before saving so the file matches what
        # graspgen_client.py --save_grasps writes, and load_all_grasps()
        # adds the centroid back exactly once.
        centroid = cloud[:, :3].mean(axis=0)
        out = Path(tempfile.gettempdir()) / "pragmabot_live_grasps.npz"
        saved = grasps.copy()
        saved[:, :3, 3] -= centroid
        np.savez(out, grasps=saved, centroid=centroid, confidences=confidences)
        np.save(out.with_name("pragmabot_live_cloud.npy"), cloud)
        return str(out), reason

    def _capture_scene(self):
        """One live RGB-D frame as (rgb, depth, intrinsics). The default
        `_scene_source`.

        Delegates to calibration/capture_scene.py, which is pure
        rclpy+numpy (no cv_bridge) and handles the two traps that make this
        fail silently rather than loudly: the ZED's BEST_EFFORT QoS, which
        a default RELIABLE subscription never matches, and the 32FC1-metres
        vs 16UC1-millimetres depth encoding split, which is a 1000x error
        that passes every downstream guard because only the SCALE is wrong.
        """
        from pragmabot_bridge.live_perception import _calibration_dir
        _calibration_dir()
        import capture_scene  # noqa: PLC0415

        return capture_scene.capture(
            color_topic=self.get_parameter("color_topic").value,
            depth_topic=self.get_parameter("depth_topic").value,
            info_topic=self.get_parameter("camera_info_topic").value,
            timeout_s=self.get_parameter("capture_timeout_s").value,
            verbose=False,
        )

    def _resolve_placement(self, placement_object: str, camera_frame: str):
        """Turn the planner's placement_object into a point in fr3_link0.

        The placement counterpart of _resolve_grasps(), and deliberately
        NOT a call to GraspGen: a surface is not a graspable object, so
        there is no grasp to generate. What comes back is one point on the
        surface - detected with the same GroundedSAM server and the same
        guards as picking, chosen away from the surface's rim, and
        back-projected with mask_to_pointcloud's own pinhole code.

        Every failure path returns (None, reason) rather than raising, and
        every reason states that a perceived pose WAS attempted and what
        stopped it. execute_place() then falls back to place_offset_xyz and
        puts that sentence in the ExecuteSkill message, where the planner's
        self-reflection reads it. A silent fallback would let the planner
        believe "place on the plate" succeeded on the plate.

        Returns:
            (point_base, reason). point_base is (3,) in fr3_link0, or None.
        """
        if not placement_object or not placement_object.strip():
            return None, (
                "no placement object was named in the action, so the object was "
                "released at the fixed place_offset_xyz relative to where it was "
                "picked up - not on any perceived surface")

        prefix = (f"a perceived placement pose on {placement_object!r} was "
                  f"attempted but ")
        suffix = ("; the object was released at the fixed place_offset_xyz "
                  "instead, so it is NOT necessarily on "
                  f"the {placement_object}")

        if not self.get_parameter("use_live_perception").value:
            return None, (prefix + "use_live_perception is false on this node"
                          + suffix)

        if self._scene_source is None:
            return None, (prefix + "the bridge has no RGB-D source configured "
                          "(_scene_source is unset)" + suffix)

        try:
            rgb, depth, intrinsics = self._scene_source()
        except Exception as exc:  # noqa: BLE001
            return None, (prefix + "an RGB-D frame could not be captured "
                          f"({type(exc).__name__}: {exc})" + suffix)

        from pragmabot_bridge.live_perception import LivePerception

        perception = LivePerception(
            host=self.get_parameter("perception_host").value,
            perception_port=self.get_parameter("perception_port").value,
            graspgen_port=self.get_parameter("graspgen_port").value,
            grasp_topk=int(self.get_parameter("grasp_topk").value),
            num_grasps=int(self.get_parameter("graspgen_num_grasps").value),
        )
        point_cam, reason, _info = perception.surface_point_for(
            placement_object, rgb, depth, intrinsics,
            n_candidates=self.get_parameter("placement_fps_candidates").value,
            min_interior_px=self.get_parameter("placement_interior_px").value,
            patch_px=self.get_parameter("placement_patch_px").value,
            max_mask_frac=self.get_parameter("placement_max_mask_frac").value,
        )
        if point_cam is None:
            return None, prefix + reason + suffix

        # Same TF pattern execute_pick() uses - one lookup_transform resolves
        # the whole fr3_link0 -> ... -> optical chain.
        try:
            self._wait_for_transform("fr3_link0", camera_frame)
            stamped = self._tf_buffer.lookup_transform(
                "fr3_link0", camera_frame, rclpy.time.Time()
            )
            T_base_from_cam = grasp_transform.transform_to_matrix(stamped)
        except Exception as exc:  # noqa: BLE001
            return None, (prefix + f"the TF lookup fr3_link0 <- {camera_frame} "
                          f"failed ({exc})" + suffix)

        point_base = (T_base_from_cam @ np.append(np.asarray(point_cam, float), 1.0))[:3]
        return point_base, (
            f"{reason}; in fr3_link0 that is "
            f"[{point_base[0]:.3f}, {point_base[1]:.3f}, {point_base[2]:.3f}] m")

    def _hold_height(self, camera_frame: str) -> tuple[float, str]:
        """How far the gripper origin sat above the held object's lowest point.

        Needed because a perceived surface point is where the OBJECT must
        end up, while the pose commanded to MoveIt is where the GRIPPER
        goes. The offset between them is the object's own height above its
        grasp, which is measured, not assumed: the cloud saved during the
        pick is transformed into fr3_link0 and its minimum z is subtracted
        from the grasp pose's z.

        Returns (metres, reason). On any failure it returns 0.0 and says so -
        the object is then released from the surface plus the clearance only,
        which drops it from roughly its own height. That is a real behaviour
        change worth reflecting on, so it goes in the message.
        """
        unknown = ("the held object's height above the grasp could not be "
                   "measured, so the release height is the clearance alone "
                   "and the object will drop from about its own height")

        path = self._live_cloud_path()
        if not path or not Path(path).is_file() or self._last_grasp_T_base is None:
            return 0.0, unknown

        try:
            cloud_cam = np.load(path).astype(np.float64)[:, :3]
            stamped = self._tf_buffer.lookup_transform(
                "fr3_link0", camera_frame, rclpy.time.Time()
            )
            T = grasp_transform.transform_to_matrix(stamped)
            cloud_base = (T[:3, :3] @ cloud_cam.T).T + T[:3, 3]
            hold = float(self._last_grasp_T_base[2, 3] - cloud_base[:, 2].min())
        except Exception:  # noqa: BLE001
            return 0.0, unknown

        # A gripper more than half a metre above the thing it is holding, or
        # below it, means the cloud and the grasp are not the same object.
        if not 0.0 <= hold <= 0.5:
            return 0.0, (f"the measured hold height {hold:.3f} m is not "
                         "physically plausible, so it was ignored; " + unknown)

        return hold, (f"the gripper held the object {hold * 100:.1f} cm above "
                      "its lowest visible point")

    def _held_geometry_in_hand(self, object_pcd_file: str, T_base_from_cam: np.ndarray,
                               grasp_T_base: np.ndarray):
        """The just-grasped object's points in the fr3_hand frame, or None.

        The camera sees mostly the top and one side, so the visible cloud's
        lowest point is above the real bottom. The object was resting on the
        table when grasped, so its footprint is also added at table_z. If it
        was actually stacked, that makes the tool look longer than it is, and
        a tool push then runs higher, not into the table.
        """
        if not object_pcd_file or not Path(object_pcd_file).is_file():
            self.get_logger().warn("held-object geometry not recorded: no object cloud")
            return None
        try:
            cloud_cam = np.load(object_pcd_file).astype(np.float64)[:, :3]
            cloud_base = (T_base_from_cam[:3, :3] @ cloud_cam.T).T + T_base_from_cam[:3, 3]
            footprint = cloud_base.copy()
            footprint[:, 2] = float(self.get_parameter("table_z").value)
            pts = np.vstack([cloud_base, footprint])
            inv = np.linalg.inv(grasp_T_base)
            return (inv[:3, :3] @ pts.T).T + inv[:3, 3]
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"held-object geometry not recorded: {exc}")
            return None

    def _live_cloud_path(self) -> str:
        """Cloud from the most recent live detection, else the parameter.

        execute_pick() uses this only to estimate gripper width at the
        grasp contact point; falling back to the parameter keeps offline
        replay working.
        """
        live = Path(tempfile.gettempdir()) / "pragmabot_live_cloud.npy"
        if self.get_parameter("use_live_perception").value and live.is_file():
            return str(live)
        return self.get_parameter("object_pcd_file").get_parameter_value().string_value

    def _execute_skill_cb(self, goal_handle):
        """Dispatch one planner skill decision onto the robot.

        Blocking by design; see the module docstring on why this needs a
        MultiThreadedExecutor. Never raises out of the callback - an
        uncaught exception here would abort the goal with no message, and
        the planner's STM would record a failure with no reason to reflect
        on. Every path returns a populated `message`.
        """
        request = goal_handle.request
        skill = request.chosen_skill.lower()
        # Clear per goal: a reason left over from the previous skill would be
        # reported as this one's cause, which is worse than saying nothing.
        self._last_failure = ""
        self.get_logger().info(
            f"ExecuteSkill goal: skill={skill!r} target={request.target_object!r} "
            f"placement={request.placement_object!r}"
        )

        def feedback(status: str) -> None:
            msg = ExecuteSkill.Feedback()
            msg.status = status
            goal_handle.publish_feedback(msg)

        result = ExecuteSkill.Result()
        try:
            if skill == "pick":
                feedback("perceiving")
                # Resolve the planner's target_object to grasps NOW, rather
                # than reading a pre-computed grasp_file. Without this the
                # bridge picks whatever was segmented in the last offline
                # run, and a "success" on the wrong object writes a
                # fabricated entry into the graded memory.
                grasp_file, perception_reason = self._resolve_grasps(request.target_object)
                if not grasp_file:
                    result.success = False
                    # The reason names the object, the confidences and the
                    # guard that fired. It goes into STM and the VLM
                    # reflects on it, so pass it through verbatim.
                    result.message = f"pick aborted: {perception_reason}"
                    self.get_logger().error(result.message)
                else:
                    self.get_logger().info(f"perception: {perception_reason}")
                    feedback("picking")
                    ok = self.execute_pick(
                        grasp_file,
                        group_name=self.get_parameter("group_name").value,
                        eef_link=self.get_parameter("eef_link").value,
                        standoff_m=self.get_parameter("standoff_m").value,
                        lift_m=self.get_parameter("lift_m").value,
                        gripper_width=self.get_parameter("gripper_width").value,
                        gripper_speed=self.get_parameter("gripper_speed").value,
                        gripper_force=self.get_parameter("gripper_force").value,
                        gripper_epsilon=self.get_parameter("gripper_epsilon").value,
                        camera_frame=self.get_parameter("camera_frame").value,
                        # Prefer the cloud live perception just produced, so
                        # the gripper width is estimated from THIS object
                        # rather than a stale file.
                        object_pcd_file=self._live_cloud_path(),
                        grasp_index=self.get_parameter("grasp_index").value,
                        place_after_s=0.0,  # the planner decides when to place
                        gripper_open_width=self.get_parameter("gripper_open_width").value,
                        home_gripper_first=self.get_parameter("home_gripper_first").value,
                    )
                    result.success = ok
                    result.message = (
                        f"picked {request.target_object}"
                        if ok
                        else (self._last_failure or "pick failed for an unrecorded reason")
                    )

            elif skill == "place":
                feedback("placing")
                ok, message = self.execute_place(
                    placement_object=request.placement_object,
                    group_name=self.get_parameter("group_name").value,
                    eef_link=self.get_parameter("eef_link").value,
                    offset_xyz=self.get_parameter("place_offset_xyz").value,
                    standoff_m=self.get_parameter("standoff_m").value,
                    lift_m=self.get_parameter("lift_m").value,
                    gripper_open_width=self.get_parameter("gripper_open_width").value,
                    gripper_speed=self.get_parameter("gripper_speed").value,
                    camera_frame=self.get_parameter("camera_frame").value,
                )
                result.success = ok
                result.message = message

            elif skill == "push":
                feedback("pushing")
                ok, message = self.execute_push(
                    target_object=request.target_object,
                    push_direction=request.push_direction,
                    group_name=self.get_parameter("group_name").value,
                    eef_link=self.get_parameter("eef_link").value,
                    standoff_m=self.get_parameter("standoff_m").value,
                    lift_m=self.get_parameter("lift_m").value,
                    gripper_speed=self.get_parameter("gripper_speed").value,
                    camera_frame=self.get_parameter("camera_frame").value,
                    home_gripper_first=self.get_parameter("home_gripper_first").value,
                    with_held_object=bool(request.push_with_held_object),
                )
                result.success = ok
                result.message = message

            else:
                result.success = False
                result.message = f"unknown skill: {skill!r} (expected pick, place or push)"

        except Exception as exc:  # noqa: BLE001 - must not escape the callback
            self.get_logger().error(f"{skill} raised: {exc}")
            result.success = False
            result.message = f"{skill} raised an exception: {exc}"

        # succeed() regardless of result.success: the ACTION completed
        # normally, and `success` is the outcome it carries. abort() would
        # discard the message, which is exactly the reason the planner
        # needs in order to self-reflect.
        goal_handle.succeed()
        self.get_logger().info(
            f"ExecuteSkill done: success={result.success} message={result.message!r}"
        )
        return result

    # ------------------------------------------------------------------
    # MoveIt goal helpers
    # ------------------------------------------------------------------

    def _spin_until_done(self, future, timeout_sec: float | None = None):
        """Wait for `future` from inside a skill callback.

        The skill callbacks run under a MultiThreadedExecutor with a
        ReentrantCallbackGroup, so OTHER executor threads keep servicing the
        node while this one waits - a plain poll on future.done() is enough.
        The module-level rclpy.spin_until_future_complete() must NOT be used
        here: it spins up its own throwaway executor and add_node() on a
        node the main executor already owns silently fails, which after the
        first completed goal leaves the main executor no longer dispatching
        the action server (the bridge then freezes on the next goal, e.g.
        a `place` after a `pick`).
        """
        end = None if timeout_sec is None else time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done():
            if end is not None and time.monotonic() > end:
                break
            time.sleep(0.005)
        return future

    def _wait_for_transform(self, target_frame: str, source_frame: str, timeout_sec: float = 10.0):
        start = time.time()
        while not self._tf_buffer.can_transform(target_frame, source_frame, rclpy.time.Time()):
            time.sleep(0.05)
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

    def _on_joint_state(self, msg: JointState) -> None:
        self._latest_joint_state = msg

    def _ik_joint_config(self, group_name: str, link_name: str, pose: Pose,
                          timeout_s: float = 0.5):
        """Solve IK for `pose`, seeded from the live joint state, for
        SCORING a candidate before any motion is attempted - never for
        execution (that stays on _compute_cartesian_path/_move_to_pose and
        their own guards). Returns (names, positions) from the solution,
        or None if no live joint state has arrived yet, the service isn't
        up, or the solve fails/times out - callers must treat None as
        "couldn't check", not "candidate is bad".
        """
        if self._latest_joint_state is None:
            return None
        if not self._ik_client.service_is_ready():
            if not self._ik_client.wait_for_service(timeout_sec=1.0):
                return None

        request = GetPositionIK.Request()
        request.ik_request.group_name = group_name
        request.ik_request.ik_link_name = link_name
        request.ik_request.avoid_collisions = True
        request.ik_request.robot_state.joint_state = self._latest_joint_state
        request.ik_request.pose_stamped.header.frame_id = "fr3_link0"
        request.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        request.ik_request.pose_stamped.pose = pose
        request.ik_request.timeout.sec = 0
        request.ik_request.timeout.nanosec = int(timeout_s * 1e9)

        future = self._ik_client.call_async(request)
        self._spin_until_done(future, timeout_sec=timeout_s + 1.0)
        result = future.result()
        if result is None or result.error_code.val != MoveItErrorCodes.SUCCESS:
            return None
        js = result.solution.joint_state
        return list(js.name), list(js.position)

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
        """Plan a straight line through `waypoints`, refining the step on failure.

        WHY A LOOP, AND WHY IT CHANGES THE REQUEST EACH TIME
        ----------------------------------------------------
        /compute_cartesian_path is DETERMINISTIC - unlike OMPL, it runs no
        randomised sampling. Calling it again with an identical request
        returns an identical fraction, so the usual "retry N times and hope"
        loop is N times the wait for the same answer. Each retry here
        therefore HALVES max_step (the eef_step) instead of repeating.

        That is a real second chance, not a re-roll. The service walks the
        line in max_step increments and IK-solves each one, seeding each
        solve from the previous solution; it truncates at the first
        increment it cannot solve. A smaller step is a smaller extrapolation
        from a known-good seed, so increments that failed at 1cm are often
        solvable at 5mm or 2.5mm - a finer discretisation of the SAME
        straight line finds a continuous joint path where the coarse one hit
        a wall.

        Note that revolute_jump_threshold is NOT what truncates here: the
        service accepts the field but does not forward it to the
        interpolator (moveit2 #2404). That is why the jump guard is applied
        client-side in _execute_trajectory instead of being trusted to the
        request.

        The straight line itself never changes. Only how finely it is
        sampled does, so a plan accepted at 1mm is the same motion the 1cm
        attempt was asking for - it is not a detour bought by relaxing a
        safety check.

        Returns the best result obtained (highest `fraction`), or None if
        the service call itself failed. Callers still enforce
        `fraction >= 1.0`; this only stops them failing over a step size.
        """
        max_step = float(self.get_parameter("cartesian_max_step").value)
        min_step = float(self.get_parameter("cartesian_min_step").value)
        max_tries = int(self.get_parameter("cartesian_max_tries").value)
        # Truncate the path (rather than execute through it) if any single
        # max_step-sized Cartesian step would require an unusually large
        # single-joint angle change -- the direct symptom of the IK
        # solution passing near a singularity. 0.2 rad (~11 deg) per 1cm
        # Cartesian step is a conservative starting point, not empirically
        # tuned for this robot/workspace -- loosen or tighten via the
        # revolute_jump_threshold parameter if it aborts on fine motions
        # or doesn't catch a real one.
        #
        # NOTE: 0.0 does NOT mean "disallow jumps". In MoveIt it means
        # "disable the check entirely", so every jump is accepted and the
        # arm may flip a joint mid-line at full planned speed. Leave this
        # positive on hardware.
        jump = float(self.get_parameter("revolute_jump_threshold").value)
        if jump <= 0.0:
            self.get_logger().warn(
                "revolute_jump_threshold=0.0 DISABLES the joint-jump check "
                "- near-singular flips will be accepted, not rejected."
            )

        self._cartesian_client.wait_for_service()

        schedule = cartesian_step_schedule(max_step, min_step, max_tries)
        best = None
        for attempt, max_step in enumerate(schedule, start=1):
            request = GetCartesianPath.Request()
            request.header.frame_id = "fr3_link0"
            request.header.stamp = self.get_clock().now().to_msg()
            request.group_name = group_name
            request.link_name = link_name
            request.waypoints = waypoints
            request.max_step = max_step
            request.avoid_collisions = True
            request.revolute_jump_threshold = jump

            future = self._cartesian_client.call_async(request)
            self._spin_until_done(future, timeout_sec=15.0)
            result = future.result()

            if result is None:
                self.get_logger().error(
                    "/compute_cartesian_path service call failed on attempt "
                    f"{attempt} (max_step={max_step:.4f} m)"
                )
                return best

            if best is None or result.fraction > best.fraction:
                best = result

            if result.fraction >= 1.0:
                if attempt > 1:
                    self.get_logger().info(
                        f"Cartesian path complete on attempt {attempt} after "
                        f"refining max_step to {max_step:.4f} m"
                    )
                return result

            if attempt == len(schedule):
                break

            self.get_logger().warn(
                f"Cartesian path incomplete (fraction={result.fraction:.3f}) at "
                f"max_step={max_step:.4f} m - retrying with a finer step"
            )

        self.get_logger().error(
            f"Cartesian path incomplete after {len(schedule)} attempt(s), finest "
            f"step {schedule[-1]:.4f} m (best fraction={best.fraction:.3f}). "
            "Refining further will not help: either the jump threshold is "
            "truncating on a persistent near-singular joint flip, or the "
            "straight line is genuinely unreachable - goal out of range, "
            "blocked by a collision object, or the arm needs a different "
            "starting configuration."
        )
        return best

    def _execute_trajectory(self, robot_trajectory) -> bool:
        """Vet a trajectory against the real FR3 limits, then execute it.

        THE GAP THIS CLOSES
        -------------------
        MoveIt hands back a trajectory timed against the URDF's CONSTANT
        velocity limit. The FR3's actual limit shrinks as a joint nears its
        position limit, and a URDF cannot express that, so the planner is
        structurally blind to it: it will happily return a trajectory that
        libfranka then refuses at 1 kHz with "speed limits reached", after
        the arm has already started moving.

        This is the last point before the trajectory leaves our process, so
        it is the only place the check can be made once and cover every
        motion - free-space and Cartesian alike.

        Slowing down is EXACT, not a heuristic: re-timing is a
        reparameterisation of the same geometric path, so no waypoint moves
        and a collision-free path stays collision-free (see
        grasp_transform.retime_trajectory). What re-timing cannot fix is a
        configuration so close to a limit that no speed is legal - there
        the answer is a different grasp, and this refuses rather than
        moving.
        """
        jt = robot_trajectory.joint_trajectory

        if self.get_parameter("enforce_velocity_limits").value:
            safety = float(self.get_parameter("velocity_safety_margin").value)

            jump = grasp_transform.max_joint_jump(jt)
            jump_max = float(self.get_parameter("max_joint_jump_rad").value)
            if jump > jump_max:
                self.get_logger().error(
                    f"Refusing to execute: largest single-joint step between "
                    f"consecutive waypoints is {jump:.3f} rad, above the "
                    f"{jump_max:.3f} rad guard. Two waypoints ~1cm apart in "
                    "Cartesian space needing that much joint motion is the "
                    "signature of a near-singular configuration - the arm "
                    "would flip through it at speed. Pick a different grasp."
                )
                return False

            ok, reason, _ = self._limits.check_trajectory(jt, safety)
            if not ok:
                factor = self._limits.retime_factor_for(jt, safety)
                if factor is None:
                    self.get_logger().error(f"Refusing to execute: {reason}")
                    return False
                self.get_logger().warn(
                    f"{reason} - slowing the trajectory to {factor * 100:.0f}% "
                    "of its planned speed. The path is unchanged; only its "
                    "timing is."
                )
                grasp_transform.retime_trajectory(jt, factor)

                ok, reason, _ = self._limits.check_trajectory(jt, safety)
                if not ok:
                    self.get_logger().error(
                        f"Refusing to execute: still illegal after re-timing - {reason}"
                    )
                    return False

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = robot_trajectory
        result = self._send_goal_blocking(self._execute_client, goal, "ExecuteTrajectory")
        if result is None:
            return False
        return result.result.error_code.val == MoveItErrorCodes.SUCCESS

    def _grasp(self, width: float, speed: float, force: float, epsilon: float) -> bool:
        """Close on an object of roughly `width` metres.

        THE EPSILON IS DELIBERATELY ASYMMETRIC. franka_msgs/Grasp reports
        success only if the fingers stop inside
        [width - inner, width + outer]. This used to pass the SAME epsilon
        for both, and with a width estimate that is biased low (a
        single-view cloud under-measures whatever the fingers cannot see)
        a symmetric +/-20 mm band around, say, 9 mm becomes 0..29 mm --
        which reports SUCCESS when the fingers close on empty air and
        FAILURE when they close on the real 45 mm object. Observed
        directly on this robot: empty closes were lifted, real grasps were
        abandoned before the retreat.

        inner is therefore small (closing far NARROWER than commanded means
        nothing is in the hand - a genuine failure) and outer is generous
        (closing WIDER than commanded just means the object is bigger than
        our estimate, which is the expected direction of the error and
        still a real grasp).

        gripper_min_held_width then widens inner down to a fixed finger gap,
        so a squeezed sponge counts while a close on air (gap ~0) still fails.
        """
        inner = float(self.get_parameter("gripper_epsilon_inner").value)
        outer = float(self.get_parameter("gripper_epsilon_outer").value)
        min_held = float(self.get_parameter("gripper_min_held_width").value)
        if min_held > 0.0 and width - min_held > inner:
            inner = width - min_held

        goal = Grasp.Goal()
        goal.width = width
        goal.speed = speed
        goal.force = force
        goal.epsilon = GraspEpsilon(inner=inner, outer=outer)

        result = self._send_goal_blocking(self._gripper_client, goal, "Grasp")
        if result is None:
            return False
        if not result.result.success:
            self.get_logger().error(f"Grasp reported failure: {result.result.error}")
            return False
        return True


    def _add_table_collision(self) -> bool:
        """Push a tabletop box into MoveIt's planning scene, once per run.

        WHY THIS IS NOT OPTIONAL. move_group starts with an empty world.
        With no table in it, OMPL and the Cartesian interpolator are both
        free to route the arm THROUGH the tabletop on the way to a
        standoff pose - and did, repeatedly, on every object tried: the
        grasp pose itself measured correct (0.0 mm off centre, 8.7 deg
        from vertical, confidence 0.967) while the PATH to it had to be
        stopped by hand. Nothing about the grasp, the object or the
        calibration could fix that, because none of them are what plans
        the motion.

        Uses the /apply_planning_scene SERVICE rather than publishing to
        /planning_scene: a message published before move_group's
        subscriber is connected is dropped silently, and the first pick
        then plans against an empty world anyway - the exact failure this
        is here to prevent. The service call is acknowledged.

        Returns True if the scene now contains the table.
        """
        if self._table_in_scene:
            return True
        if not bool(self.get_parameter("add_table_collision").value):
            return True
        if not self._scene_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(
                "/apply_planning_scene unavailable - continuing WITHOUT a table "
                "in the planning scene. Watch the standoff move: nothing stops "
                "MoveIt routing the arm through the tabletop."
            )
            return False

        table_z = float(self.get_parameter("table_z").value)
        thickness = float(self.get_parameter("table_thickness").value)

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [
            float(self.get_parameter("table_size_x").value),
            float(self.get_parameter("table_size_y").value),
            thickness,
        ]

        pose = Pose()
        pose.position.x = float(self.get_parameter("table_center_x").value)
        pose.position.y = 0.0
        # Box CENTRE sits half a thickness below the surface, so its top
        # face lands exactly on table_z rather than floating above it.
        pose.position.z = table_z - thickness / 2.0
        pose.orientation.w = 1.0

        table = CollisionObject()
        table.header = Header(frame_id="fr3_link0")
        table.id = "table"
        table.primitives = [box]
        table.primitive_poses = [pose]
        table.operation = CollisionObject.ADD

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [table]

        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self._scene_client.call_async(request)
        self._spin_until_done(future, timeout_sec=10.0)
        result = future.result()
        if result is None or not result.success:
            self.get_logger().warn("Applying the table collision object failed")
            return False

        self._table_in_scene = True
        self.get_logger().info(
            f"Table added to the planning scene: top at z={table_z:.3f} m, "
            f"{box.dimensions[0]:.2f} x {box.dimensions[1]:.2f} m centred at "
            f"x={pose.position.x:.2f}. Plans can no longer pass through it."
        )
        return True

    def _home_gripper(self) -> bool:
        # Homing fully opens the hand, so whatever it held is no longer held.
        self._last_grasp_T_base = None
        self._held_cloud_hand = None
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

    def _fail_log(self, reason: str) -> None:
        """Log an error AND retain it as the reason the current skill failed.

        Everything in the pick path reports through here so the action result
        can carry the real cause back to the planner.
        """
        self._last_failure = reason
        self.get_logger().error(reason)

    def _send_goal_blocking(self, client: ActionClient, goal_msg, name: str):
        """Send an action goal and wait for its result, bounded at every step.

        WHY THE TIMEOUTS ARE NOT OPTIONAL
        ---------------------------------
        Every wait here used to be unbounded, and a dead action server is
        indistinguishable from a slow one when you are blocked forever. It
        happened: the User Stop was pressed, libfranka dropped the gripper
        connection, franka_gripper_node crashed on a Poco NetException - and
        its action names stayed advertised in the DDS graph afterwards. The
        bridge sent a homing goal to `Action servers: 0`, and simply stopped,
        with no error, no feedback and no arm motion. Diagnosing that from
        outside took ten minutes; a timeout would have printed one line.

        Note that `wait_for_server()` returning True is NOT proof the server
        is alive - stale discovery outlives the process. The result timeout
        is the check that actually holds.

        On a result timeout the goal is cancelled rather than abandoned: if
        the server is merely slow the arm is still moving, and walking away
        from a live goal is how you get an unattended trajectory.
        """
        server_timeout = float(self.get_parameter("action_server_timeout_s").value)
        result_timeout = float(self.get_parameter("action_result_timeout_s").value)

        if not client.wait_for_server(timeout_sec=server_timeout):
            self.get_logger().error(
                f"{name} action server did not appear within {server_timeout:.0f}s - "
                "it is not running, or it died and left its name in the graph. "
                "Check `ros2 action info <action>` reports Action servers: 1."
            )
            return None

        send_future = client.send_goal_async(goal_msg)
        self._spin_until_done(send_future, timeout_sec=server_timeout)
        if not send_future.done():
            self.get_logger().error(
                f"{name} did not acknowledge the goal within {server_timeout:.0f}s - "
                "the server is advertised but not responding (most likely it "
                "crashed while its action names are still discoverable)."
            )
            return None

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"{name} goal rejected")
            return None

        result_future = goal_handle.get_result_async()
        self._spin_until_done(result_future, timeout_sec=result_timeout)
        if not result_future.done():
            self.get_logger().error(
                f"{name} returned no result within {result_timeout:.0f}s - "
                "cancelling the goal. If the robot is in User Stop, release it "
                "and check Desk before retrying."
            )
            try:
                cancel_future = goal_handle.cancel_goal_async()
                self._spin_until_done(cancel_future, timeout_sec=5.0)
            except Exception as exc:  # noqa: BLE001 - already failing; report and move on
                self.get_logger().warn(f"{name} cancel request failed: {exc}")
            return None

        return result_future.result()


def main(args=None):
    rclpy.init(args=args)
    node = PragmabotBridge()

    # MultiThreadedExecutor is REQUIRED, not a tuning choice: the skill
    # callbacks block on spin_until_future_complete while waiting for
    # MoveIt, which self-deadlocks on a single-threaded executor.
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # One-shot CLI mode: if grasp_file is set at launch, run a pick
    # immediately, then stay up serving the action. Preserves the manual
    # `ros2 run ... -p grasp_file:=...` workflow used for bring-up testing.
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

    node.get_logger().info("Serving /pragmabot/execute_skill - waiting for planner goals")
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
