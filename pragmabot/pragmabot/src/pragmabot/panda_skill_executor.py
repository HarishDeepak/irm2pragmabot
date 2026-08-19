import rospy
import actionlib
from pragmabot.msg import (
    PandaPickAction,  PandaPickGoal,
    PandaPlaceAction, PandaPlaceGoal,
    PandaPushAction,  PandaPushGoal,
)

ACTION_TIMEOUT = 120.0  # seconds


class PandaSkillExecutor:

    def __init__(self):
        self.pick_client  = actionlib.SimpleActionClient("/panda/pick",  PandaPickAction)
        self.place_client = actionlib.SimpleActionClient("/panda/place", PandaPlaceAction)
        self.push_client  = actionlib.SimpleActionClient("/panda/push",  PandaPushAction)

        rospy.loginfo("Waiting for panda action servers...")
        self.pick_client.wait_for_server(timeout=rospy.Duration(30.0))
        self.place_client.wait_for_server(timeout=rospy.Duration(30.0))
        self.push_client.wait_for_server(timeout=rospy.Duration(30.0))
        rospy.loginfo("All panda action servers connected")

    def execute(self, action) -> dict:
        """
        action: NextBestAction Pydantic object from VLMTaskPlanner
        Returns: {"success": bool, "pre_image": None, "post_image": None, "message": str}
        """
        skill = action.skill.lower()

        if skill == "pick":
            return self._execute_pick(action)
        elif skill == "place":
            return self._execute_place(action)
        elif skill == "push":
            return self._execute_push(action)
        elif skill == "done":
            return {"success": True, "pre_image": None, "post_image": None,
                    "message": "task marked done by planner"}
        else:
            return {"success": False, "pre_image": None, "post_image": None,
                    "message": "unknown skill: %s" % skill}

    def _execute_pick(self, action) -> dict:
        goal = PandaPickGoal(
            target_object=getattr(action, "target_object", "") or "",
            use_annotation=False,
        )
        self.pick_client.send_goal(goal, feedback_cb=self._feedback_cb)
        done = self.pick_client.wait_for_result(rospy.Duration(ACTION_TIMEOUT))
        if not done:
            self.pick_client.cancel_goal()
            return {"success": False, "pre_image": None, "post_image": None,
                    "message": "pick timed out"}
        result = self.pick_client.get_result()
        return {"success": result.success, "pre_image": None, "post_image": None,
                "message": result.message}

    def _execute_place(self, action) -> dict:
        from geometry_msgs.msg import PoseStamped
        goal = PandaPlaceGoal(
            target_location=getattr(action, "target_location", "") or "",
            place_pose=PoseStamped(),
        )
        self.place_client.send_goal(goal, feedback_cb=self._feedback_cb)
        done = self.place_client.wait_for_result(rospy.Duration(ACTION_TIMEOUT))
        if not done:
            self.place_client.cancel_goal()
            return {"success": False, "pre_image": None, "post_image": None,
                    "message": "place timed out"}
        result = self.place_client.get_result()
        return {"success": result.success, "pre_image": None, "post_image": None,
                "message": result.message}

    def _execute_push(self, action) -> dict:
        goal = PandaPushGoal(
            target_object=getattr(action, "target_object", "") or "",
            goal_region=getattr(action, "target_location", "") or "",
        )
        self.push_client.send_goal(goal, feedback_cb=self._feedback_cb)
        done = self.push_client.wait_for_result(rospy.Duration(ACTION_TIMEOUT))
        if not done:
            self.push_client.cancel_goal()
            return {"success": False, "pre_image": None, "post_image": None,
                    "message": "push timed out"}
        result = self.push_client.get_result()
        return {"success": result.success, "pre_image": None, "post_image": None,
                "message": result.message}

    @staticmethod
    def _feedback_cb(fb):
        rospy.loginfo("[skill] %s", fb.status)
