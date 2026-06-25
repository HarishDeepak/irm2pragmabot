#!/usr/bin/env python3
import rospy
import actionlib
from pragmabot.msg import PandaPushAction, PandaPushResult

class PandaPushServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer(
            "/panda/push", PandaPushAction, execute_cb=self.execute_cb, auto_start=False
        )
        self.server.start()
        rospy.loginfo("PandaPushServer STUB ready")

    def execute_cb(self, goal):
        rospy.loginfo(f"[push stub] goal: {goal.target_object} -> {goal.goal_region}")
        result = PandaPushResult(success=False, message="not implemented yet")
        self.server.set_aborted(result)

if __name__ == "__main__":
    rospy.init_node("panda_push_server")
    PandaPushServer()
    rospy.spin()