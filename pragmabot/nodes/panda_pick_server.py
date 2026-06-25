#!/usr/bin/env python3
import rospy
import actionlib
from pragmabot.msg import PandaPickAction, PandaPickResult

class PandaPickServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer(
            "/panda/pick", PandaPickAction, execute_cb=self.execute_cb, auto_start=False
        )
        self.server.start()
        rospy.loginfo("PandaPickServer STUB ready")

    def execute_cb(self, goal):
        rospy.loginfo(f"[pick stub] goal: {goal.target_object}")
        result = PandaPickResult(success=False, message="not implemented yet")
        self.server.set_aborted(result)

if __name__ == "__main__":
    rospy.init_node("panda_pick_server")
    PandaPickServer()
    rospy.spin()