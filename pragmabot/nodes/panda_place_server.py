#!/usr/bin/env python3
import rospy
import actionlib
from pragmabot.msg import PandaPlaceAction, PandaPlaceResult

class PandaPlaceServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer(
            "/panda/place", PandaPlaceAction, execute_cb=self.execute_cb, auto_start=False
        )
        self.server.start()
        rospy.loginfo("PandaPlaceServer STUB ready")

    def execute_cb(self, goal):
        rospy.loginfo(f"[place stub] goal: {goal.target_location}")
        result = PandaPlaceResult(success=False, message="not implemented yet")
        self.server.set_aborted(result)

if __name__ == "__main__":
    rospy.init_node("panda_place_server")
    PandaPlaceServer()
    rospy.spin()