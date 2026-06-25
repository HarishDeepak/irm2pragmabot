import rospy

class PandaSkillExecutor:
    def __init__(self):
        rospy.loginfo("PandaSkillExecutor stub — Phase 6 will complete this")

    def execute(self, action) -> dict:
        """Stub: returns fake success so VLM loop can be tested offline."""
        rospy.loginfo(f"[STUB] execute called: skill={action.skill}, target={action.target_object}")
        return {
            "success": True,
            "pre_image": None,
            "post_image": None,
            "message": "stub response",
        }