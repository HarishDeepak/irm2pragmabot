class GraspGenClient:
    def __init__(self, url="http://localhost:8080/grasp"):
        self.url = url
    def generate(self, point_cloud, gripper="franka_panda", num_grasps=20):
        raise NotImplementedError("GraspGen not yet installed — Phase 4")