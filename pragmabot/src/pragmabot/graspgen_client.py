import numpy as np
import rospy
import zmq
import msgpack
import msgpack_numpy
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_matrix

msgpack_numpy.patch()

GRASPGEN_HOST = "172.17.0.1"  # Docker bridge — host machine from inside container
GRASPGEN_PORT = 5556


class GraspGenClient:
    """ZMQ client for the GraspGen server running on the host machine."""

    def __init__(self, host=GRASPGEN_HOST, port=GRASPGEN_PORT, timeout_ms=60000):
        self._addr = f"tcp://{host}:{port}"
        self._timeout_ms = timeout_ms
        self._ctx = zmq.Context()
        self._socket = None

    def _connect(self):
        sock = self._ctx.socket(zmq.REQ)
        sock.setsockopt(zmq.RCVTIMEO, self._timeout_ms)
        sock.setsockopt(zmq.SNDTIMEO, self._timeout_ms)
        sock.setsockopt(zmq.LINGER, 0)
        sock.connect(self._addr)
        return sock

    def _request(self, payload):
        if self._socket is None:
            self._socket = self._connect()
        self._socket.send(msgpack.packb(payload))
        return msgpack.unpackb(self._socket.recv(), raw=False)

    def generate(
        self,
        point_cloud,           # Nx3 float32, panda_link0 frame, object-centred
        num_grasps=200,
        topk=20,
        grasp_threshold=-1.0,
    ):
        """
        Send point cloud to GraspGen server, return list of
        {"pose": PoseStamped, "score": float} sorted descending by score.
        Frame = panda_link0.
        """
        pc = np.asarray(point_cloud, dtype=np.float32)
        payload = {
            "action": "infer",
            "point_cloud": pc,
            "num_grasps": num_grasps,
            "topk_num_grasps": topk,
            "grasp_threshold": grasp_threshold,
            "min_grasps": 5,
            "max_tries": 6,
            "remove_outliers": True,
        }
        try:
            resp = self._request(payload)
        except zmq.error.Again:
            rospy.logerr("GraspGen server timeout — is it running on %s?", self._addr)
            return []
        except Exception as e:
            rospy.logerr("GraspGen request failed: %s", e)
            return []

        grasps = np.asarray(resp["grasps"], dtype=np.float32)        # (M, 4, 4)
        scores = np.asarray(resp["confidences"], dtype=np.float32)   # (M,)

        results = []
        for T, score in zip(grasps, scores):
            ps = PoseStamped()
            ps.header.frame_id = "panda_link0"
            ps.header.stamp = rospy.Time.now()
            ps.pose.position.x = float(T[0, 3])
            ps.pose.position.y = float(T[1, 3])
            ps.pose.position.z = float(T[2, 3])
            q = quaternion_from_matrix(T)   # [x, y, z, w]
            ps.pose.orientation.x = q[0]
            ps.pose.orientation.y = q[1]
            ps.pose.orientation.z = q[2]
            ps.pose.orientation.w = q[3]
            results.append({"pose": ps, "score": float(score)})

        results.sort(key=lambda x: x["score"], reverse=True)
        return results

    def close(self):
        if self._socket is not None:
            self._socket.close()
            self._socket = None
        self._ctx.term()
