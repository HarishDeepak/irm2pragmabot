"""Scene observation via synchronized ROS 2 camera topics.

Ported from ROS 1 (rospy + message_filters) to ROS 2 (rclpy). The public
surface is unchanged - pragmabot_node.py still calls
`get_scene_observation()` and unpacks the same 4-tuple - so none of the
14 upstream VLM/memory modules are affected.

Three real differences from the ROS 1 version, none cosmetic:

1. **A node handle is required.** rospy had a hidden global node; rclpy
   does not. The caller owns the node and passes it in, so this class
   never calls rclpy.init() and never spins on its own.

2. **Polling spins the executor.** The ROS 1 version slept in a
   `while self.latest_obs is None: time.sleep(0.02)` loop, relying on
   rospy's background callback thread. In rclpy, callbacks only fire
   while something spins. Sleeping without spinning here would deadlock
   forever - the subscription would never deliver. We spin_once instead.

3. **QoS must be explicitly BEST_EFFORT.** ROS 2 sensor publishers
   (including the ZED wrapper and `ros2 bag play`) default to a
   BEST_EFFORT sensor QoS profile. A subscriber left on the default
   RELIABLE profile is QoS-incompatible and silently receives *nothing* -
   no error, no warning, just an observation that never arrives. This is
   the single most common "my ROS 2 node sees no images" cause.

Also handles raw vs compressed color: our bag records
`/zed/zed_node/rgb/color/rect/image` as a plain sensor_msgs/Image, while
upstream's ANYmal camera published CompressedImage. Selected by the
`color_image_compressed` config flag rather than hardcoded.
"""

import logging
from typing import Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from omegaconf import DictConfig
from PIL import Image
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CameraInfo as CameraInfoMsg
from sensor_msgs.msg import CompressedImage as CompressedImageMsg
from sensor_msgs.msg import Image as ImageMsg

from pragmabot.geometry import CameraIntrinsics

logger = logging.getLogger(__name__)

# Matches rclpy's rmw_qos_profile_sensor_data: BEST_EFFORT + KEEP_LAST(5).
# See module docstring point 3 - getting this wrong yields silence, not an error.
SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)

# Upper bound on how long get_scene_observation() waits for a synchronized
# triplet before giving up. Generous: `ros2 bag play` may not have been
# started yet when the planner asks for its first frame.
OBSERVATION_TIMEOUT_S = 30.0


class SceneObserver:
    """Capture synchronized color/depth images and camera intrinsics from ROS 2 topics."""

    def __init__(self, config: DictConfig, node: Node) -> None:
        """Initialize subscribers for synchronized camera topics.

        Args:
            config: Configuration object containing ``color_image``,
                ``depth_image``, ``camera_info``, and optionally
                ``color_image_compressed`` (defaults to False - our ZED bag
                publishes raw Image, not CompressedImage).
            node: The live rclpy node that owns these subscriptions. Required -
                rclpy has no implicit global node the way rospy did.
        """
        self.node = node
        self.color_image_topic = config.color_image
        self.depth_image_topic = config.depth_image
        self.camera_info_topic = config.camera_info
        # getattr, not config.get: OmegaConf DictConfig supports attribute
        # access, and this keeps an older config without the key working.
        self.color_is_compressed = bool(getattr(config, "color_image_compressed", False))

        self.latest_obs = None
        self.request_pending = False

        color_type = CompressedImageMsg if self.color_is_compressed else ImageMsg
        logger.info(
            "Establishing continuous synchronized connections "
            "(color=%s as %s, depth=%s, info=%s)",
            self.color_image_topic,
            color_type.__name__,
            self.depth_image_topic,
            self.camera_info_topic,
        )

        self.color_sub = Subscriber(node, color_type, self.color_image_topic, qos_profile=SENSOR_QOS)
        self.depth_sub = Subscriber(node, ImageMsg, self.depth_image_topic, qos_profile=SENSOR_QOS)
        self.info_sub = Subscriber(node, CameraInfoMsg, self.camera_info_topic, qos_profile=SENSOR_QOS)

        self.time_sync = ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub, self.info_sub], queue_size=10, slop=1.0
        )
        self.time_sync.registerCallback(self._sync_callback)

        self.bridge = CvBridge()

    def get_scene_observation(self) -> Tuple[Image.Image, Image.Image, CameraIntrinsics, object]:
        """Fetch a fresh synchronized color, depth, and info observation.

        Blocks (spinning the node) until the synchronizer delivers a triplet
        or OBSERVATION_TIMEOUT_S elapses.

        Raises:
            TimeoutError: if no synchronized triplet arrives in time. Raised
                rather than returning None so the caller fails loudly instead
                of passing None into a VLM call - the usual causes are the
                bag not playing, a topic name mismatch, or a QoS mismatch.
        """

        # Clear old data and open the gate for the callback
        self.latest_obs = None
        self.request_pending = True

        # Poll until the sync callback delivers a fresh frame. spin_once is
        # what actually lets the subscription fire (see module docstring
        # point 2) - a bare sleep here would block forever.
        deadline = self.node.get_clock().now().nanoseconds + int(OBSERVATION_TIMEOUT_S * 1e9)
        while self.latest_obs is None:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if self.node.get_clock().now().nanoseconds > deadline:
                self.request_pending = False
                raise TimeoutError(
                    f"No synchronized camera observation after {OBSERVATION_TIMEOUT_S}s on "
                    f"color={self.color_image_topic}, depth={self.depth_image_topic}, "
                    f"info={self.camera_info_topic}. Check, in this order: "
                    "(1) is the ZED wrapper running or `ros2 bag play` active? "
                    "(2) ROS_DOMAIN_ID=7 exported in THIS shell? "
                    "(3) do the topic names match `ros2 topic list` exactly? "
                    "(4) is color_image_compressed set to match the publisher's type?"
                )

        color_msg, depth_msg, info_msg = self.latest_obs
        observation_time = depth_msg.header.stamp  # Use depth timestamp as the observation time

        # Build intrinsics from the camera info message. ROS 2 renamed the
        # intrinsic matrix field from `K` (ROS 1) to `k`.
        intrinsics = CameraIntrinsics(
            width=info_msg.width,
            height=info_msg.height,
            fx=info_msg.k[0],
            fy=info_msg.k[4],
            ppx=info_msg.k[2],
            ppy=info_msg.k[5],
        )

        # Process color image
        if self.color_is_compressed:
            color_bytes = np.frombuffer(color_msg.data, np.uint8)
            color_bgr = cv2.imdecode(color_bytes, cv2.IMREAD_COLOR)
        else:
            # ZED publishes rect/image as BGRA8; imgmsg_to_cv2 with an
            # explicit bgr8 target drops the alpha channel for us.
            color_bgr = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
        color_rgb = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2RGB)
        color_image = Image.fromarray(color_rgb)

        # Process depth image
        depth_array = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        depth_image = Image.fromarray(depth_array)
        return color_image, depth_image, intrinsics, observation_time

    def _sync_callback(self, color_msg, depth_msg: ImageMsg, info_msg: CameraInfoMsg) -> None:
        """Only update the synchronized triplet if actively requested."""
        if self.request_pending:
            self.latest_obs = (color_msg, depth_msg, info_msg)
            self.request_pending = False  # Instantly close the gate so we only grab one frame
