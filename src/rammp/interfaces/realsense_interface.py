import math
import struct
import time
from copy import deepcopy
from threading import Lock
from types import SimpleNamespace

import cv2
import argparse
import message_filters
import numpy as np
import rclpy
import tf2_ros

from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point, TransformStamped, WrenchStamped
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Bool, Float64, Float64MultiArray, String
from visualization_msgs.msg import Marker, MarkerArray


class RealSenseInterface:
    def __init__(self, node: Node):
        self.node = node

        # Top Camera Data
        self.camera_lock = Lock()
        self.camera_header = None
        self.camera_color_msg = None
        self.camera_info_data = None
        self.camera_depth_msg = None

        self.bridge = CvBridge()

        self.tf_buffer_lock = Lock()
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

        self.broadcaster = tf2_ros.TransformBroadcaster(self.node)

        queue_size = 1000
        from rclpy.qos import qos_profile_sensor_data, QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
        image_qos = qos_profile_sensor_data
        info_qos = QoSProfile(depth=10, history=QoSHistoryPolicy.KEEP_LAST, reliability=QoSReliabilityPolicy.RELIABLE)

        self.color_image_sub = message_filters.Subscriber(
            self.node,
            Image,
            "/camera/wrist/color/image_raw",
            qos_profile=image_qos,
        )
        self.camera_info_sub = message_filters.Subscriber(
            self.node,
            CameraInfo,
            "/camera/wrist/color/camera_info",
            qos_profile=info_qos,
        )
        self.depth_image_sub = message_filters.Subscriber(
            self.node,
            Image,
            "/camera/wrist/aligned_depth_to_color/image_raw",
            qos_profile=image_qos,
        )

        self.ts_top = message_filters.TimeSynchronizer(
            [self.color_image_sub, self.camera_info_sub, self.depth_image_sub],
            queue_size=queue_size,
        )
        self.ts_top.registerCallback(self.rgbd_callback)

        time.sleep(2.0)  # sleep until all subscribers are registered

    def rgbd_callback(self, rgb_image_msg, camera_info_msg, depth_image_msg):
        # Only keep the latest messages here; converting every frame at the
        # camera rate cost ~30% of a Jetson core while no task needed images.
        with self.camera_lock:
            self.camera_color_msg = rgb_image_msg
            self.camera_info_data = camera_info_msg
            self.camera_depth_msg = depth_image_msg
            self.camera_header = rgb_image_msg.header

    def get_camera_data(self):
        with self.camera_lock:
            rgb_image_msg = self.camera_color_msg
            depth_image_msg = self.camera_depth_msg
            camera_info = deepcopy(self.camera_info_data)
            header = deepcopy(self.camera_header)
        if rgb_image_msg is None:
            return {
                "rgb_image": None,
                "camera_info": camera_info,
                "depth_image": None,
                "header": header,
            }
        try:
            # Convert on demand; the messages are immutable once received, so
            # the converted arrays are fresh copies for the caller.
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_image_msg, "bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(depth_image_msg, "32FC1")
        except CvBridgeError as e:
            self.node.get_logger().error(f"CvBridge error: {e}")
            return {
                "rgb_image": None,
                "camera_info": camera_info,
                "depth_image": None,
                "header": header,
            }
        return {
            "rgb_image": rgb_image,
            "camera_info": camera_info,
            "depth_image": depth_image,
            "header": header,
        }

    def get_base_to_camera_transform(self):
        with self.camera_lock:
            camera_info_data = deepcopy(self.camera_info_data)
            if camera_info_data is None:
                return None

        target_frame = "wrist_color_optical_frame"
        stamp = Time.from_msg(camera_info_data.header.stamp)

        try:
            with self.tf_buffer_lock:
                transform = self.tf_buffer.lookup_transform(
                    "base_link",
                    target_frame,
                    stamp,
                )

            T = np.zeros((4, 4))
            T[:3, :3] = Rotation.from_quat(
                [
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w,
                ]
            ).as_matrix()
            T[:3, 3] = np.array(
                [
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z,
                ]
            )
            T[3, 3] = 1.0
            return T

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
            tf2_ros.TransformException,
        ) as e:
            self.node.get_logger().warning(
                f"TF lookup base_link->{target_frame} failed: {type(e).__name__}: {e}",
                throttle_duration_sec=1.0,
            )
            return None
        
def main(args=None):
    rclpy.init(args=args)
    node = Node("realsense_interface_node")

    interface = RealSenseInterface(node)
    camera_data = interface.get_camera_data()
    base_to_camera = interface.get_base_to_camera_transform()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()