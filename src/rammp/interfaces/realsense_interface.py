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
import rospy
import tf2_ros
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point, TransformStamped, WrenchStamped
from scipy.spatial.transform import Rotation
from sensor_msgs import point_cloud2
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Bool, Float64, Float64MultiArray, String
from visualization_msgs.msg import Marker, MarkerArray

class RealSenseInterface:
    def __init__(self, record_goal_pose=False):

        # Top Camera Data
        self.camera_lock = Lock()
        self.camera_header = None
        self.camera_color_data = None
        self.camera_info_data = None
        self.camera_depth_data = None

        self.bridge = CvBridge()

        self.tf_buffer_lock = Lock()
        self.tfBuffer = tf2_ros.Buffer()  # Using default cache time of 10 secs
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.broadcaster = tf2_ros.TransformBroadcaster()

        queue_size = 1000
        self.color_image_sub = message_filters.Subscriber(
            "/camera/color/image_raw",
            Image,
            queue_size=queue_size,
            buff_size=65536 * queue_size,
        )
        self.camera_info_sub = message_filters.Subscriber(
            "/camera/color/camera_info",
            CameraInfo,
            queue_size=queue_size,
            buff_size=65536 * queue_size,
        )
        self.depth_image_sub = message_filters.Subscriber(
            "/camera/aligned_depth_to_color/image_raw",
            Image,
            queue_size=queue_size,
            buff_size=65536 * queue_size,
        )
        ts_top = message_filters.TimeSynchronizer(
            [self.color_image_sub, self.camera_info_sub, self.depth_image_sub],
            queue_size=queue_size,
        )
        ts_top.registerCallback(self.rgbdCallback)
        ts_top.enable_reset = True

        time.sleep(2.0) # sleep until all subscribers are registered

    def rgbdCallback(self, rgb_image_msg, camera_info_msg, depth_image_msg):
        # print("RGB Callback")

        try:
            # Convert your ROS Image message to OpenCV2
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_image_msg, "bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(depth_image_msg, "32FC1")
        except CvBridgeError as e:
            print(e)

        with self.camera_lock:
            self.camera_color_data = rgb_image
            self.camera_info_data = camera_info_msg
            self.camera_depth_data = depth_image
            self.camera_header = rgb_image_msg.header

    def get_camera_data(self):
        with self.camera_lock:
            return {
                "rgb_image": deepcopy(self.camera_color_data),
                "camera_info": deepcopy(self.camera_info_data),
                "depth_image": deepcopy(self.camera_depth_data),
                "header": deepcopy(self.camera_header),
            }

    def get_base_to_camera_transform(self):
        with self.camera_lock:
            camera_info_data = deepcopy(self.camera_info_data)
            if camera_info_data is None:
                return None
        target_frame = "camera_color_optical_frame"
        stamp = camera_info_data.header.stamp
        try:
            with self.tf_buffer_lock:
                transform = self.tfBuffer.lookup_transform(
                    "base_link",
                    target_frame,
                    rospy.Time(secs=stamp.secs, nsecs=stamp.nsecs),
                )
                T = np.zeros((4,4))
                T[:3,:3] = Rotation.from_quat([transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w]).as_matrix()
                T[:3,3] = np.array([transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z]).reshape(1,3)
                T[3,3] = 1
                return T
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            # print("Exception finding transform between base_link and", target_frame)
            return None