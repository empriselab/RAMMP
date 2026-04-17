#!/usr/bin/env python3
"""
Overlays the detected cup handle pose onto the wrist camera image and publishes
the result as a sensor_msgs/Image for viewing in RViz.

Subscribes:
  /camera/wrist/color/image_raw     (sensor_msgs/Image)
  /camera/wrist/color/camera_info   (sensor_msgs/CameraInfo)
  /arm/drink/cup_handle             (cornell_feeding_interfaces/CupInfo)

Publishes:
  /arm/drink/cup_handle_viz         (sensor_msgs/Image)
"""

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import message_filters

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from cornell_feeding_interfaces.msg import CupInfo
import tf2_ros
from scipy.spatial.transform import Rotation


class CupHandleViz(Node):
    def __init__(self):
        super().__init__("cup_handle_viz")

        self.bridge = CvBridge()
        self.latest_cup_info = None

        image_qos = qos_profile_sensor_data
        info_qos = QoSProfile(
            depth=10,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        self.image_sub = message_filters.Subscriber(
            self, Image, "/camera/wrist/color/image_raw", qos_profile=image_qos
        )
        self.info_sub = message_filters.Subscriber(
            self, CameraInfo, "/camera/wrist/color/camera_info", qos_profile=info_qos
        )
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.info_sub], queue_size=10, slop=0.05
        )
        self.ts.registerCallback(self.image_callback)

        self.cup_sub = self.create_subscription(
            CupInfo, "/arm/drink/cup_handle", self.cup_callback, 10
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.viz_pub = self.create_publisher(Image, "/arm/drink/cup_handle_viz", 10)

        self.get_logger().info("cup_handle_viz node started")

    def cup_callback(self, msg: CupInfo):
        self.latest_cup_info = msg

    def image_callback(self, image_msg: Image, camera_info_msg: CameraInfo):
        try:
            img = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"CvBridge error: {e}")
            return

        cup_info = self.latest_cup_info

        if cup_info is not None and cup_info.success:
            # --- Project 3D cup pose into image ---
            fx = camera_info_msg.k[0]
            fy = camera_info_msg.k[4]
            cx = camera_info_msg.k[2]
            cy = camera_info_msg.k[5]

            pos_base = np.array(cup_info.pose[:3])
            quat = cup_info.pose[3:]  # qx qy qz qw

            # Transform from base_link to camera frame
            try:
                tf = self.tf_buffer.lookup_transform(
                    "wrist_color_optical_frame",
                    "base_link",
                    rclpy.time.Time(),
                )
                t = tf.transform.translation
                r = tf.transform.rotation
                T = np.eye(4)
                T[:3, :3] = Rotation.from_quat([r.x, r.y, r.z, r.w]).as_matrix()
                T[:3, 3] = [t.x, t.y, t.z]
            except Exception as e:
                self.get_logger().warn(f"TF lookup failed: {e}", throttle_duration_sec=2.0)
                self._publish(img, image_msg)
                return

            pos_cam = (T[:3, :3] @ pos_base) + T[:3, 3]

            if pos_cam[2] <= 0:
                self._publish(img, image_msg)
                return

            # Project cup center
            u = int(fx * pos_cam[0] / pos_cam[2] + cx)
            v = int(fy * pos_cam[1] / pos_cam[2] + cy)

            # Draw axes (0.05 m length) in camera frame
            rot_base = Rotation.from_quat(quat).as_matrix()
            axis_len = 0.05
            axes_base = rot_base @ (axis_len * np.eye(3))  # columns: x, y, z axes

            colors = [(0, 0, 255), (0, 255, 0), (255, 0, 0)]  # x=red, y=green, z=blue
            labels = ["X", "Y", "Z"]
            for i in range(3):
                tip_base = pos_base + axes_base[:, i]
                tip_cam = (T[:3, :3] @ tip_base) + T[:3, 3]
                if tip_cam[2] <= 0:
                    continue
                u2 = int(fx * tip_cam[0] / tip_cam[2] + cx)
                v2 = int(fy * tip_cam[1] / tip_cam[2] + cy)
                cv2.arrowedLine(img, (u, v), (u2, v2), colors[i], 2, tipLength=0.3)
                cv2.putText(img, labels[i], (u2 + 4, v2 + 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, colors[i], 1)

            # Draw center dot
            cv2.circle(img, (u, v), 6, (0, 255, 255), -1)

            # Bounding box (pixel coords from CupInfo)
            bb = cup_info.bounding_box
            if any(bb):
                cv2.rectangle(img, (bb[0], bb[1]), (bb[2], bb[3]), (0, 255, 255), 2)

            # Pose text
            cv2.putText(
                img,
                f"pos: ({pos_base[0]:.3f}, {pos_base[1]:.3f}, {pos_base[2]:.3f})",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                2,
            )
        else:
            cv2.putText(
                img, "No cup detected", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2
            )

        self._publish(img, image_msg)

    def _publish(self, img, image_msg):
        out = self.bridge.cv2_to_imgmsg(img, "bgr8")
        out.header = image_msg.header
        self.viz_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CupHandleViz()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
