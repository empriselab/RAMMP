#!/usr/bin/env python3
"""
Interactive HSV color mask tuner for cup handle detection.

Subscribes to /camera/wrist/color/image_raw and shows a live OpenCV window
with trackbars to adjust HSV bounds. Prints the final values when you press Q.

Usage:
    python3 color_mask_tuner.py
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


# Current values from drink_perception.py
DEFAULTS = dict(h_low=70, s_low=70, v_low=70, h_high=110, s_high=255, v_high=255)


class ColorMaskTuner(Node):
    def __init__(self):
        super().__init__("color_mask_tuner")
        self.bridge = CvBridge()
        self.latest_frame = None

        self.sub = self.create_subscription(
            Image,
            "/camera/wrist/color/image_raw",
            self.image_callback,
            qos_profile_sensor_data,
        )

        cv2.namedWindow("Tuner", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Mask", cv2.WINDOW_NORMAL)

        for name, val in DEFAULTS.items():
            limit = 180 if name.startswith("h") else 255
            cv2.createTrackbar(name, "Tuner", val, limit, lambda x: None)

        self.timer = self.create_timer(0.033, self.update)
        self.get_logger().info("Waiting for camera frames...")

    def image_callback(self, msg):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"CvBridge error: {e}")

    def update(self):
        if self.latest_frame is None:
            return

        frame = self.latest_frame.copy()

        h_low  = cv2.getTrackbarPos("h_low",  "Tuner")
        s_low  = cv2.getTrackbarPos("s_low",  "Tuner")
        v_low  = cv2.getTrackbarPos("v_low",  "Tuner")
        h_high = cv2.getTrackbarPos("h_high", "Tuner")
        s_high = cv2.getTrackbarPos("s_high", "Tuner")
        v_high = cv2.getTrackbarPos("v_high", "Tuner")

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower = np.array([h_low, s_low, v_low])
        upper = np.array([h_high, s_high, v_high])
        mask = cv2.inRange(hsv, lower, upper)

        kernel = np.ones((5, 5), np.uint8)
        mask_clean = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask_clean = cv2.morphologyEx(mask_clean, cv2.MORPH_CLOSE, kernel)

        overlay = frame.copy()
        overlay[mask_clean > 0] = (0, 255, 0)
        blended = cv2.addWeighted(frame, 0.5, overlay, 0.5, 0)

        pixel_count = int(np.sum(mask_clean > 0))
        cv2.putText(blended, f"Masked pixels: {pixel_count}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv2.imshow("Tuner", blended)
        cv2.imshow("Mask", mask_clean)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            print("\n--- Copy these into drink_perception.py ---")
            print(f"lower = np.array([{h_low}, {s_low}, {v_low}])")
            print(f"upper = np.array([{h_high}, {s_high}, {v_high}])")
            cv2.destroyAllWindows()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ColorMaskTuner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
