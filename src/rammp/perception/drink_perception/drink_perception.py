# Description: This script is used to detect ArUco markers and estimate their pose in the camera frame.

# python imports
import os, sys
import cv2
import numpy as np
import time
import math
from scipy.spatial.transform import Rotation
from sklearn.cluster import DBSCAN   # <-- ADDED
import open3d as o3d

# ros imports
import rospy
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import tf2_ros
from geometry_msgs.msg import Point, Pose
from visualization_msgs.msg import MarkerArray, Marker

from rammp.control.robot_controller.arm_client import ArmInterfaceClient
from rammp.control.robot_controller.command_interface import CartesianCommand, JointCommand, CloseGripperCommand, OpenGripperCommand
from geometry_msgs.msg import TransformStamped
from collections import deque

from geometry_msgs.msg import Pose as pose_msg


class TFInterface:
    def __init__(self):
        self.tfBuffer = tf2_ros.Buffer()  # Using default cache time of 10 secs
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.broadcaster = tf2_ros.TransformBroadcaster()
        time.sleep(1.0)

    def updateTF(self, source_frame, target_frame, pose):

        t = TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = source_frame
        t.child_frame_id = target_frame

        t.transform.translation.x = pose[0][3]
        t.transform.translation.y = pose[1][3]
        t.transform.translation.z = pose[2][3]

        R = Rotation.from_matrix(pose[:3, :3]).as_quat()
        t.transform.rotation.x = R[0]
        t.transform.rotation.y = R[1]
        t.transform.rotation.z = R[2]
        t.transform.rotation.w = R[3]

        self.broadcaster.sendTransform(t)

    def get_frame_to_frame_transform(self, camera_info_data, frame_A = "base_link", target_frame = "camera_color_optical_frame"):
        stamp = camera_info_data.header.stamp
        try:
            transform = self.tfBuffer.lookup_transform(
                frame_A,
                target_frame,
                rospy.Time(secs=stamp.secs, nsecs=stamp.nsecs),
            )
            return transform
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            # print("Exexption finding transform between base_link and", target_frame)
            return None

    def make_homogeneous_transform(self, transform):
        A_to_B = np.zeros((4, 4))
        A_to_B[:3, :3] = Rotation.from_quat(
            [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            ]
        ).as_matrix()
        A_to_B[:3, 3] = np.array(
            [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ]
        ).reshape(1, 3)
        A_to_B[3, 3] = 1

        return A_to_B

    def pose_to_matrix(self, pose):
        position = pose[0]
        orientation = pose[1]
        pose_matrix = np.zeros((4, 4))
        pose_matrix[:3, 3] = position
        pose_matrix[:3, :3] = Rotation.from_quat(orientation).as_matrix()
        pose_matrix[3, 3] = 1
        return pose_matrix
    
    def matrix_to_pose(self, mat):
        position = mat[:3, 3]
        orientation = Rotation.from_matrix(mat[:3, :3]).as_quat()
        return (position, orientation)


class DrinkPerception(TFInterface):
    def __init__(self, num_perception_samples=25):

        self.turned_on = False
        self.num_perception_samples = num_perception_samples
        self.bridge = CvBridge()

        self.color_image_sub = message_filters.Subscriber(
            '/camera/color/image_raw', Image)
        self.camera_info_sub = message_filters.Subscriber(
            '/camera/color/camera_info', CameraInfo)
        self.depth_image_sub = message_filters.Subscriber(
            '/camera/aligned_depth_to_color/image_raw', Image)
        
        self.handle_points_pub = rospy.Publisher("/handle_points", Marker, queue_size=1)
        self.handle_center_pub = rospy.Publisher("/handle_center", Marker, queue_size=1)

        # to simulate an aruco being detected
        self.aruco_pose_publisher_0 = rospy.Publisher("/aruco_pose_0", Pose, queue_size=10)

        ts = message_filters.TimeSynchronizer(
            [self.color_image_sub,
             self.camera_info_sub,
             self.depth_image_sub], 1)
        ts.registerCallback(self.rgbdCallback)

        super().__init__()

    def turn_on(self):
        self.turned_on = True

    def turn_off(self):
        self.turned_on = False

    def rgbdCallback(self, rgb_image_msg, camera_info_msg, depth_image_msg):

        # if hasattr(self, "saved") and self.saved:
            # return

        if not self.turned_on:
            return

        try:
            rgb_image = self.bridge.imgmsg_to_cv2(
                rgb_image_msg, "bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(
                depth_image_msg, "32FC1")
        except CvBridgeError as e:
            print(e)
            return

        # print("Got images")
        # cv2.imwrite("rgb.png", rgb_image)
        # depth_mm = (depth_image * 1000.0).astype("uint16")
        # cv2.imwrite("depth.png", depth_mm)

        # -----------------------------
        # Color mask
        # -----------------------------
        mask = self.detect_handle_color(rgb_image)
        mask = self.clean_mask(mask)

        # -----------------------------
        # Extract ALL 3D points from mask
        # -----------------------------
        points_3d = []
        pixels = []

        ys, xs = np.where(mask > 0)
        for u, v in zip(xs, ys):
            ok, p = self.pixel2World(
                camera_info_msg, u, v, depth_image)
            if ok:
                points_3d.append(p)
                pixels.append((u, v))

        if len(points_3d) == 0:
            # rospy.logwarn("No valid 3D points from mask.")
            return

        points_3d = np.array(points_3d)
        pixels = np.array(pixels)

        # print("Found all pixels")

        # -----------------------------
        # DBSCAN clustering (7 cm)
        # -----------------------------
        clustering = DBSCAN(
            eps=0.07,
            min_samples=50
        ).fit(points_3d)

        # print("Ran DBSCAN")

        labels = clustering.labels_
        valid = labels >= 0

        if not np.any(valid):
            # rospy.logwarn("DBSCAN found no clusters.")
            return

        unique, counts = np.unique(labels[valid], return_counts=True)
        main_label = unique[np.argmax(counts)]

        cluster_pixels = pixels[labels == main_label]
        cluster_points_3d = points_3d[labels == main_label]

        # print("Found cluster")

        # -----------------------------
        # Project cluster back to image
        # -----------------------------
        cluster_mask = np.zeros(mask.shape, dtype=np.uint8)
        for u, v in cluster_pixels:
            cluster_mask[v, u] = 255

        cluster_mask = cv2.dilate(
            cluster_mask, np.ones((3, 3), np.uint8), iterations=1)

        vis = rgb_image.copy()
        vis[cluster_mask > 0] = (0, 0, 255)

        # cv2.imwrite("handle_mask.png", vis)
        # rospy.loginfo(
        #     f"Saved handle_mask.png with {cluster_pixels.shape[0]} pixels")
        
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(cluster_points_3d)

        plane_model, inliers = pcd.segment_plane(
            distance_threshold=0.003,
            ransac_n=3,
            num_iterations=500
        )

        plane_cloud = pcd.select_by_index(inliers)
        non_plane_cloud = pcd.select_by_index(inliers, invert=True)

        pts3d_planar = np.asarray(plane_cloud.points)
        pts3d_handle = np.asarray(non_plane_cloud.points)
        self.visualizeHandle(pts3d_planar)

        # Plane normal
        a, b, c, d = plane_model
        n = np.array([a, b, c])
        n = n / np.linalg.norm(n)

        # Build plane basis (camera is level)
        up = np.array([0, 0, 1])
        u = np.cross(up, n)
        u = u / np.linalg.norm(u)
        v = np.cross(n, u)

        # Project 3D points to 2D plane coordinates
        P0 = pts3d_planar.mean(axis=0)
        P = pts3d_planar - P0
        x = P @ u
        y = P @ v
        P2 = np.stack([x, y], axis=1).astype(np.float32)

        # Fit minimum-area rectangle
        rect = cv2.minAreaRect(P2)
        (center_2d, _, _) = rect

        # Back-project center to 3D
        center_3d = P0 + center_2d[0] * u + center_2d[1] * v

        # Get 4 rectangle corners in 2D (plane coordinates)
        box_2d = cv2.boxPoints(rect)  # shape (4,2)

        # Back-project corners to 3D
        corners_3d = []
        for x2d, y2d in box_2d:
            p3d = P0 + x2d * u + y2d * v
            corners_3d.append(p3d)

        corners_3d = np.array(corners_3d)

        points_to_show = np.vstack([center_3d.reshape(1, 3), corners_3d])  # (5,3)
        self.visualizeHandleCorners(points_to_show)

        # Sort corners by image-space Y (top vs bottom)
        # Smaller Y = higher in image (top)
        ys = corners_3d[:, 1]
        top_idx = np.argsort(ys)[:2]
        bottom_idx = np.argsort(ys)[2:]

        top_pts = corners_3d[top_idx]
        bottom_pts = corners_3d[bottom_idx]

        # Sort left/right within top and bottom using X
        top_left, top_right = top_pts[np.argsort(top_pts[:, 0])]
        bottom_left, bottom_right = bottom_pts[np.argsort(bottom_pts[:, 0])]

        # X-axis: bottom → top
        x_axis = ((top_left + top_right) / 2.0) - ((bottom_left + bottom_right) / 2.0)
        x_axis = x_axis / np.linalg.norm(x_axis)

        # Y-axis: right → left
        y_axis = ((top_left + bottom_left) / 2.0) - ((top_right + bottom_right) / 2.0)
        y_axis = y_axis / np.linalg.norm(y_axis)

        # Z-axis: towards camera (right-handed)
        z_axis = np.cross(x_axis, y_axis)
        z_axis = z_axis / np.linalg.norm(z_axis)

        # Re-orthogonalize Y to avoid drift
        y_axis = np.cross(z_axis, x_axis)
        y_axis = y_axis / np.linalg.norm(y_axis)

        # Rotation matrix (columns are axes)
        R_mat = np.column_stack((x_axis, y_axis, z_axis))

        transform = self.get_frame_to_frame_transform(camera_info_msg)

        if transform is not None:   
            base_to_camera = self.make_homogeneous_transform(transform)

            # cam to tag homogeneous transform
            camera_to_tag = np.zeros((4, 4))
            camera_to_tag[:3, :3] = R_mat
            camera_to_tag[:3, 3] = center_3d
            camera_to_tag[3, 3] = 1 

            # base to tag homogeneous transform and update tf
            base_to_tag = np.dot(base_to_camera, camera_to_tag)
            self.updateTF("base_link", "AR_tag_0", base_to_tag)
            self.update_aruco_pose(base_to_tag)

    def update_aruco_pose(self, aruco_pose_mat):

        position, orientation = self.matrix_to_pose(aruco_pose_mat)        
        pose_msg = Pose()
        pose_msg.position.x = position[0]
        pose_msg.position.y = position[1]
        pose_msg.position.z = position[2]
        pose_msg.orientation.x = orientation[0]
        pose_msg.orientation.y = orientation[1]
        pose_msg.orientation.z = orientation[2]
        pose_msg.orientation.w = orientation[3]
        self.aruco_pose_publisher_0.publish(pose_msg)

    def detect_handle_color(self, bgr_image):
        hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        lower = np.array([60, 50, 50])
        upper = np.array([95, 180, 200])
        return cv2.inRange(hsv, lower, upper)

    def clean_mask(self, mask):
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask
    
    def visualizeHandle(self, points):

        marker = Marker()
        marker.header.frame_id = "camera_color_optical_frame"  # IMPORTANT: match your camera TF
        marker.header.stamp = rospy.Time.now()

        marker.ns = "handle_points"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD

        # Point size (meters)
        marker.scale.x = 0.005
        marker.scale.y = 0.005

        # Color (red)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Lifetime (0 = forever)
        marker.lifetime = rospy.Duration(0)

        # Fill points
        for x, y, z in points:
            p = Point()
            p.x = x
            p.y = y
            p.z = z
            marker.points.append(p)

        self.handle_points_pub.publish(marker)

    def visualizeHandleCorners(self, points_3d):
        """
        points_3d: Nx3 numpy array.
        First point is the center (green, larger).
        Remaining points are corners (blue, smaller).
        """

        # --- Center marker (green sphere) ---
        corner_marker = Marker()
        corner_marker.header.frame_id = "camera_color_optical_frame"
        corner_marker.header.stamp = rospy.Time.now()

        corner_marker.ns = "handle_corners"
        corner_marker.id = 1
        corner_marker.type = Marker.SPHERE_LIST
        corner_marker.action = Marker.ADD

        corner_marker.scale.x = 0.015
        corner_marker.scale.y = 0.015
        corner_marker.scale.z = 0.015

        corner_marker.color.r = 0.0
        corner_marker.color.g = 0.0
        corner_marker.color.b = 1.0
        corner_marker.color.a = 1.0

        corner_marker.lifetime = rospy.Duration(0)

        for x, y, z in points_3d:
            p = Point()
            p.x = float(x)
            p.y = float(y)
            p.z = float(z)
            corner_marker.points.append(p)

        self.handle_center_pub.publish(corner_marker)

    def pixel2World(self, camera_info, image_x, image_y, depth_image):

        # print("Image pixels: ", image_x, image_y)
        # print("Depth shape: ", depth_image.shape)

        if image_y >= depth_image.shape[0] or image_x >= depth_image.shape[1]:
            return False, None

        depth = depth_image[image_y, image_x]
        depth = depth / 1000 # convert from mm to m
        # print("Depth: ", depth)

        if math.isnan(depth) or depth < 0.05 or depth > 1.0:
            return False, None

        fx = camera_info.K[0]
        fy = camera_info.K[4]
        cx = camera_info.K[2]
        cy = camera_info.K[5]

        world_x = (depth / fx) * (image_x - cx)
        world_y = (depth / fy) * (image_y - cy)
        world_z = depth

        # print("3D Pixel: ", world_x, world_y, world_z)

        return True, (world_x, world_y, world_z)


if __name__ == '__main__':
    rospy.init_node('DrinkPerception')
    DrinkPerception()
    rospy.spin()
