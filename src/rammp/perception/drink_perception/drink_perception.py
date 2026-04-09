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

class DrinkPerception():
    def __init__(self):
        pass

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

    def run_perception(self, rgb_image, camera_info, depth_image, base_to_camera_transform):

        # -----------------------------
        # Color mask
        # -----------------------------
        mask = self.detect_handle_color(rgb_image)

        # save mask for debugging (overlay on RGB)
        vis = rgb_image.copy()
        vis[mask > 0] = (0, 255, 0)
        cv2.imwrite("color_mask.png", vis)


        mask = self.clean_mask(mask)

        # -----------------------------
        # Extract ALL 3D points from mask
        # -----------------------------
        points_3d = []
        pixels = []

        ys, xs = np.where(mask > 0)
        for u, v in zip(xs, ys):
            ok, p = self.pixel2World(
                camera_info, u, v, depth_image)
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

        cv2.imwrite("handle_mask.png", vis)
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

        # cam to tag homogeneous transform
        camera_to_tag = np.zeros((4, 4))
        camera_to_tag[:3, :3] = R_mat
        camera_to_tag[:3, 3] = center_3d
        camera_to_tag[3, 3] = 1 

        # base to tag homogeneous transform and update tf
        base_to_tag = np.dot(base_to_camera_transform, camera_to_tag)

        return self.matrix_to_pose(base_to_tag)

    def detect_handle_color(self, bgr_image):
        hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        # lower = np.array([60, 50, 50])
        # upper = np.array([95, 180, 200])
        # Wider but still green-focused
        # lower = np.array([50, 30, 30])
        # upper = np.array([105, 255, 255])

        # lower = np.array([80, 120, 120])
        # upper = np.array([100, 255, 255])
        
        lower = np.array([70, 70, 70])
        upper = np.array([110, 255, 255])

        return cv2.inRange(hsv, lower, upper)

    def clean_mask(self, mask):
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

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
