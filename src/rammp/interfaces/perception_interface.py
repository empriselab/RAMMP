"""An interface for perception (robot joints, human head poses, etc.)."""

import numpy as np
from pybullet_helpers.geometry import Pose
from scipy.spatial.transform import Rotation as R
import pickle

from rclpy.node import Node

from rammp.interfaces.realsense_interface import RealSenseInterface

from rammp.perception.drink_perception.drink_perception import DrinkPerception
from rammp.perception.head_perception.deca_perception import HeadPerception

class PerceptionInterface:
    """An interface for perception (robot joints, human head poses, etc.)."""

    def __init__(self, node: Node, simulation: bool = False, log_dir: str | None = None) -> None:
        self.node = node
        self.simulation = simulation
        self.log_dir = log_dir

        if not self.simulation:
            self.realsense_interface = RealSenseInterface(self.node)

            self._head_perception = HeadPerception()
            # Warm start head perception
            self._head_perception.set_tool("drink")
            for _ in range(10):
                self.run_head_perception()

            self._drink_perception = DrinkPerception()
        else:
            self.realsense_interface = None
            self._head_perception = None
            self._drink_perception = None

        self.last_drink_poses = None

    def run_head_perception(self, ):
        # print("Running Head Perception")
        if self.simulation:
            try:
                # read from logged data
                with open(self.log_dir / f'head_perception_data_drink.pkl', 'rb') as f:
                    head_perception_data = pickle.load(f)
            except FileNotFoundError:
                raise FileNotFoundError("No transfer logged data found for tool: ", self.tool)
            return head_perception_data
        
        camera_data = self.realsense_interface.get_camera_data()
        base_to_camera = self.realsense_interface.get_base_to_camera_transform()

        head_perception_data = self._head_perception.run_deca(
            camera_data["rgb_image"],
            camera_data["camera_info"],
            camera_data["depth_image"],
            base_to_camera,
            debug_print=False,
            visualize=False,
            filter_noisy_readings=False,
        )

        if head_perception_data is not None:
            head_perception_data = {
                "head_pose": head_perception_data["head_pose"],
                "face_keypoints": head_perception_data["landmarks2d"],
                "tool_tip_target_pose": head_perception_data["tool_tip_target_pose"],
                "camera_color_data": camera_data["rgb_image"],
            }
            if self.log_dir is not None:
                with open(self.log_dir / f'head_perception_data_drink.pkl', 'wb') as f:
                    pickle.dump(head_perception_data, f)
            return head_perception_data
        else:
            return None

    def perceive_drink_pickup_poses(self):

        def get_drink_transform():
            tf = np.zeros((4, 4))
            tf[:3, :3] = R.from_euler("xyz", [0, 0, np.pi / 2]).as_matrix()
            tf[:3, 3] = np.array([0.0, 0.0, 0.0]) 
            tf[3, 3] = 1
            return tf

        def get_pre_grasp_transform():
            tf = np.zeros((4, 4))
            tf[:3, :3] = R.from_euler("xyz", [np.pi, 0, np.pi / 2]).as_matrix()
            tf[:3, 3] = np.array([0.02, 0.01, 0.15]) 
            tf[3, 3] = 1
            return tf

        def get_inside_bottom_transform():
            tf = get_pre_grasp_transform()
            tf[2, 3] = 0.017
            return tf

        def get_inside_top_transform():
            tf = get_inside_bottom_transform()
            tf[0, 3] = 0.043
            return tf
        
        def get_post_grasp_pose():
            tf = get_inside_top_transform()
            tf[0, 3] = 0.233
            return tf
        
        def get_place_inside_bottom_transform():
            tf = get_inside_bottom_transform()
            # tf[1, 3] = 0.0
            return tf

        def get_place_pre_grasp_transform():
            tf = get_pre_grasp_transform()
            # tf[2, 3] = 0.25
            # tf[1, 3] = 0.0
            return tf
                
        if self.simulation:
            # load them from a pickle file
            with open(self.log_dir / 'drink_pickup_pos.pkl', 'rb') as f:
                drink_pickup_pos = pickle.load(f)
            drink_poses = drink_pickup_pos["last_drink_poses"]

        else:
            for _ in range(5): # 5 times so that it stabilizes
                camera_data = self.realsense_interface.get_camera_data()
                base_to_camera =-self.realsense_interface.get_base_to_camera_transform()
                self.aruco_pose = self._drink_perception.run_perception(camera_data["rgb_image"], camera_data["camera_info"], camera_data["depth_image"], base_to_camera)

            drink_poses  = {}
            drink_poses['drink_pose'] = self.get_aruco_relative_pose(get_drink_transform(), "drink")
            drink_poses['pre_grasp_pose'] = self.get_aruco_relative_pose(get_pre_grasp_transform(), "drink")
            drink_poses['inside_bottom_pose'] = self.get_aruco_relative_pose(get_inside_bottom_transform(), "drink")
            drink_poses['inside_top_pose'] = self.get_aruco_relative_pose(get_inside_top_transform(), "drink")
            drink_poses['post_grasp_pose'] = self.get_aruco_relative_pose(get_post_grasp_pose(), "drink")
            drink_poses['place_inside_bottom_pose'] = self.get_aruco_relative_pose(get_place_inside_bottom_transform(), "drink")
            drink_poses['place_pre_grasp_pose'] = self.get_aruco_relative_pose(get_place_pre_grasp_transform(), "drink")

        self.last_drink_poses = drink_poses

        return drink_poses
    
    def record_drink_pickup_joint_pos(self, joint_positions):
        if self.simulation:
            return
        
        self.drink_pickup_joint_pos = joint_positions[:7]
        # save them in a pickle file
        drink_pickup_pos = {
            "last_drink_poses": self.last_drink_poses,
            "drink_pickup_joint_pos": self.drink_pickup_joint_pos
        }
        with open(self.log_dir / 'drink_pickup_pos.pkl', 'wb') as f:
            pickle.dump(drink_pickup_pos, f)
        print("Drink pickup poses recorded")

    def get_aruco_relative_pose(self, transform, override_angles = ""):
        aruco_pos_mat = self.pose_to_matrix(self.aruco_pose)
        goal_frame = np.dot(aruco_pos_mat, transform)
        goal_pose = self.matrix_to_pose(goal_frame)

        # If true, use 2 hardcoded angle values.
        if override_angles == "drink":
            rot = R.from_quat(goal_pose[1])
            roll = np.pi / 2
            pitch = 0
            _, _, yaw = rot.as_euler("xyz")
            new_rot = R.from_euler("xyz", [roll, pitch, yaw])
            goal_pose = Pose(goal_pose[0], new_rot.as_quat())
        
        return goal_pose

    def pose_to_matrix(self, pose):
        position = pose[0]
        orientation = pose[1]
        pose_matrix = np.zeros((4, 4))
        pose_matrix[:3, 3] = position
        pose_matrix[:3, :3] = R.from_quat(orientation).as_matrix()
        pose_matrix[3, 3] = 1
        return pose_matrix
    
    def matrix_to_pose(self, mat):
        position = mat[:3, 3]
        orientation = R.from_matrix(mat[:3, :3]).as_quat()
        return Pose(position, orientation) 