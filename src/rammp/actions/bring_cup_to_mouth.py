from typing import Any

import numpy as np
import time
import pickle
from scipy.spatial.transform import Rotation
from pybullet_helpers.geometry import Pose

from rammp.actions.base import BaseAction
from rammp.perception.gestures_perception.static_gesture_detectors import mouth_open

class BringCupToMouthAction(BaseAction):
    """Bring cup to mouth."""

    def get_name(self) -> str:
        return "BringCupToMouth"

    def execute_action(self, params = None) -> None:
        outside_mouth_distance = 0.10

        self.move_to_joint_positions(self.sim.scene_description.drink_transfer_waypoint_pos)
        self.move_to_joint_positions(self.sim.scene_description.drink_before_transfer_pos)
        
        if self.robot_interface is not None:
            mouth_open(self.perception_interface, termination_event=None, timeout=600) # 10 minutes

        # move to infront of mouth
        head_perception_data = self.perception_interface.run_head_perception()
        forque_target_base = head_perception_data["tool_tip_target_pose"]
        head_pose = head_perception_data["head_pose"]

        if self.log_dir is not None:
            file_name = "head_perception_data"
            id = 0
            while (self.log_dir / f"{file_name}_{id}.pkl").exists():
                id += 1
            with open(self.log_dir / f"{file_name}_{id}.pkl", "wb") as f:
                pickle.dump(head_perception_data, f)
        self.sim.set_head_pose(Pose(position=head_pose[:3], orientation=Rotation.from_euler('yxz', head_pose[3:], degrees=True).as_quat()))

        # set mouth pose to be facing away from the wheelchair
        forque_target_base[:3, :3] = Rotation.from_quat([0.523, -0.503, -0.469, 0.503]).as_matrix()
        
        servo_point_forque_target = np.identity(4)
        servo_point_forque_target[:3,3] = np.array([0, 0, -outside_mouth_distance]).reshape(1,3)
        infront_mouth_target = forque_target_base @ servo_point_forque_target

        # # mouth is assumed to be facing away from the wheelchair
        # infront_mouth_target[:3, :3] = Rotation.from_quat([0.478, -0.505, -0.515, 0.502]).as_matrix()
        wrist_to_tip = self.sim.scene_description.tool_frame_to_drink_tip
        tip_to_wrist = np.linalg.inv(wrist_to_tip.to_matrix())
        tool_frame_target = infront_mouth_target @ tip_to_wrist

        target_pose = Pose.from_matrix(tool_frame_target)

        self.move_to_ee_pose(target_pose)
