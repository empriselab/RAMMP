from rammp.actions.base import BaseAction
import time

class GrabCupFromTableAction(BaseAction):
    """Pick up a tool (drink)."""

    def get_name(self) -> str:
        return "GrabCupFromTable"

    def execute_action(self, params = None) -> None:
        drink_poses = self.perception_interface.get_last_drink_pickup_poses()

        self.move_to_ee_pose(drink_poses['pre_grasp_pose'])
        time.sleep(0.1)
        self.move_to_ee_pose(drink_poses['inside_bottom_pose'])
        time.sleep(0.1)

        self.move_to_ee_pose(drink_poses['inside_top_pose'])
        time.sleep(0.1)

        self.grasp_tool("drink")
        time.sleep(0.1)

        self.move_to_ee_pose(drink_poses['post_grasp_pose'])
        time.sleep(0.1)

        self.move_to_joint_positions(self.sim.scene_description.home_pos)

