from rammp.actions.base import BaseAction

class GrabCupFromTableAction(BaseAction):
    """Pick up a tool (drink)."""

    def get_name(self) -> str:
        return "GrabCupFromTable"
    
    def execute_action(self, params = None) -> None:
        drink_poses = self.perception_interface.get_last_drink_pickup_poses()

        self.move_to_ee_pose(drink_poses['pre_grasp_pose'])
        self.move_to_ee_pose(drink_poses['inside_bottom_pose'])
        self.move_to_ee_pose(drink_poses['inside_top_pose'])
        self.grasp_tool("drink")
        self.move_to_ee_pose(drink_poses['post_grasp_pose'])

        # self.perception_interface.record_drink_pickup_joint_pos()

        self.move_to_joint_positions(self.sim.scene_description.drink_staging_pos)
        self.move_to_joint_positions(self.sim.scene_description.home_pos)

