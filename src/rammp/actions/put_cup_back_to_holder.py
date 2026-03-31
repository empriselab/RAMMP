from rammp.actions.base import BaseAction

class PutCupBackToHolderAction(BaseAction):
    """Put a cup back to its holder."""

    def get_name(self) -> str:
        return "PutCupBackToHolder"
    
    def execute_action(self, params = None) -> None:
        
        self.move_to_joint_positions(self.sim.scene_description.home_pos)
        self.move_to_joint_positions(self.sim.scene_description.above_drink_handle_pos)
        self.move_to_ee_pose(self.sim.scene_description.inside_drink_handle_pose)
        self.ungrasp_tool("drink")
        self.move_to_ee_pose(self.sim.scene_description.below_drink_handle_pose)
        self.move_to_ee_pose(self.sim.scene_description.outside_drink_handle_pose)
        self.move_to_joint_positions(self.sim.scene_description.home_pos)

