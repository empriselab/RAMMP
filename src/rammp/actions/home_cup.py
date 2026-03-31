from rammp.actions.base import BaseAction

class HomeCupAction(BaseAction):
    """Move a cup to the home position after drinking."""

    def get_name(self) -> str:
        return "HomeCup"
    
    def execute_action(self, params = None) -> None:
        
        self.move_to_joint_positions(self.sim.scene_description.drink_before_transfer_pos)
        self.move_to_joint_positions(self.sim.scene_description.drink_transfer_waypoint_pos)
        self.move_to_joint_positions(self.sim.scene_description.home_pos)

