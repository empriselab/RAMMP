from rammp.actions.base import BaseAction


class LocateCup(BaseAction):
    """Move to a cup-observation pose before cup perception."""

    def get_name(self) -> str:
        return "LocateCup"

    def execute_action(self, params=None) -> None:
        self.move_to_joint_positions(self.sim.scene_description.home_pos)
        self.close_gripper()
        self.move_to_joint_positions(self.sim.scene_description.drink_gaze_pos)
