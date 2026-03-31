from rammp.actions.base import BaseAction
import time

class PickupAndOrderAction(BaseAction):
    """Pick up drink from wheelchair holder and place an order."""

    def get_name(self) -> str:
        return "PickupAndOrder"
    
    def execute_action(self, params = None) -> None:
        
        # Pick drink from wheelchair holder
        self.move_to_joint_positions(self.sim.scene_description.home_pos)
        self.move_to_joint_positions(self.sim.scene_description.outside_drink_handle_pos)
        self.close_gripper()
        self.move_to_ee_pose(self.sim.scene_description.below_drink_handle_pose)
        self.move_to_ee_pose(self.sim.scene_description.inside_drink_handle_pose)
        self.grasp_tool("drink")
        self.move_to_ee_pose(self.sim.scene_description.above_drink_handle_pose)
        self.move_to_joint_positions(self.sim.scene_description.home_pos)

        # Place an order
        self.move_to_joint_positions(self.sim.scene_description.home_pos)
        self.move_to_ee_pose(self.sim.scene_description.drink_handover_pose)
        input("Press Enter to ungrasp drink")
        # time.sleep(5.0) # Wait for 5 seconds to simulate waiting for the human to take the drink
        self.ungrasp_tool("drink")
        time.sleep(3.0) # Wait for 3 seconds after ungrasping
        self.move_to_joint_positions(self.sim.scene_description.home_pos)
