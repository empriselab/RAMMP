from rammp.actions.base import BaseAction
import time

class PickupAndOrderAction(BaseAction):
    """Pick up drink from wheelchair holder and place an order."""

    def get_name(self) -> str:
        return "PickupAndOrder"

    def execute_action(self, params = None) -> None:

        # Pick drink from wheelchair holder
        print("Moving to outside drink handle position")
        self.move_to_joint_positions(self.sim.scene_description.outside_drink_handle_pos)
        time.sleep(0.1)
        print("Closing gripper")
        self.close_gripper()
        time.sleep(0.1)

        print("Moving to below drink handle pose")
        self.move_to_ee_pose(self.sim.scene_description.below_drink_handle_pose)
        time.sleep(0.1)

        print("Moving to inside drink handle pose")
        self.move_to_ee_pose(self.sim.scene_description.inside_drink_handle_pose)
        time.sleep(0.1)

        print("Grasping drink")
        self.grasp_tool("drink")
        time.sleep(0.1)

        print("Moving to above drink handle pose")
        self.move_to_ee_pose(self.sim.scene_description.above_drink_handle_pose)
        time.sleep(0.1)

        print("Moving to home position")
        self.move_to_joint_positions(self.sim.scene_description.home_pos)
        time.sleep(0.1)

        print("Moving to drink handover pose")
        self.move_to_joint_positions(self.sim.scene_description.drink_handover_pos)

