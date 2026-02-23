from typing import Any

import time

from relational_structs import (
    GroundAtom,
    GroundOperator,
    LiftedAtom,
    LiftedOperator,
    Object,
    Predicate,
    Type,
    Variable,
)
from rammp.actions.base import (
    HighLevelAction,
    tool_type,
    GripperFree,
    Holding,
)

class StowToolHLA(HighLevelAction):
    """Stow a tool (drink)."""

    def get_name(self) -> str:
        return "StowTool"

    def get_operator(self) -> LiftedOperator:
        tool = Variable("?tool", tool_type)
        return LiftedOperator(
            self.get_name(),
            parameters=[tool],
            preconditions={Holding([tool])},
            add_effects={LiftedAtom(GripperFree, [])},
            delete_effects={Holding([tool])},
        )
    
    def get_behavior_tree_filename(
        self,
        objects: tuple[Object, ...],
        params: dict[str, Any],
    ) -> str:
        # del params  # not used right now
        # assert "drink_location" in params
        self.drink_location = params.get("drink_location", "table") # "wheelchair_handle" or "table"
        assert len(objects) == 1
        tool = objects[0]
        assert tool.name in ["drink"]
        return f"stow_{tool.name}.yaml"

    def stow_drink(self, speed: str) -> None:
        assert self.sim.held_object_name == "drink"

        if self.robot_interface is not None:
            self.robot_interface.set_speed(speed)

        if self.robot_interface is not None:
            self.robot_interface.stop_maintain_home_orientation()

        if self.drink_location == "table":
            self.stow_drink_on_table()
        elif self.drink_location == "wheelchair_handle":
            self.stow_drink_in_wheelchair()
        elif self.drink_location == "handover":
            self.handover_drink()
        else:
            raise ValueError(f"Invalid drink location: {self.drink_location}")

    def handover_drink(self) -> None:
        
        self.move_to_joint_positions(self.sim.scene_description.home_pos)
        self.move_to_ee_pose(self.sim.scene_description.drink_handover_pose)
        # input("Press Enter to ungrasp drink")
        time.sleep(5.0) # Wait for 5 seconds to simulate waiting for the human to take the drink
        self.ungrasp_tool("drink")
        self.move_to_joint_positions(self.sim.scene_description.home_pos)

    def stow_drink_in_wheelchair(self) -> None:
        self.move_to_joint_positions(self.sim.scene_description.home_pos)
        self.move_to_joint_positions(self.sim.scene_description.above_drink_handle_pos)
        self.move_to_ee_pose(self.sim.scene_description.inside_drink_handle_pose)
        self.ungrasp_tool("drink")
        self.move_to_ee_pose(self.sim.scene_description.below_drink_handle_pose)
        self.move_to_ee_pose(self.sim.scene_description.outside_drink_handle_pose)
        self.move_to_joint_positions(self.sim.scene_description.home_pos)

    def stow_drink_on_table(self) -> None:

        last_drink_poses, last_drink_pickup_joint_pos = self.perception_interface.get_last_drink_pickup_configs()
        x_movement, y_movement = 0, 0

        # self.move_to_joint_positions(self.sim.scene_description.drink_before_transfer_pos)
        if abs(x_movement) < 0.01 and abs(y_movement) < 0.01:
            self.move_to_joint_positions(last_drink_pickup_joint_pos)
        self.move_to_ee_pose(last_drink_poses['inside_top_pose'])
        self.ungrasp_tool("drink")
        self.move_to_ee_pose(last_drink_poses['place_inside_bottom_pose'])
        self.move_to_ee_pose(last_drink_poses['place_pre_grasp_pose'])
        self.move_to_joint_positions(self.sim.scene_description.drink_staging_pos)
        self.move_to_joint_positions(self.sim.scene_description.retract_pos)