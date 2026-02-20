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
        del params  # not used right now
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