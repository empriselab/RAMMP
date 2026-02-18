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

class PickToolHLA(HighLevelAction):
    """Pick up a tool (drink)."""

    def get_name(self) -> str:
        return "PickTool"

    def get_operator(self) -> LiftedOperator:
        tool = Variable("?tool", tool_type)
        return LiftedOperator(
            self.get_name(),
            parameters=[tool],
            preconditions={LiftedAtom(GripperFree, [])},
            add_effects={Holding([tool])},
            delete_effects={LiftedAtom(GripperFree, [])},
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
        return f"pick_{tool.name}.yaml"
        
    def pick_drink(self, speed: str) -> None:
        assert self.sim.held_object_name is None

        if self.robot_interface is not None:
            self.robot_interface.set_speed(speed)

        self.move_to_joint_positions(self.sim.scene_description.retract_pos)
        self.close_gripper()
        self.move_to_joint_positions(self.sim.scene_description.drink_gaze_pos)

        drink_poses = self.perception_interface.perceive_drink_pickup_poses()

        # self.move_to_joint_positions(self.sim.scene_description.drink_staging_pos)
        self.move_to_ee_pose(drink_poses['pre_grasp_pose'])
        self.move_to_ee_pose(drink_poses['inside_bottom_pose'])
        self.move_to_ee_pose(drink_poses['inside_top_pose'])
        self.grasp_tool("drink")
        self.move_to_ee_pose(drink_poses['post_grasp_pose'])

        self.perception_interface.record_drink_pickup_joint_pos()

        self.move_to_joint_positions(self.sim.scene_description.home_pos)
        
        if self.robot_interface is not None:
            self.robot_interface.start_maintain_home_orientation()
        # self.move_to_joint_positions(self.sim.scene_description.drink_before_transfer_pos)
