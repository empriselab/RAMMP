"""Base class definition for high level skills."""

from __future__ import annotations

import abc
from pathlib import Path
from typing import Any, Dict

from pybullet_helpers.geometry import Pose

from rammp.interfaces.perception_interface import PerceptionInterface
from rammp.interfaces.rviz_interface import RVizInterface
from rammp.control.robot_controller.arm_client import ArmInterfaceClient
from rammp.control.robot_controller.command_interface import (
    CartesianCommand,
    CloseGripperCommand,
    JointCommand,
    KinovaCommand,
    OpenGripperCommand,
)

from rammp.simulation.simulator import FeedingDeploymentPyBulletSimulator
from rammp.simulation.state import FeedingDeploymentWorldState

# Define high-level skills
class BaseAction(abc.ABC):
    """Base class for high-level skill."""

    def __init__(
        self,
        sim: FeedingDeploymentPyBulletSimulator,
        robot_interface: ArmInterfaceClient,
        perception_interface: PerceptionInterface,
        rviz_interface: RVizInterface,
        no_waits: bool,
        log_dir: Path,
    ) -> None:
        self.sim = sim
        self.robot_interface = robot_interface
        self.perception_interface = perception_interface
        self.rviz_interface = rviz_interface
        self.no_waits = no_waits
        self.log_dir = log_dir

    @abc.abstractmethod
    def get_name(self) -> str:
        """Get a human-readable name for this skill."""

    @abc.abstractmethod
    def execute_action(self, params: dict[str, Any] = None) -> None:
        """Execute the action on the robot."""
        
    def move_to_joint_positions(self, joint_positions: list[float]) -> None:
        
        plan = None 
        if not self.no_waits:
            plan = self.sim.plan_to_joint_positions(joint_positions)
        if self.robot_interface is None:
            self.sim.visualize_plan(plan)
        else:
            self.execute_robot_command(JointCommand(pos=joint_positions), plan)
            
    def move_to_ee_pose(self, pose: Pose) -> None:

        plan = None
        if not self.no_waits:
            plan = self.sim.plan_to_ee_pose(pose)
        if self.robot_interface is None:
            self.sim.visualize_plan(plan)
        else:
            self.execute_robot_command(CartesianCommand(pos=pose.position, quat=pose.orientation), plan)
    
    def grasp_tool(self, tool: str) -> None:
        self.sim.grasp_object(tool)
        if self.robot_interface is not None:
            self.execute_robot_command(OpenGripperCommand(), tool_update=tool)

    def ungrasp_tool(self, tool: str) -> None:
        self.sim.ungrasp_object()
        if self.robot_interface is not None:
            self.execute_robot_command(CloseGripperCommand(), tool_update=tool)

    def open_gripper(self) -> None:
        if self.robot_interface is None:
            self.sim.robot.open_fingers()
        else:
            self.execute_robot_command(OpenGripperCommand())
    
    def close_gripper(self) -> None:
        if self.robot_interface is None:
            self.sim.robot.close_fingers()
        else:
            self.execute_robot_command(CloseGripperCommand())

    def execute_robot_command(self, robot_command: KinovaCommand, plan_viz: list[FeedingDeploymentWorldState] = None, tool_update: str = None) -> None:
        """Execute the given commands on the robot."""
        if self.robot_interface is None:
            raise ValueError("Robot interface is not available to execute commands.")
        
        if not self.no_waits:
            if tool_update is not None:
                self.rviz_interface.tool_update(True, tool_update, Pose((0, 0, 0), (0, 0, 0, 1))) # pickup the drink
            if plan_viz is not None:
                self.rviz_interface.visualize_plan(plan_viz)
            input("Execute next command?")
        self.robot_interface.execute_command(robot_command)