"""High-level actions that we can simulate and execute."""

from __future__ import annotations

import abc
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable
from gymnasium.spaces import Space, Box, Text
from tomsutils.spaces import EnumSpace
import functools
from operator import attrgetter
import itertools
import string
import traceback
import re
import inspect

import yaml

import numpy as np
import time

from pybullet_helpers.geometry import Pose, multiply_poses
from pybullet_helpers.spaces import PoseSpace
from pybullet_helpers.joint import JointPositions
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
from pybullet_helpers.gui import visualize_pose

from tomsutils.llm import LargeLanguageModel

from rammp.interfaces.perception_interface import PerceptionInterface
from rammp.interfaces.web_interface import WebInterface
from rammp.interfaces.rviz_interface import RVizInterface
from rammp.control.robot_controller.arm_client import ArmInterfaceClient
from rammp.control.robot_controller.command_interface import (
    CartesianCommand,
    CloseGripperCommand,
    JointCommand,
    KinovaCommand,
    OpenGripperCommand,
)

from rammp.simulation.planning import (
    _get_plan_to_execute_grasp,
    _get_plan_to_execute_ungrasp,
)
from rammp.simulation.simulator import FeedingDeploymentPyBulletSimulator
from rammp.simulation.state import FeedingDeploymentWorldState

# Define some predicates that can be used for sequencing the high-level actions.
tool_type = Type("tool")  # utensil, drink, or wiping tool
GripperFree = Predicate("GripperFree", [])  # not holding any tool
Holding = Predicate("Holding", [tool_type])  # holding tool
ToolTransferDone = Predicate("ToolTransferDone", [tool_type])  # wiped, drank, or ate
EmulateTransferDone = Predicate("EmulateTransferDone", [])  # emulated transfer
ToolPrepared = Predicate("ToolPrepared", [tool_type])  # e.g., bite acquired
PlateInView = Predicate("PlateInView", [])  # of the hand camera
ResetPos = Predicate("ResetPos", [])  # robot in reset position
IsUtensil = Predicate("IsUtensil", [tool_type])

# Define high-level actions.
class HighLevelAction(abc.ABC):
    """Base class for high-level action."""

    def __init__(
        self,
        sim: FeedingDeploymentPyBulletSimulator,
        robot_interface: ArmInterfaceClient,
        perception_interface: PerceptionInterface,
        rviz_interface: RVizInterface,
        web_interface: WebInterface,
        hla_hyperparams: dict[str, Any],
        no_waits: bool,
        log_dir: Path,
        behavior_tree_dir: Path,
        behavior_tree_execution_log: Path,
    ) -> None:
        self.sim = sim
        self.robot_interface = robot_interface
        self.perception_interface = perception_interface
        self.rviz_interface = rviz_interface
        self.web_interface = web_interface
        self.hla_hyperparams = hla_hyperparams
        self.no_waits = no_waits
        self.log_dir = log_dir
        self.behavior_tree_dir = behavior_tree_dir
        self.behavior_tree_execution_log = behavior_tree_execution_log
        # NOTE: assuming 7-dof and that first 7 entries are arm joints (not gripper).
        self.arm_joint_lower_limits = self.sim.robot.joint_lower_limits[:7]
        self.arm_joint_upper_limits = self.sim.robot.joint_upper_limits[:7]
        # Used for generating unique names for user-added nodes.
        self.new_node_counter = itertools.count()

    @abc.abstractmethod
    def get_name(self) -> str:
        """Get a human-readable name for this HLA."""

    @abc.abstractmethod
    def get_operator(self) -> LiftedOperator:
        """Create a planning operator for this HLA."""

    @abc.abstractmethod
    def get_behavior_tree_filename(
        self,
        objects: tuple[Object, ...],
        params: dict[str, Any],
    ) -> str:
        """Get the YAML filename (not path, just name) for the behavior tree."""

    def execute_action(
        self,
        objects: tuple[Object, ...],
        params: dict[str, Any],
    ) -> None:
        """Execute the action on the robot."""
        bt_filename = self.get_behavior_tree_filename(objects, params)
        bt_filepath = self.behavior_tree_dir / bt_filename
        assert bt_filepath.exists()
        bt = load_behavior_tree(bt_filepath, self)
        bt.tick()

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

    def reset_wrist(self) -> None:
        if self.wrist_interface is not None:
            time.sleep(1.0) # wait for the utensil to be connected
            print("Resetting wrist controller ...")
            self.wrist_interface.set_velocity_mode()
            self.wrist_interface.reset()

    def pause(self, duration: float) -> None:
        time.sleep(duration)

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


@dataclass(frozen=True)
class GroundHighLevelAction:
    """A high-level action with objects and parameters specified.

    For example, the parameters for bite acquisition might include the
    preferred food item. These parameters can be populated automatically
    or set by the user.
    """

    hla: HighLevelAction  # want this to be executed
    objects: tuple[Object, ...]  # for grounding the high-level action
    params: dict = field(default_factory=lambda: {})  # see docstring

    def __str__(self) -> str:
        obj_str = ", ".join([o.name for o in self.objects])
        return f"{self.hla.get_name()}({obj_str})"

    def get_operator(self) -> GroundOperator:
        """Get the operator for this ground HLA."""
        return self.hla.get_operator().ground(self.objects)

    def get_preconditions(self) -> set[GroundAtom]:
        """Get the preconditions for executing this command."""
        return self.get_operator().preconditions

    def execute_action(self) -> None:
        """Execute the command."""
        self.hla.execute_action(self.objects, self.params)

    def process_behavior_tree_node_addition(self, new_node_type: str, new_node_parameters: dict[str, Any],
                                            anchor_node_name: str, before_or_after: str) -> str:
        """Validate and add a new node into the behavior tree."""
        return self.hla.process_behavior_tree_node_addition(self.objects, self.params, new_node_type, new_node_parameters, anchor_node_name, before_or_after)

    def process_behavior_tree_parameter_update(self, node_name: str, parameter_name: str, new_parameter_value: Any) -> str:
        """Validate and update the behavior tree for this ground HLA."""
        return self.hla.process_behavior_tree_parameter_update(self.objects, self.params, node_name, parameter_name, new_parameter_value)


class ResetHLA(HighLevelAction):
    """Move the robot to retract position without any tool."""

    def get_name(self) -> str:
        return "Reset"

    def get_operator(self) -> LiftedOperator:
        return LiftedOperator(
            self.get_name(),
            parameters=[],
            preconditions={LiftedAtom(GripperFree, [])},
            add_effects={LiftedAtom(ResetPos, [])},
            delete_effects=set(),
        )
    
    def get_behavior_tree_filename(
        self,
        objects: tuple[Object, ...],
        params: dict[str, Any],
    ) -> str:
        # Behavior trees not used for this HLA
        raise NotImplementedError

    def execute_action(
        self,
        objects: tuple[Object, ...],
        params: dict[str, Any],
    ) -> None:
        assert len(objects) == 0
        assert self.sim.held_object_name is None

        self.move_to_joint_positions(self.sim.scene_description.retract_pos)

        # set FLAIR preferences to None
        if self.flair is not None:
            self.flair.clear_preference()


def pddl_plan_to_hla_plan(
    pddl_plan: list[GroundOperator], hlas: set[HighLevelAction]
) -> list[GroundHighLevelAction]:
    """Convert a PDDL plan into a sequence of HLA and objects."""
    hla_plan = []
    op_to_hla = {hla.get_operator(): hla for hla in hlas}
    for ground_operator in pddl_plan:
        hla = op_to_hla[ground_operator.parent]
        objects = tuple(ground_operator.parameters)
        ground_hla = GroundHighLevelAction(hla, objects)
        hla_plan.append(ground_hla)
    return hla_plan


@dataclass
class BehaviorTreeParameter:
    """A single parameter for a policy in a behavior tree.
    
    One policy may have multiple parameters.
    """
    
    name: str
    description: str
    space: Space
    is_user_editable: bool

    def __hash__(self):
        return hash(self.name)  # assume unique names
    
    def __eq__(self, other: Any):
        return isinstance(other, BehaviorTreeParameter) and other.name == self.name
    
    def get_yaml_dict(self) -> dict[str, Any]:
        """Get a YAML dict for this parameter."""
        space_dict = get_space_yaml_dict(self.space)
        return {
            "name": self.name,
            "description": self.description,
            "is_user_editable": self.is_user_editable,
            "space": space_dict
        }


class BehaviorTreeParameterizedPolicy(abc.ABC):
    """A parameterized policy for an action node in a behavior tree."""

    def __init__(self, name: str) -> None:
        self._name = name

    @abc.abstractmethod
    def get_parameters(self) -> list[BehaviorTreeParameter]:
        """Expose ordered parameters for this policy."""

    @abc.abstractmethod
    def get_function_name(self) -> str:
        """Get the name of the function for dumping to YAML."""

    def run(self, bindings: dict[BehaviorTreeParameter, Any]) -> None:
        """Run the policy given values for the parameters."""
        parameters = self.get_parameters()
        assert set(bindings) == set(parameters)
        ordered_parameter_values = []
        for parameter in parameters:
            value = bindings[parameter]
            assert parameter.space.contains(value), (
                f"Value {value} invalid for parameter {parameter}")
            ordered_parameter_values.append(value)
        
        print(f"Executing parameterized policy {self._name} with bindings:")
        for parameter, value in zip(parameters, ordered_parameter_values, strict=True):
            print(f"  {parameter.name} = {value}")

        self._execute(*ordered_parameter_values)

    @abc.abstractmethod
    def _execute(self, *args: Any) -> None:
        """Execute the policy given ordered values for the parameters."""


class FunctionalBehaviorTreeParameterizedPolicy(BehaviorTreeParameterizedPolicy):
    """A parameterized policy defined by a given function."""

    def __init__(self, name: str, parameters: list[BehaviorTreeParameter],
                 fn: Callable[[Any], None]) -> None:
        super().__init__(name)
        self._parameters = parameters
        self._fn = fn

    def get_parameters(self) -> list[BehaviorTreeParameter]:
        return list(self._parameters)

    def get_function_name(self) -> str:
        return self._fn.__name__
        
    def _execute(self, *args: Any) -> None:
        return self._fn(*args)
    

class BehaviorTreeNode(abc.ABC):
    """A node in a behavior tree."""

    def __init__(self, name: str, description: str, execution_log_path: str) -> None:
        self.name = name
        self._description = description
        self._execution_log_path = execution_log_path
        self.parent = None  # may be set by parent during tree construction

    @abc.abstractmethod
    def tick(self) -> bool:
        """Execute the node for one step and return a status."""

    @abc.abstractmethod
    def get_node(self, name: str) -> BehaviorTreeNode | None:
        """Get a node in this subtree with the given name."""

    @abc.abstractmethod
    def get_yaml_dict(self, hla: HighLevelAction) -> dict[str, Any]:
        """Get a dictionary to pass to yaml.dump()."""

    @abc.abstractmethod
    def walk(self) -> list[BehaviorTreeNode]:
        """Enumerate all nodes including this one."""

    def log_start(self) -> None:
        """Log the start of execution."""
        with open(self._execution_log_path, 'a') as f:
            f.write(f"Starting node: {self.name}\n")

    def log_end(self) -> None:
        """Log the end of execution."""
        with open(self._execution_log_path, 'a') as f:
            f.write(f"Finished executing node: {self.name}\n")


class ParameterizedActionBehaviorTreeNode(BehaviorTreeNode):
    """A node in a behavior tree that executes a parameterized action open-loop.
    
    For now, the status after execution is not checked.

    Parameters may or may not be user-editable.
    """
    def __init__(self, name: str, description: str, execution_log_path:str, policy: BehaviorTreeParameterizedPolicy,
                 bindings: dict[BehaviorTreeParameter, Any]) -> None:
        super().__init__(name, description, execution_log_path)
        self._policy = policy
        self._bindings = bindings

    def tick(self) -> bool:
        self.log_start()
        self._policy.run(self._bindings)
        self.log_end()
        return True  # assume this worked

    def get_node(self, name: str) -> BehaviorTreeNode | None:
        if name == self.name:
            return self
        return None
    
    def walk(self) -> list[BehaviorTreeNode]:
        return [self]

    def get_yaml_dict(self, hla: HighLevelAction) -> dict[str, Any]:
        fn_name = self._policy.get_function_name()
        assert hasattr(hla, fn_name)
        fn_str = f"!hla {fn_name}"
        parameter_dicts = []
        for parameter in self._policy.get_parameters():
            parameter_dict = parameter.get_yaml_dict().copy()
            parameter_dict["value"] = self._bindings[parameter]
            parameter_dicts.append(parameter_dict)
        return {
            "name": self.name,
            "description": self._description,
            "type": "Behavior",
            "parameters": parameter_dicts,
            "fn": fn_str,
        }

    def get_parameter(self, name: str) -> BehaviorTreeParameter | None:
        """Access a policy parameter by name."""
        for parameter in self._policy.get_parameters():
            if parameter.name == name:
                return parameter
        return None
    
    def set_parameter(self, parameter: BehaviorTreeParameter, value: Any) -> None:
        """Update the parameter binding."""
        assert parameter.space.contains(value)
        self._bindings[parameter] = value


class SequenceBehaviorTreeNode(BehaviorTreeNode):
    """A sequence node in a behavior tree.
    
    For now, the status after execution is not checked.
    """
    def __init__(self, name: str, description: str, execution_log_path: str, children: list[BehaviorTreeNode]) -> None:
        super().__init__(name, description, execution_log_path)
        self._children = children
        for child in children:
            assert child.parent is None
            child.parent = self

    def tick(self) -> bool:
        self.log_start()
        for child in self._children:
            child.tick()
        self.log_end()
        return True  # assume everything worked

    def get_node(self, name: str) -> BehaviorTreeNode | None:
        for child in self._children:
            if child.get_node(name) is not None:
                return child
        return None
    
    def walk(self) -> list[BehaviorTreeNode]:
        lst = [self]
        for child in self._children:
            lst.extend(child.walk())
        return lst

    def get_yaml_dict(self, hla: HighLevelAction) -> dict[str, Any]:
        child_dicts = [child.get_yaml_dict(hla) for child in self._children]
        return {
            "name": self.name,
            "description": self._description,
            "type": "Sequence",
            "children": child_dicts,
        }
    
    def add_child(self, node: BehaviorTreeNode, anchor_node: BehaviorTreeNode, before_or_after: str) -> None:
        """Add a new child node."""
        assert anchor_node in self._children
        anchor_index = self._children.index(anchor_node)
        if before_or_after == "before":
            new_child_index = anchor_index - 1
        elif before_or_after == "after":
            new_child_index = anchor_index + 1
        else:
            raise ValueError(f"Invalid {before_or_after=}")
        self._children.insert(new_child_index, node)
        node.parent = self


def _eval_expression(obj, loader, node):
    value = loader.construct_scalar(node)
    try:
        # Need to use attrgetter instead of getattr for nested attributes.
        return attrgetter(value)(obj)
    except Exception as e:
        raise ValueError(f"Error evaluating expression '{value}': {e}")


def load_behavior_tree(filepath: Path, hla: HighLevelAction) -> BehaviorTreeNode:

    with open(filepath, "r", encoding="utf-8") as f:
        yaml_text = f.read()

    class CustomLoader(yaml.SafeLoader):
        pass

    CustomLoader.add_constructor('!hla', functools.partial(_eval_expression, hla))
    CustomLoader.add_constructor('!scene_description', functools.partial(_eval_expression,
                                                                         hla.sim.scene_description))
    root_dict = yaml.load(yaml_text, Loader=CustomLoader)
    return _parse_node(root_dict, hla.behavior_tree_execution_log)


def save_behavior_tree(behavior_tree: BehaviorTreeNode, filepath: Path, hla: HighLevelAction) -> None:

    yaml_dict = behavior_tree.get_yaml_dict(hla)
    yaml_str = yaml.dump(yaml_dict, sort_keys=False)

    # Cannot find any better way to do this... the issue is that the YAML dumper
    # puts single quotes around the !hla expression, but then loading doesn't
    # work. I tried many things on both loading and dumping and then gave up.
    lines = []
    for line in yaml_str.splitlines():
        if "'!hla" in line:
            assert line.endswith("'")
            line = line[:-1]
            line = line.replace("'!hla", "!hla")
        lines.append(line)
    yaml_str = "\n".join(lines)

    with open(filepath, "w", encoding="utf-8") as f:
        f.write(yaml_str)


def _parse_node(node_dict: dict, execution_log_path: str) -> BehaviorTreeNode:
    """Recursively parse a node from the loaded YAML structure."""
    node_name = node_dict["name"]
    node_description = node_dict["description"]
    node_type = node_dict["type"]

    if node_type == "Sequence":
        children_dicts = node_dict.get("children", [])
        children_nodes = [_parse_node(child, execution_log_path) for child in children_dicts]
        return SequenceBehaviorTreeNode(node_name, node_description, execution_log_path, children_nodes)

    elif node_type == "Behavior":
        params_list = node_dict.get("parameters", [])
        parameters = []
        bindings = {}
        for p in params_list:
            p_name = p["name"]
            p_description = p["description"]
            p_is_user_editable = p["is_user_editable"]
            space_spec = p["space"]
            space_type = space_spec["type"]
            if space_type == "Box":
                space = Box(np.array(space_spec["lower"]),
                            np.array(space_spec["upper"]),
                            dtype=np.float64)
            elif space_type == "Enum":
                space = EnumSpace(space_spec["elements"])
            elif space_type == "PoseSpace":
                space = PoseSpace()
            elif space_type == "Text":
                space = Text(max_length=100000000, charset=frozenset(string.printable))
            else:
                raise ValueError(f"Unrecognized space type: {space_type}")
            param_obj = BehaviorTreeParameter(
                name=p_name,
                description=p_description,
                space=space,
                is_user_editable=p_is_user_editable
            )
            parameters.append(param_obj)
            param_value = p["value"]
            bindings[param_obj] = param_value
        fn = node_dict["fn"]  
        policy = FunctionalBehaviorTreeParameterizedPolicy(
            name=node_name,
            parameters=parameters,
            fn=fn
        )
        return ParameterizedActionBehaviorTreeNode(node_name, node_description, execution_log_path, policy, bindings)

    else:
        raise ValueError(f"Unknown node type: {node_type}")


def get_space_yaml_dict(space: Space) -> dict[str, Any]:
    """Get a YAML dict for a space."""
    if isinstance(space, Box):
        return {
            "type": "Box",
            "lower": space.low.tolist(),
            "upper": space.high.tolist(),
        }
            
    if isinstance(space, EnumSpace):
        return {
            "type": "Enum",
            "elements": space.elements,
        }
    
    if isinstance(space, PoseSpace):
        return {
            "type": "PoseSpace",
        }
    
    if isinstance(space, Text):
        return {
            "type": "Text",
        }

    raise ValueError(f"Unrecognized space type: {space}")