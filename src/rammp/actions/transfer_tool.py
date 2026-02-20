from typing import Any

import numpy as np
import time
import pickle
from scipy.spatial.transform import Rotation
from pathlib import Path
import json
from pybullet_helpers.geometry import Pose

try:
    import rospy
    from std_msgs.msg import Bool
except ModuleNotFoundError:
    ROSPY_IMPORTED = False

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
    ToolPrepared,
    ToolTransferDone,
)

from rammp.actions.feel_the_bite.outside_mouth_transfer import OutsideMouthTransfer
from rammp.perception.gestures_perception.static_gesture_detectors import mouth_open, head_nod

class TransferToolHLA(HighLevelAction):
    """Transfer drink."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.tool = None
        self.head_perception_log_dir = self.log_dir / "head_perception_log"
        self.head_perception_log_dir.mkdir(exist_ok=True)

        self.transfer = OutsideMouthTransfer(self.sim, self.robot_interface, self.perception_interface, self.rviz_interface, self.no_waits, self.head_perception_log_dir)
        
    def set_tool(self, tool):
        self.tool = tool

    def detect_initiate_transfer(self, initiate_transfer_interaction: str, ready_to_initiate_mode: str):
        if initiate_transfer_interaction == "button":
            self.perception_interface.detect_button_press()
        elif initiate_transfer_interaction == "open_mouth":
            print("Starting mouth open detection")
            mouth_open(self.perception_interface, termination_event=None, timeout=600) # 10 minutes
            print("Detected mouth open")
        elif initiate_transfer_interaction == "auto_timeout":
            time.sleep(5.0)
        else:
            raise NotImplementedError
        print("Initiating transfer")

    def detect_transfer_complete(self, transfer_complete_interaction: str, ready_for_transfer_interaction: str):
        if transfer_complete_interaction == "button":
            self.perception_interface.detect_button_press()
        elif transfer_complete_interaction == "sense":
            print("Starting head nod detection")
            head_nod(self.perception_interface, termination_event=None, timeout=600) # 10 minutes
            print("Head nod detected")
        elif transfer_complete_interaction == "auto_timeout":
            time.sleep(5.0)
        else:
            raise NotImplementedError
        print("Detected transfer completion")

    def relay_ready_to_initiate_transfer(self, ready_to_initiate_transfer_interaction: str, initiate_transfer_interaction: str):
        if ready_to_initiate_transfer_interaction == "silent":
            pass
        elif ready_to_initiate_transfer_interaction == "voice":
            if initiate_transfer_interaction == "button":
                self.perception_interface.speak("Please press the button when ready")
            elif initiate_transfer_interaction == "open_mouth":
                self.perception_interface.speak("Please open your mouth when ready")
            elif initiate_transfer_interaction == "auto_timeout":
                self.perception_interface.speak("Please wait 5 seconds for the transfer to initiate")
            else:
                # Check if the initiate_transfer_interaction is a synthesized gesture.
                gestures = dict(self.load_synthesized_gestures())
                # load from synthesized_gestures_dict_path
                with open(self.synthesized_gestures_dict_path, "r") as f:
                    synthesized_gesture_function_name_to_label = json.load(f)

                if initiate_transfer_interaction in gestures:
                    self.perception_interface.speak(f"Please do a {synthesized_gesture_function_name_to_label[initiate_transfer_interaction]} to initiate transfer")
                else:
                    raise NotImplementedError
        elif ready_to_initiate_transfer_interaction == "beep":
            self.perception_interface.speak("Beep")
        else:
            raise NotImplementedError

    def relay_ready_for_transfer(self, ready_for_transfer_interaction: str):
        if ready_for_transfer_interaction == "silent":
            pass
        elif ready_for_transfer_interaction == "voice":
            self.perception_interface.speak("Ready for transfer")
        elif ready_for_transfer_interaction == "beep":
            self.perception_interface.speak("Beep")
        else:
            raise NotImplementedError

    def execute_transfer(self, ready_to_initiate_mode: str, initiate_transfer_mode: str,
                         ready_to_transfer_mode: str, transfer_complete_mode: str,
                         outside_mouth_distance: float = 0.0,
                         maintain_position_at_goal = False):
        
        self.perception_interface.set_head_perception_tool(self.tool)
        print("--- Starting head perception thread ---")
        self.perception_interface.start_head_perception_thread()
        if self.robot_interface is not None:
            time.sleep(2.0) # let head perception thread warmstart / robot to stabilize
            self.robot_interface.set_tool(self.tool)
        else:
            time.sleep(1.0) # let sim head perception thread warmstart

        if self.robot_interface is not None:
            self.relay_ready_to_initiate_transfer(ready_to_initiate_mode, initiate_transfer_mode)
            self.detect_initiate_transfer(initiate_transfer_mode, ready_to_initiate_mode)

        self.transfer.set_tool(self.tool)
        self.transfer.move_to_transfer_state(outside_mouth_distance, maintain_position_at_goal)

        if self.robot_interface is not None:
            self.relay_ready_for_transfer(ready_to_transfer_mode)
            self.detect_transfer_complete(transfer_complete_mode, ready_to_transfer_mode)

        # shutdown the head perception thread
        self.perception_interface.stop_head_perception_thread()

        self.transfer.move_to_before_transfer_state()        

    def get_name(self) -> str:
        return "TransferTool"

    def get_operator(self) -> LiftedOperator:
        tool = Variable("?tool", tool_type)
        return LiftedOperator(
            self.get_name(),
            parameters=[tool],
            preconditions={Holding([tool]), ToolPrepared([tool])},
            add_effects={LiftedAtom(ToolTransferDone, [tool])},
            delete_effects={ToolPrepared([tool])},
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
        return f"transfer_{tool.name}.yaml"    

    def transfer_drink(self, speed: str, *args, **kwargs) -> None:
        assert self.sim.held_object_name == "drink"

        if self.robot_interface is not None:
            self.robot_interface.set_speed(speed)
        
        # Assume the second last item in args is the ask_confirmation
        ask_confirmation = args[-2]

        # Assume the last item in args is autocontinue time
        drink_autocontinue_time = args[-1]

        # All other items (everything except the last two) should go on to the next call
        remaining_args = args[:-2]


        if self.web_interface is not None:
            self.web_interface.set_drink_autocontinue_timeout(drink_autocontinue_time)
            if ask_confirmation:
                self.web_interface.get_drink_transfer_confirmation()

        # input("Press Enter to stop maintaining home orientation")

        if self.robot_interface is not None:
            self.robot_interface.stop_maintain_home_orientation()

        # self.move_to_joint_positions(self.sim.scene_description.home_pos)
        self.move_to_joint_positions(self.sim.scene_description.drink_transfer_waypoint_pos)
        self.move_to_joint_positions(self.sim.scene_description.drink_before_transfer_pos)

        # input("Ready to transfer drink... please implement me")

        self.set_tool("drink")    
        self.execute_transfer(*remaining_args, maintain_position_at_goal=True, **kwargs)

        self.move_to_joint_positions(self.sim.scene_description.drink_transfer_waypoint_pos)
        self.move_to_joint_positions(self.sim.scene_description.home_pos)

        if self.robot_interface is not None:
            self.robot_interface.start_maintain_home_orientation()
