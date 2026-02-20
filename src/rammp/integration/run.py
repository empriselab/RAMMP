"""The main entry point for running the integrated system."""

from pathlib import Path
import queue
import os
import sys
import signal
import shutil
import numpy as np
import pickle
from tomsutils.llm import OpenAILLM
import time

try:
    import rospy
    from std_msgs.msg import String

    ROSPY_IMPORTED = True
except ModuleNotFoundError:
    ROSPY_IMPORTED = False

from relational_structs import (
    GroundAtom,
    LiftedAtom,
    Object,
    PDDLDomain,
    PDDLProblem,
    Predicate,
)
from relational_structs.utils import parse_pddl_plan, get_object_combinations
from tomsutils.pddl_planning import run_pddl_planner
from tomsutils.spaces import EnumSpace
from pybullet_helpers.geometry import Pose

from rammp.actions.base import (
    GripperFree,
    Holding,
    ToolPrepared,
    ToolTransferDone,
    EmulateTransferDone,
    ResetPos,
    tool_type,
    GroundHighLevelAction,
    ResetHLA,
    pddl_plan_to_hla_plan,
    load_behavior_tree,
    save_behavior_tree,
)

# Interfaces
from rammp.interfaces.perception_interface import PerceptionInterface
from rammp.interfaces.web_interface import WebInterface
from rammp.interfaces.rviz_interface import RVizInterface
from rammp.control.robot_controller.arm_client import ArmInterfaceClient
from rammp.simulation.scene_description import (
    SceneDescription,
    create_scene_description_from_config,
)
from rammp.simulation.simulator import (
    FeedingDeploymentPyBulletSimulator,
    FeedingDeploymentWorldState,
)

# Tool skills
from rammp.actions.pick_tool import PickToolHLA
from rammp.actions.stow_tool import StowToolHLA
from rammp.actions.transfer_tool import TransferToolHLA

# All the high level actions we want to consider.
HLAS = {PickToolHLA, StowToolHLA, TransferToolHLA, ResetHLA}

assert os.environ.get("PYTHONHASHSEED") == "0", \
        "Please add `export PYTHONHASHSEED=0` to your bash profile!"

class _Runner:
    """A class for running the integrated system."""

    def __init__(self, scene_config: str, user: str, scenario:str, run_on_robot: bool, use_interface: bool, use_gui: bool, simulate_head_perception: bool, max_motion_planning_time: float,
                 resume_from_state: str = "", no_waits: bool = False) -> None:
        self.run_on_robot = run_on_robot
        self.use_interface = use_interface  
        self.simulate_head_perception = simulate_head_perception
        self.max_motion_planning_time = max_motion_planning_time
        self.no_waits = no_waits

        # logs are saved in user/scenario directory
        self.log_dir = Path(__file__).parent / "log" / user / scenario
        self.execution_log = Path(__file__).parent / "log" / "execution_log.txt" # in root log directory
        self.run_behavior_tree_dir = self.log_dir / "behavior_trees"
        
        if not self.log_dir.exists():
            if not (self.log_dir.parent).exists(): # new user
                assert scenario == "default", "First run with a new user must be in default scenario."
                os.makedirs(self.log_dir, exist_ok=True)
                
                # Copy the initial behavior trees into a directory for this run, where
                # they will be modified based on user feedback.
                self.run_behavior_tree_dir.mkdir(exist_ok=True)
                original_behavior_tree_dir = Path(__file__).parents[1] / "actions" / "behavior_trees"
                assert original_behavior_tree_dir.exists()
                for original_bt_filename in original_behavior_tree_dir.glob("*.yaml"):
                    shutil.copy(original_bt_filename, self.run_behavior_tree_dir)

                # Copy the initial gesture detection file into a directory for this run,
                # where it will be updated from LLM-based few-shot learning.
                self.gesture_detectors_dir.mkdir(exist_ok=True)
                original_gesture_detection_filepath = Path(__file__).parents[1] / "perception" / "gestures_perception" / "synthesized_gesture_detectors.py"
                assert original_gesture_detection_filepath.exists()
                shutil.copy(original_gesture_detection_filepath, self.gesture_detectors_dir)

            elif not (self.log_dir).exists(): # new scenario
                assert (Path(__file__).parent / "log" / user / "default").exists(), "Do not have default scenario for this user."
                os.makedirs(self.log_dir, exist_ok=True)
                shutil.copytree(Path(__file__).parent / "log" / user / "default", self.log_dir, dirs_exist_ok=True)

        if resume_from_state == "":
            # clear behavior tree execution log
            with open(self.execution_log, "w") as f:
                f.write("")
            self._saved_state_infile = None
        else:
            self._saved_state_infile = Path(__file__).parent / "saved_states" / (resume_from_state + ".p")
        self._saved_state_outfile = Path(__file__).parent / "saved_states" / "last_state.p"

        # Initialize the interface to the robot.
        if run_on_robot:
            self.robot_interface = ArmInterfaceClient()  # type: ignore  # pylint: disable=no-member
        else:
            self.robot_interface = None

        self.llm = OpenAILLM(
            model_name="gpt-4.1-2025-04-14",
            cache_dir=self.log_dir / "llm_cache",
        )

        # Initialize the perceiver (e.g., get joint states or human head poses).
        self.perception_interface = PerceptionInterface(robot_interface=self.robot_interface, simulate_head_perception=self.simulate_head_perception, log_dir=self.log_dir)

        # Initialize the simulator.
        scene_config_path = Path(__file__).parent.parent / "simulation" / "configs" / f"{scene_config}.yaml"
        self.scene_description = create_scene_description_from_config(str(scene_config_path))

        if run_on_robot:
            if not np.allclose(self.scene_description.initial_joints, self.perception_interface.get_robot_joints(), atol=0.2):
                print("Initial joint state in scene description does not match the actual robot joint state.")
                print("Initial Robot Joints:", self.perception_interface.get_robot_joints())
                print("Initial Joints in Scene Description:", self.scene_description.initial_joints)
                
        else:
            print("Running in simulation mode.")

        if self.run_on_robot:
            self.rviz_interface = RVizInterface(self.scene_description)
        else:
            self.rviz_interface = None

        self.sim = FeedingDeploymentPyBulletSimulator(self.scene_description, use_gui=use_gui, ignore_user=True)

        if self.use_interface:
            # Initialize the web interface.
            self.task_selection_queue = queue.Queue()
            self.web_interface = WebInterface(self.task_selection_queue, self.log_dir)
        else:
            self.web_interface = None

        # Create skills for high-level planning.
        hla_hyperparams = {"max_motion_planning_time": max_motion_planning_time}
        print("Creating HLAs...")
        self.hlas = {
            cls(self.sim, self.robot_interface, self.perception_interface, self.rviz_interface, self.web_interface, hla_hyperparams,
                self.no_waits, self.log_dir, self.run_behavior_tree_dir, self.execution_log) for cls in HLAS  # type: ignore
        }
        print("HLAs created.")
        self.hla_name_to_hla = {hla.get_name(): hla for hla in self.hlas}
        self.operators = {hla.get_operator() for hla in self.hlas}
        self.predicates: set[Predicate] = {
            ToolPrepared,
            GripperFree,
            Holding,
            ToolTransferDone,
            EmulateTransferDone,
            ResetPos,
        }
        self.types = {tool_type}
        self.domain = PDDLDomain(
            "AssistedFeeding", self.operators, self.predicates, self.types
        )
        self.drink = Object("drink", tool_type)
        self.all_objects = {self.drink}
        self.object_name_to_object = {"drink": self.drink}
        # Create all ground HLAs that will be used.
        self._all_ground_hlas = []
        for hla_name, hla in sorted(self.hla_name_to_hla.items()):
            types = [p.type for p in hla.get_operator().parameters]
            for obj_combo in get_object_combinations(sorted(self.all_objects), types):
                ground_hla = (hla, obj_combo)
                self._all_ground_hlas.append(ground_hla)
        # Rewrite the behavior trees to avoid any inconsistencies.
        for hla, objs in self._all_ground_hlas:
            try:
                bt_filepath = hla.behavior_tree_dir / hla.get_behavior_tree_filename(objs, {})
            except NotImplementedError:
                continue
            bt = load_behavior_tree(bt_filepath, hla)
            save_behavior_tree(bt, bt_filepath, hla)

        # Track the current high-level state.
        self.current_atoms = {
            LiftedAtom(GripperFree, []),
            ToolPrepared([self.drink]),
        }

        if self._saved_state_infile:
            self._load_from_state()
            print("WARNING: The system state has been restored to:")
            print(" ", sorted(self.current_atoms))
            resp = input("Are you sure you want to continue from here? [y/n] ")
            while resp not in ["y", "n"]:
                resp = input("Please enter 'y' or 'n': ")
                if resp == "n":
                    self.stop_all_threads()
                    sys.exit(0)

        print("Runner is ready.")
        self.active = True

    def run(self) -> None:

        assert self.web_interface is not None, "Run takes user commands from the web interface which is None."
        
        self.web_interface.ready_for_task_selection()
        last_task_type = None
        while self.active:
            try:
                task_selection_command = self.task_selection_queue.get(timeout=1)
                self.web_interface.clear_received_messages() # So that only the latest message is processed
                task, task_type = task_selection_command["task"], task_selection_command["type"]
                if task == "reset":
                    self.process_user_command(GroundHighLevelAction(self.hla_name_to_hla["Reset"], ()))
                    last_task_type = None
                elif task == "meal_assistance":
                    if task_type == "sip":
                        self.process_user_command(GroundHighLevelAction(self.hla_name_to_hla["TransferTool"], (self.drink,)))
                    else:
                        print(f"Invalid meal assistance task type: {task_type}")
                    last_task_type = task_type
                else:
                    print(f"Invalid task selection: {task_selection_command}")
                    last_task_type = None
                # self.web_interface.clear_received_messages() # So that only the latest message is processed
                # time.sleep(1.0)
                self.web_interface.ready_for_task_selection(last_task_type=last_task_type)
                print("Ready for next user command.")
                print("Current web interface page:", self.web_interface.current_page)
            except queue.Empty:
                # Wait for user commands.
                time.sleep(0.1) 
                continue

    def stop_all_threads(self) -> None:
        self.active = False
        if self.web_interface is not None:
            self.web_interface.stop_all_threads()

    def signal_handler(self, signal, frame):
        print("\nReceived SIGINT.")
        self.stop_all_threads()
        print("\nprogram exiting gracefully")
        sys.exit(0)

    def process_user_command(
        self, user_command: GroundHighLevelAction | set[GroundAtom]
    ) -> None:
        """Process a user command."""

        print(f"Working towards user command: {user_command}")

        # Plan to the preconditions of the HLA.
        if isinstance(user_command, GroundHighLevelAction):
            goal_atoms = user_command.get_preconditions()
        else:
            goal_atoms = user_command
        problem = PDDLProblem(
            self.domain.name,
            "AssistedFeeding",
            self.all_objects,
            self.current_atoms,
            goal_atoms,
        )
        plan_strs = run_pddl_planner(
            str(self.domain), str(problem), planner="fd-opt",
        )
        assert plan_strs is not None
        plan_ops = parse_pddl_plan(plan_strs, self.domain, problem)
        print("Found plan to the preconditions of the command:")
        for i, op in enumerate(plan_ops):
            print(f"{i}. {op.short_str}")
        plan_hlas = pddl_plan_to_hla_plan(plan_ops, self.hlas)
        # Append the user command to the plan if it's an action.
        if isinstance(user_command, GroundHighLevelAction):
            plan_hlas.append(user_command)

        for ground_hla in plan_hlas:
            print(f"Refining {ground_hla}")
            operator = ground_hla.get_operator()

            # import ipdb; ipdb.set_trace()
            assert operator.preconditions.issubset(self.current_atoms)

            # Execute the high-level plan in simulation
            ground_hla.execute_action()

            sim_state = self.sim.get_current_state()

            self.current_atoms -= operator.delete_effects
            self.current_atoms |= operator.add_effects

            # Super hack: the drink and wipe are always prepared.
            self.current_atoms.add(ToolPrepared([self.drink]))

            # Save the latest state in case we want to resume execution
            # after a crash.
            self._save_state(sim_state, self.current_atoms)

    def make_video(self, outfile: Path) -> None:
        """Create a video of the simulated trajectory."""
        self.sim.make_simulation_video(outfile)
        print(f"Saved video to {outfile}")

    def _save_state(self, sim_state: FeedingDeploymentWorldState, atoms: set[GroundAtom]) -> None:
        with open(self._saved_state_outfile, "wb") as f:
            pickle.dump((sim_state, atoms), f)
        print(f"Saved system state to {self._saved_state_outfile}")

    def _load_from_state(self) -> None:
        with open(self._saved_state_infile, "rb") as f:
            sim_state, self.current_atoms = pickle.load(f)
        if sim_state is not None:
            assert isinstance(sim_state, FeedingDeploymentWorldState)
            self.sim.sync(sim_state)
            if self.rviz_interface is not None:
                self.rviz_interface.joint_state_update(sim_state.robot_joints)
                if sim_state.held_object:
                    self.rviz_interface.tool_update(True, sim_state.held_object, Pose((0, 0, 0), (0, 0, 0, 1)))
                
        print(f"Loaded system state from {self._saved_state_infile}")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--scene_config", type=str, default="wheelchair") # name of the scene config (rough head-plate-robot setup)
    parser.add_argument("--user", type=str, default="default") # name of the user
    parser.add_argument("--scenario", type=str, default="default") # name of the scenario
    parser.add_argument("--run_on_robot", action="store_true")
    parser.add_argument("--use_interface", action="store_true")
    parser.add_argument("--use_gui", action="store_true")
    parser.add_argument("--simulate_head_perception", action="store_true")
    parser.add_argument("--make_videos", action="store_true")
    parser.add_argument("--max_motion_planning_time", type=float, default=10.0)
    parser.add_argument("--resume_from_state", type=str, default="")
    parser.add_argument("--no_waits", action="store_true")
    args = parser.parse_args()

    if args.user == "":
        raise ValueError("Please provide a user name.")

    if args.run_on_robot or args.use_interface:
        if not ROSPY_IMPORTED:
            raise ModuleNotFoundError("Need ROS to run on robot or use interface")
        else:
            rospy.init_node("rammp", anonymous=True)

    runner = _Runner(args.scene_config,
                     args.user,
                     args.scenario,
                     args.run_on_robot, 
                     args.use_interface,
                     args.use_gui,
                     args.simulate_head_perception,
                     args.max_motion_planning_time,
                     args.resume_from_state,
                     args.no_waits)
    
    # Handle Ctrl+C gracefully
    signal.signal(signal.SIGINT, runner.signal_handler)
    
    if not args.use_interface:
        runner.process_user_command(GroundHighLevelAction(runner.hla_name_to_hla["TransferTool"], (runner.drink,)))
        # for i in range(10):
            # runner.process_user_command(GroundHighLevelAction(runner.hla_name_to_hla["PickTool"], (runner.drink,)))
            # runner.process_user_command(GroundHighLevelAction(runner.hla_name_to_hla["StowTool"], (runner.drink,)))
    else:
        runner.run()

    if args.make_videos:
        output_path = Path(__file__).parent / "videos" / "full.mp4"
        runner.make_video(output_path)

    if args.run_on_robot:
        rospy.spin()