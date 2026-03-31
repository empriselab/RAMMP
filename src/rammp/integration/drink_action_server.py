#!/usr/bin/env python3
from pathlib import Path
import argparse

import rospy
import actionlib
from drink_actions_test.msg import DrinkActionAction, DrinkActionFeedback, DrinkActionResult

# Interfaces
from rammp.interfaces.perception_interface import PerceptionInterface
from rammp.interfaces.rviz_interface import RVizInterface
from rammp.control.robot_controller.arm_client import ArmInterfaceClient
from rammp.simulation.scene_description import create_scene_description_from_config
from rammp.simulation.simulator import FeedingDeploymentPyBulletSimulator

from rammp.actions.bring_cup_to_mouth import BringCupToMouthAction
from rammp.actions.grab_cup_from_table import GrabCupFromTableAction
from rammp.actions.home_cup import HomeCupAction
from rammp.actions.pickup_and_order import PickupAndOrderAction
from rammp.actions.put_cup_back_to_holder import PutCupBackToHolderAction

class DrinkActionServers:
    def __init__(self, scene_config: str, run_on_robot: bool, use_gui: bool, no_waits: bool = False):

        self.log_dir = Path(__file__).parent / "log" 

        # Initialize the simulator.
        scene_config_path = Path(__file__).parent.parent / "simulation" / "configs" / f"{scene_config}.yaml"
        self.scene_description = create_scene_description_from_config(str(scene_config_path))

        self.perception_interface = PerceptionInterface(simulation=not run_on_robot, log_dir=self.log_dir)

        # Initialize the interface to the robot.
        if run_on_robot:
            self.robot_interface = ArmInterfaceClient()  # type: ignore  # pylint: disable=no-member
            self.rviz_interface = RVizInterface(self.scene_description)
        else:
            self.robot_interface = None
            self.rviz_interface = None

        self.sim = FeedingDeploymentPyBulletSimulator(self.scene_description, use_gui=use_gui, ignore_user=True)

        HLAS = {BringCupToMouthAction, GrabCupFromTableAction, HomeCupAction, PickupAndOrderAction, PutCupBackToHolderAction}

        self.hlas = {
            cls(self.sim, self.robot_interface, self.perception_interface, self.rviz_interface, no_waits, self.log_dir) for cls in HLAS  # type: ignore
        }
        print("HLAs created.")
        self.hla_name_to_hla = {hla.get_name(): hla for hla in self.hlas}

        self.pickup_and_order_server = actionlib.SimpleActionServer(
            "/arm/drink/pickup_and_order",
            DrinkActionAction,
            execute_cb=self.execute_pickup_and_order,
            auto_start=False,
        )

        self.grab_cup_from_table_server = actionlib.SimpleActionServer(
            "/arm/drink/grab_cup_from_table",
            DrinkActionAction,
            execute_cb=self.execute_grab_cup_from_table,
            auto_start=False,
        )

        self.bring_cup_to_mouth_server = actionlib.SimpleActionServer(
            "/arm/drink/bring_cup_to_mouth",
            DrinkActionAction,
            execute_cb=self.execute_bring_cup_to_mouth,
            auto_start=False,
        )

        self.home_cup_server = actionlib.SimpleActionServer(
            "/arm/drink/home_cup",
            DrinkActionAction,
            execute_cb=self.execute_home_cup,
            auto_start=False,
        )

        self.put_cup_back_to_holder_server = actionlib.SimpleActionServer(
            "/arm/drink/put_cup_back_to_holder",
            DrinkActionAction,
            execute_cb=self.execute_put_cup_back_to_holder,
            auto_start=False,
        )

        self.pickup_and_order_server.start()
        self.grab_cup_from_table_server.start()
        self.bring_cup_to_mouth_server.start()
        self.home_cup_server.start()
        self.put_cup_back_to_holder_server.start()

        rospy.loginfo("All 5 drink action servers are up.")

    def _publish_dummy_feedback(self, server, state_text):
        feedback = DrinkActionFeedback()
        feedback.state = state_text
        server.publish_feedback(feedback)

    def _finish_success(self, server, msg):
        result = DrinkActionResult()
        result.success = True
        result.message = msg
        server.set_succeeded(result)

    def _finish_abort(self, server, msg):
        result = DrinkActionResult()
        result.success = False
        result.message = msg
        server.set_aborted(result)

    def execute_pickup_and_order(self, goal):
        rospy.loginfo("pickup_and_order goal received: %s", goal.request_id)
        self._publish_dummy_feedback(self.pickup_and_order_server, "starting pickup_and_order")
        self.hla_name_to_hla["PickupAndOrder"].execute_action()
        self._finish_success(self.pickup_and_order_server, "pickup_and_order dummy implementation complete")

    def execute_grab_cup_from_table(self, goal):
        rospy.loginfo("grab_cup_from_table goal received: %s", goal.request_id)
        self._publish_dummy_feedback(self.grab_cup_from_table_server, "starting grab_cup_from_table")
        self.hla_name_to_hla["GrabCupFromTable"].execute_action()
        self._finish_success(self.grab_cup_from_table_server, "grab_cup_from_table dummy implementation complete")

    def execute_bring_cup_to_mouth(self, goal):
        rospy.loginfo("bring_cup_to_mouth goal received: %s", goal.request_id)
        self._publish_dummy_feedback(self.bring_cup_to_mouth_server, "starting bring_cup_to_mouth")
        self.hla_name_to_hla["BringCupToMouth"].execute_action()
        self._finish_success(self.bring_cup_to_mouth_server, "bring_cup_to_mouth dummy implementation complete")

    def execute_home_cup(self, goal):
        rospy.loginfo("home_cup goal received: %s", goal.request_id)
        self._publish_dummy_feedback(self.home_cup_server, "starting home_cup")
        self.hla_name_to_hla["HomeCup"].execute_action()
        self._finish_success(self.home_cup_server, "home_cup dummy implementation complete")

    def execute_put_cup_back_to_holder(self, goal):
        rospy.loginfo("put_cup_back_to_holder goal received: %s", goal.request_id)
        self._publish_dummy_feedback(self.put_cup_back_to_holder_server, "starting put_cup_back_to_holder")
        self.hla_name_to_hla["PutCupBackToHolder"].execute_action()
        self._finish_success(self.put_cup_back_to_holder_server, "put_cup_back_to_holder dummy implementation complete")

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--scene_config", type=str, default="wheelchair") # name of the scene config (rough head-plate-robot setup)
    parser.add_argument("--run_on_robot", action="store_true")
    parser.add_argument("--use_gui", action="store_true")
    parser.add_argument("--no_waits", action="store_true")
    args = parser.parse_args()


    rospy.init_node("drink_action_server")
    DrinkActionServers(
        scene_config=args.scene_config,
        run_on_robot=args.run_on_robot,
        use_gui=args.use_gui,
        no_waits=args.no_waits,
    )
    rospy.spin()