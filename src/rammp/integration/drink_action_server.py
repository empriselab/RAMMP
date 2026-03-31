#!/usr/bin/env python3
from pathlib import Path
import argparse

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor

from drink_actions_test.action import DrinkAction

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


class DrinkActionServers(Node):
    def __init__(
        self,
        scene_config: str,
        run_on_robot: bool,
        use_gui: bool,
        no_waits: bool = False,
    ):
        super().__init__("drink_action_server")

        self.log_dir = Path(__file__).parent / "log"

        # Initialize the simulator.
        scene_config_path = (
            Path(__file__).parent.parent
            / "simulation"
            / "configs"
            / f"{scene_config}.yaml"
        )
        self.scene_description = create_scene_description_from_config(
            str(scene_config_path)
        )
        
        self.perception_interface = PerceptionInterface(
            node=self,
            simulation=not run_on_robot,
            log_dir=self.log_dir,
        )

        if run_on_robot:
            self.robot_interface = ArmInterfaceClient(node=self)
            self.rviz_interface = RVizInterface(node=self, scene_description=self.scene_description)
        else:
            self.robot_interface = None
            self.rviz_interface = None

        self.sim = FeedingDeploymentPyBulletSimulator(
            self.scene_description,
            use_gui=use_gui,
            ignore_user=True,
        )

        hlas_classes = {
            BringCupToMouthAction,
            GrabCupFromTableAction,
            HomeCupAction,
            PickupAndOrderAction,
            PutCupBackToHolderAction,
        }

        self.hlas = {
            cls(
                self.sim,
                self.robot_interface,
                self.perception_interface,
                self.rviz_interface,
                no_waits,
                self.log_dir,
            )
            for cls in hlas_classes
        }
        print("HLAs created.")
        self.hla_name_to_hla = {hla.get_name(): hla for hla in self.hlas}

        self.pickup_and_order_server = ActionServer(
            self,
            DrinkAction,
            "/arm/drink/pickup_and_order",
            self.execute_pickup_and_order,
        )

        self.grab_cup_from_table_server = ActionServer(
            self,
            DrinkAction,
            "/arm/drink/grab_cup_from_table",
            self.execute_grab_cup_from_table,
        )

        self.bring_cup_to_mouth_server = ActionServer(
            self,
            DrinkAction,
            "/arm/drink/bring_cup_to_mouth",
            self.execute_bring_cup_to_mouth,
        )

        self.home_cup_server = ActionServer(
            self,
            DrinkAction,
            "/arm/drink/home_cup",
            self.execute_home_cup,
        )

        self.put_cup_back_to_holder_server = ActionServer(
            self,
            DrinkAction,
            "/arm/drink/put_cup_back_to_holder",
            self.execute_put_cup_back_to_holder,
        )

        self.get_logger().info("All 5 drink action servers are up.")

    def _publish_dummy_feedback(self, goal_handle, state_text: str):
        feedback = DrinkAction.Feedback()
        feedback.state = state_text
        goal_handle.publish_feedback(feedback)

    def _finish_success(self, goal_handle, msg: str):
        result = DrinkAction.Result()
        result.success = True
        result.message = msg
        goal_handle.succeed()
        return result

    def _finish_abort(self, goal_handle, msg: str):
        result = DrinkAction.Result()
        result.success = False
        result.message = msg
        goal_handle.abort()
        return result

    def execute_pickup_and_order(self, goal_handle):
        self.get_logger().info(
            f"pickup_and_order goal received: {goal_handle.request.request_id}"
        )
        self._publish_dummy_feedback(goal_handle, "starting pickup_and_order")
        try:
            self.hla_name_to_hla["PickupAndOrder"].execute_action()
            return self._finish_success(
                goal_handle,
                "pickup_and_order dummy implementation complete",
            )
        except Exception as exc:
            self.get_logger().error(f"pickup_and_order failed: {exc}")
            return self._finish_abort(
                goal_handle,
                f"pickup_and_order failed: {exc}",
            )

    def execute_grab_cup_from_table(self, goal_handle):
        self.get_logger().info(
            f"grab_cup_from_table goal received: {goal_handle.request.request_id}"
        )
        self._publish_dummy_feedback(goal_handle, "starting grab_cup_from_table")
        try:
            self.hla_name_to_hla["GrabCupFromTable"].execute_action()
            return self._finish_success(
                goal_handle,
                "grab_cup_from_table dummy implementation complete",
            )
        except Exception as exc:
            self.get_logger().error(f"grab_cup_from_table failed: {exc}")
            return self._finish_abort(
                goal_handle,
                f"grab_cup_from_table failed: {exc}",
            )

    def execute_bring_cup_to_mouth(self, goal_handle):
        self.get_logger().info(
            f"bring_cup_to_mouth goal received: {goal_handle.request.request_id}"
        )
        self._publish_dummy_feedback(goal_handle, "starting bring_cup_to_mouth")
        try:
            self.hla_name_to_hla["BringCupToMouth"].execute_action()
            return self._finish_success(
                goal_handle,
                "bring_cup_to_mouth dummy implementation complete",
            )
        except Exception as exc:
            self.get_logger().error(f"bring_cup_to_mouth failed: {exc}")
            return self._finish_abort(
                goal_handle,
                f"bring_cup_to_mouth failed: {exc}",
            )

    def execute_home_cup(self, goal_handle):
        self.get_logger().info(
            f"home_cup goal received: {goal_handle.request.request_id}"
        )
        self._publish_dummy_feedback(goal_handle, "starting home_cup")
        try:
            self.hla_name_to_hla["HomeCup"].execute_action()
            return self._finish_success(
                goal_handle,
                "home_cup dummy implementation complete",
            )
        except Exception as exc:
            self.get_logger().error(f"home_cup failed: {exc}")
            return self._finish_abort(
                goal_handle,
                f"home_cup failed: {exc}",
            )

    def execute_put_cup_back_to_holder(self, goal_handle):
        self.get_logger().info(
            f"put_cup_back_to_holder goal received: {goal_handle.request.request_id}"
        )
        self._publish_dummy_feedback(goal_handle, "starting put_cup_back_to_holder")
        try:
            self.hla_name_to_hla["PutCupBackToHolder"].execute_action()
            return self._finish_success(
                goal_handle,
                "put_cup_back_to_holder dummy implementation complete",
            )
        except Exception as exc:
            self.get_logger().error(f"put_cup_back_to_holder failed: {exc}")
            return self._finish_abort(
                goal_handle,
                f"put_cup_back_to_holder failed: {exc}",
            )


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--scene_config",
        type=str,
        default="wheelchair",
    )  # name of the scene config (rough head-plate-robot setup)
    parser.add_argument("--run_on_robot", action="store_true")
    parser.add_argument("--use_gui", action="store_true")
    parser.add_argument("--no_waits", action="store_true")
    parsed_args = parser.parse_args()

    rclpy.init(args=args)

    node = DrinkActionServers(
        scene_config=parsed_args.scene_config,
        run_on_robot=parsed_args.run_on_robot,
        use_gui=parsed_args.use_gui,
        no_waits=parsed_args.no_waits,
    )

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()