"""Demo runner that calls the cornell_feeding drinking action servers.

This script acts as a ROS 2 action client and exercises the feeding
actions exposed by the drinking_node in Demo-Software/cornell_feeding:
  1. GrabCupFromTable    – /arm/drink/grab_cup_from_table
  2. BringCupToMouth     – /arm/drink/bring_cup_to_mouth
  3. HomeCup             – /arm/drink/home_cup
  4. PutCupBackToHolder  – /arm/drink/put_cup_back_to_holder
  5. PickupAndOrder      – /arm/drink/pickup_and_order

Usage:
    ros2 run rammp demo_run
"""

import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from cornell_feeding_interfaces.action import (
    GrabCupFromTable,
    BringCupToMouth,
    HomeCup,
    PutCupBackToHolder,
    PickupAndOrder,
)


class FeedingDemoClient(Node):
    """Action client node that sequences the feeding demo actions."""

    def __init__(self):
        super().__init__("feeding_demo_client")

        self._grab_cup_client = ActionClient(
            self, GrabCupFromTable, "/arm/drink/grab_cup_from_table"
        )
        self._bring_cup_client = ActionClient(
            self, BringCupToMouth, "/arm/drink/bring_cup_to_mouth"
        )
        self._home_cup_client = ActionClient(
            self, HomeCup, "/arm/drink/home_cup"
        )
        self._put_cup_back_client = ActionClient(
            self, PutCupBackToHolder, "/arm/drink/put_cup_back_to_holder"
        )
        self._pickup_and_order_client = ActionClient(
            self, PickupAndOrder, "/arm/drink/pickup_and_order"
        )

    def wait_for_servers(self, timeout_sec: float = 10.0) -> bool:
        """Wait for all action servers to become available."""
        clients = [
            (self._grab_cup_client, "/arm/drink/grab_cup_from_table"),
            (self._bring_cup_client, "/arm/drink/bring_cup_to_mouth"),
            (self._home_cup_client, "/arm/drink/home_cup"),
            (self._put_cup_back_client, "/arm/drink/put_cup_back_to_holder"),
            (self._pickup_and_order_client, "/arm/drink/pickup_and_order"),
        ]
        for client, name in clients:
            self.get_logger().info(f"Waiting for action server: {name}")
            if not client.wait_for_server(timeout_sec=timeout_sec):
                self.get_logger().error(f"Action server {name} not available")
                return False
        self.get_logger().info("All action servers are available.")
        return True

    def send_goal_sync(self, client: ActionClient, goal_msg, action_name: str):
        """Send a goal and block until the result is received."""
        self.get_logger().info(f"Sending goal to {action_name}")

        send_future = client.send_goal_async(
            goal_msg, feedback_callback=lambda fb: self._on_feedback(action_name, fb)
        )
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error(f"Goal rejected by {action_name}")
            return None

        self.get_logger().info(f"Goal accepted by {action_name}")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if result.success:
            self.get_logger().info(f"[{action_name}] Success: {result.message}")
        else:
            self.get_logger().error(f"[{action_name}] Failed: {result.message}")

        return result

    def _on_feedback(self, action_name: str, feedback_msg):
        self.get_logger().info(
            f"[{action_name}] Feedback: {feedback_msg.feedback.status}"
        )

    def run_feeding_demo(self):
        """Execute the full feeding demo sequence."""
        # Step 1: Grab the cup from table
        grab_goal = GrabCupFromTable.Goal()
        result = self.send_goal_sync(
            self._grab_cup_client, grab_goal, "GrabCupFromTable"
        )
        if result is None or not result.success:
            self.get_logger().error("GrabCupFromTable failed, aborting demo.")
            return

        # Step 2: Bring cup to mouth
        bring_goal = BringCupToMouth.Goal()
        result = self.send_goal_sync(
            self._bring_cup_client, bring_goal, "BringCupToMouth"
        )
        if result is None or not result.success:
            self.get_logger().error("BringCupToMouth failed, aborting demo.")
            return

        # Step 3: Home the cup (retract after drinking)
        home_goal = HomeCup.Goal()
        result = self.send_goal_sync(
            self._home_cup_client, home_goal, "HomeCup"
        )
        if result is None or not result.success:
            self.get_logger().error("HomeCup failed, aborting demo.")
            return

        # Step 4: Put the cup back to holder
        put_goal = PutCupBackToHolder.Goal()
        result = self.send_goal_sync(
            self._put_cup_back_client, put_goal, "PutCupBackToHolder"
        )
        if result is None or not result.success:
            self.get_logger().error("PutCupBackToHolder failed, aborting demo.")
            return

        self.get_logger().info("Feeding demo completed successfully!")


def main():
    rclpy.init()
    node = FeedingDemoClient()

    if not node.wait_for_servers(timeout_sec=10.0):
        node.get_logger().error("Could not connect to action servers. Is drinking_node running?")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    node.run_feeding_demo()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
