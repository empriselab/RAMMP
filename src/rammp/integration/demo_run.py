"""Demo runner that calls the cornell_feeding dummy action servers.

This script acts as a ROS 2 action client and exercises the feeding
actions exposed by the drinking_node in Demo-Software/cornell_feeding:
  1. GrabCup        – /arm/drink/grab_cup_from_table
  2. BringCupToMouth – /arm/drink/bring_cup_to_mouth
  3. HomeCup         – /arm/drink/home_cup
  4. PutCupBack      – /arm/drink/put_cup_back_to_holder

Usage:
    ros2 run rammp demo_run              # default: grab from table, put back to table
    ros2 run rammp demo_run --source wheelchair --destination wheelchair
"""

import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from cornell_feeding_interfaces.action import (
    GrabCup,
    BringCupToMouth,
    HomeCup,
    PutCupBack,
)


class FeedingDemoClient(Node):
    """Action client node that sequences the feeding demo actions."""

    def __init__(self):
        super().__init__("feeding_demo_client")

        self._grab_cup_client = ActionClient(
            self, GrabCup, "/arm/drink/grab_cup_from_table"
        )
        self._bring_cup_client = ActionClient(
            self, BringCupToMouth, "/arm/drink/bring_cup_to_mouth"
        )
        self._home_cup_client = ActionClient(
            self, HomeCup, "/arm/drink/home_cup"
        )
        self._put_cup_back_client = ActionClient(
            self, PutCupBack, "/arm/drink/put_cup_back_to_holder"
        )

    def wait_for_servers(self, timeout_sec: float = 10.0) -> bool:
        """Wait for all action servers to become available."""
        clients = [
            (self._grab_cup_client, "/arm/drink/grab_cup_from_table"),
            (self._bring_cup_client, "/arm/drink/bring_cup_to_mouth"),
            (self._home_cup_client, "/arm/drink/home_cup"),
            (self._put_cup_back_client, "/arm/drink/put_cup_back_to_holder"),
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

    def run_feeding_demo(self, source: str, destination: str, outside_mouth_distance: float):
        """Execute the full feeding demo sequence."""
        # Step 1: Grab the cup
        grab_goal = GrabCup.Goal()
        grab_goal.source = source
        result = self.send_goal_sync(
            self._grab_cup_client, grab_goal, "GrabCup"
        )
        if result is None or not result.success:
            self.get_logger().error("GrabCup failed, aborting demo.")
            return

        # Step 2: Bring cup to mouth
        bring_goal = BringCupToMouth.Goal()
        bring_goal.outside_mouth_distance = outside_mouth_distance
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

        # Step 4: Put the cup back
        put_goal = PutCupBack.Goal()
        put_goal.destination = destination
        result = self.send_goal_sync(
            self._put_cup_back_client, put_goal, "PutCupBack"
        )
        if result is None or not result.success:
            self.get_logger().error("PutCupBack failed, aborting demo.")
            return

        self.get_logger().info("Feeding demo completed successfully!")


def main():
    parser = argparse.ArgumentParser(description="Run the feeding demo action client")
    parser.add_argument("--source", type=str, default="table",
                        choices=["table", "wheelchair"],
                        help="Where to grab the cup from")
    parser.add_argument("--destination", type=str, default="table",
                        choices=["table", "wheelchair"],
                        help="Where to put the cup back")
    parser.add_argument("--outside_mouth_distance", type=float, default=0.05,
                        help="Distance to hold cup from mouth (meters)")
    parser.add_argument("--timeout", type=float, default=10.0,
                        help="Timeout waiting for action servers (seconds)")
    args = parser.parse_args()

    rclpy.init()
    node = FeedingDemoClient()

    if not node.wait_for_servers(timeout_sec=args.timeout):
        node.get_logger().error("Could not connect to action servers. Is drinking_node running?")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    node.run_feeding_demo(args.source, args.destination, args.outside_mouth_distance)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
