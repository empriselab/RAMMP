#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from drink_actions_test.action import DrinkAction


ACTION_TOPICS = {
    "pickup_and_order": "/arm/drink/pickup_and_order",
    "grab_cup_from_table": "/arm/drink/grab_cup_from_table",
    "bring_cup_to_mouth": "/arm/drink/bring_cup_to_mouth",
    "home_cup": "/arm/drink/home_cup",
    "put_cup_back_to_holder": "/arm/drink/put_cup_back_to_holder",
}


class DrinkActionClientNode(Node):
    def __init__(self):
        super().__init__("drink_action_client")
        self._action_client = None
        self._goal_done = False

    def feedback_cb(self, feedback_msg):
        # In ROS 2, feedback callback receives a message wrapper with `.feedback`
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Feedback: {feedback.state}")

    def send_goal(self, topic: str, request_id: str):
        self._action_client = ActionClient(self, DrinkAction, topic)

        self.get_logger().info(f"Waiting for server: {topic}")
        self._action_client.wait_for_server()

        goal_msg = DrinkAction.Goal()
        goal_msg.request_id = request_id

        self.get_logger().info(f"Sending goal to {topic}")
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_cb,
        )
        send_goal_future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by server")
            self._goal_done = True
            return

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.result_cb)

    def result_cb(self, future):
        result = future.result().result
        self.get_logger().info(
            f"Result: success={result.success} message={result.message}"
        )
        self._goal_done = True


def main():
    rclpy.init()

    if len(sys.argv) < 2:
        print(
            "Usage: ros2 run drink_actions_test drink_action_client "
            "<action_name> [request_id]"
        )
        sys.exit(1)

    action_name = sys.argv[1]
    request_id = sys.argv[2] if len(sys.argv) > 2 else "test_request"

    if action_name not in ACTION_TOPICS:
        print(f"Unknown action name: {action_name}")
        sys.exit(1)

    topic = ACTION_TOPICS[action_name]
    node = DrinkActionClientNode()
    node.send_goal(topic, request_id)

    try:
        while rclpy.ok() and not node._goal_done:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()