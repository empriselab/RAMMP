#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_srvs.srv import SetBool
from cornell_feeding_interfaces.action import DrinkAction
from cornell_feeding_interfaces.msg import CupInfo


ACTION_TOPICS = {
    "pickup_and_order": "/arm/drink/pickup_and_order",
    "locate_cup": "/arm/drink/locate_cup",
    "perceive_cup": "/arm/drink/detection/enable",
    "grab_cup_from_table": "/arm/drink/grab_cup_from_table",
    "bring_cup_to_mouth": "/arm/drink/bring_cup_to_mouth",
    "home_cup": "/arm/drink/home_cup",
    "put_cup_back_to_holder": "/arm/drink/put_cup_back_to_holder",
}


class DrinkActionClientNode(Node):
    def __init__(self):
        super().__init__("drink_action_client")
        self._action_client = None
        self._service_client = None
        self._cup_sub = None
        self._goal_done = False

    def feedback_cb(self, feedback_msg):
        # In ROS 2, feedback callback receives a message wrapper with `.feedback`
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Feedback: {feedback.state}")

    def _cup_info_cb(self, msg: CupInfo):
        if msg.success:
            self.get_logger().info(f"CupInfo: pose={list(msg.pose)}")
        else:
            self.get_logger().info("CupInfo: no cup detected")

    def send_goal(self, topic: str, request_id: str, action_name: str):
        if action_name == "perceive_cup":
            enable = request_id.lower() not in ("false", "0", "off")
            self._service_client = self.create_client(SetBool, topic)
            self.get_logger().info(f"Waiting for service: {topic}")
            self._service_client.wait_for_service()

            if enable and self._cup_sub is None:
                self._cup_sub = self.create_subscription(
                    CupInfo, "/arm/drink/cup_info", self._cup_info_cb, 10
                )

            req = SetBool.Request()
            req.data = enable
            self.get_logger().info(f"{'Starting' if enable else 'Stopping'} cup handle stream")
            future = self._service_client.call_async(req)
            future.add_done_callback(self.service_result_cb)
            return

        action_type = DrinkAction
        self._action_client = ActionClient(self, action_type, topic)

        self.get_logger().info(f"Waiting for server: {topic}")
        self._action_client.wait_for_server()

        goal_msg = action_type.Goal()
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

    def service_result_cb(self, future):
        try:
            result = future.result()
            self.get_logger().info(
                f"Result: success={result.success} message={result.message}"
            )
            # If we just disabled streaming, we're done. If we enabled it,
            # keep spinning to receive CupInfo messages until Ctrl+C.
            if self._cup_sub is None:
                self._goal_done = True
        except Exception as exc:
            self.get_logger().error(f"Service call failed: {exc}")
            self._goal_done = True


def main():
    rclpy.init()

    if len(sys.argv) < 2:
        print(
            "Usage: ros2 run drink_actions_test drink_action_client "
            "<action_name> [request_id|true|false]"
        )
        sys.exit(1)

    action_name = sys.argv[1]
    request_id = sys.argv[2] if len(sys.argv) > 2 else "test_request"

    if action_name not in ACTION_TOPICS:
        print(f"Unknown action name: {action_name}")
        sys.exit(1)

    topic = ACTION_TOPICS[action_name]
    node = DrinkActionClientNode()
    node.send_goal(topic, request_id, action_name)

    try:
        while rclpy.ok() and not node._goal_done:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()