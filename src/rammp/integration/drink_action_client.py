#!/usr/bin/env python3

import sys
import rospy
import actionlib
from drink_actions_test.msg import DrinkActionAction, DrinkActionGoal


ACTION_TOPICS = {
    "pickup_and_order": "/arm/drink/pickup_and_order",
    "grab_cup_from_table": "/arm/drink/grab_cup_from_table",
    "bring_cup_to_mouth": "/arm/drink/bring_cup_to_mouth",
    "home_cup": "/arm/drink/home_cup",
    "put_cup_back_to_holder": "/arm/drink/put_cup_back_to_holder",
}


def feedback_cb(feedback):
    rospy.loginfo("Feedback: %s", feedback.state)


def main():
    rospy.init_node("drink_action_client")

    if len(sys.argv) < 2:
        print("Usage: rosrun drink_actions_test drink_action_client.py <action_name> [request_id]")
        sys.exit(1)

    action_name = sys.argv[1]
    request_id = sys.argv[2] if len(sys.argv) > 2 else "test_request"

    if action_name not in ACTION_TOPICS:
        rospy.logerr("Unknown action name: %s", action_name)
        sys.exit(1)

    topic = ACTION_TOPICS[action_name]
    client = actionlib.SimpleActionClient(topic, DrinkActionAction)

    rospy.loginfo("Waiting for server: %s", topic)
    client.wait_for_server()

    goal = DrinkActionGoal()
    goal.request_id = request_id

    rospy.loginfo("Sending goal to %s", topic)
    client.send_goal(goal, feedback_cb=feedback_cb)
    client.wait_for_result()

    result = client.get_result()
    rospy.loginfo("Result: success=%s message=%s", result.success, result.message)


if __name__ == "__main__":
    main()