#!/usr/bin/env python3
"""
Mock Web Interface

This script mimics the real WebApp.
It ONLY publishes messages to the robot via:

    Topic:  WebAppComm
    Type:   std_msgs/String
    Data:   JSON string

Run this while your real robot-side script is running.

Commands:
    sip        -> request drink
    transfer   -> confirm drink transfer
    finish     -> finish/reset session
    jump       -> send navigation jump
    quit
"""

import rospy
import json
from std_msgs.msg import String


HELP = """
Commands:
  sip        -> {"state":"task_selection","status":"take_sip"}
  transfer   -> {"state":"post_drink_pickup","status":"drink_transfer"}
  finish     -> {"state":"task_selection","status":"finish_feeding"}
  jump       -> {"state":"task_selection","status":"jump"}
  raw <json> -> send custom JSON
  help
  quit
"""


def main():
    rospy.init_node("mock_web_interface")
    pub = rospy.Publisher("WebAppComm", String, queue_size=10)

    rospy.sleep(1.0)  # allow publisher to connect

    print("Mock Web Interface Started.")
    print(HELP)

    while not rospy.is_shutdown():
        try:
            cmd = input("[webapp] > ").strip()
        except (KeyboardInterrupt, EOFError):
            break

        if cmd == "quit":
            break

        if cmd == "help":
            print(HELP)
            continue

        if cmd == "sip":
            msg = {"state": "task_selection", "status": "take_sip"}

        elif cmd == "transfer":
            msg = {"state": "post_drink_pickup", "status": "drink_transfer"}

        elif cmd == "finish":
            msg = {"state": "task_selection", "status": "finish_feeding"}

        elif cmd == "jump":
            msg = {"state": "task_selection", "status": "jump"}

        elif cmd.startswith("raw "):
            raw_json = cmd[4:].strip()
            try:
                json.loads(raw_json)
                pub.publish(String(raw_json))
                print("Sent:", raw_json)
            except json.JSONDecodeError as e:
                print("Invalid JSON:", e)
            continue

        else:
            print("Unknown command. Type 'help'.")
            continue

        payload = json.dumps(msg)
        pub.publish(String(payload))
        print("Sent:", payload)

    print("Mock Web Interface Stopped.")


if __name__ == "__main__":
    main()