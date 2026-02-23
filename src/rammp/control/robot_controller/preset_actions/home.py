'''
Entrypoint for controlling the robot arm on compute machine. Additionally runs two important threads:
1. A thread that checks no safety anomalies have occurred using the watchdog
2. A thread that publishes joint states to ROS
'''

import threading
import time
import numpy as np

try:
    import rospy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool
    from geometry_msgs.msg import Pose
    # from netft_rdt_driver.srv import String_cmd
    ROSPY_IMPORTED = True
except ModuleNotFoundError as e:
    # print(f"ROS not imported: {e}")
    ROSPY_IMPORTED = False

from rammp.control.robot_controller.arm_interface import ArmInterface, ArmManager, NUC_HOSTNAME, ARM_RPC_PORT, RPC_AUTHKEY
from rammp.control.robot_controller.command_interface import KinovaCommand, JointTrajectoryCommand, JointCommand, CartesianCommand, OpenGripperCommand, CloseGripperCommand
# from rammp.safety.watchdog import WATCHDOG_MONITOR_FREQUENCY, PeekableQueue


if __name__ == "__main__":

    assert ROSPY_IMPORTED, "ROS is required to run on the real robot"
    rospy.init_node("retract_action")

    # make sure watchdog is running
    print("Waiting for Watchdog status...")
    rospy.wait_for_message("/watchdog_status", Bool)
    print("Watchdog is running, moving to home configuration...")

    # Register ArmInterface (no lambda needed on the client-side)
    ArmManager.register("ArmInterface")

    # Client setup
    manager = ArmManager(address=(NUC_HOSTNAME, ARM_RPC_PORT), authkey=RPC_AUTHKEY)
    manager.connect()

    # This will now use the single, shared instance of ArmInterface
    arm_interface = manager.ArmInterface()

    home_pos = [0.0, 0.26191187306569164, -3.1415742777782714, -2.269018308753582, -1.1185276577840852e-05, 0.9598948696060562, 1.5707649014940337]
    arm_interface.set_joint_position(home_pos)