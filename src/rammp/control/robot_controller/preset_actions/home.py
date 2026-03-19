'''
Entrypoint for controlling the robot arm on compute machine. Additionally runs two important threads:
1. A thread that checks no safety anomalies have occurred using the watchdog
2. A thread that publishes joint states to ROS
'''

import rclpy
from std_msgs.msg import Bool

from rammp.control.robot_controller.arm_interface import ArmInterface, ArmManager, NUC_HOSTNAME, ARM_RPC_PORT, RPC_AUTHKEY
from rammp.control.robot_controller.command_interface import KinovaCommand, JointTrajectoryCommand, JointCommand, CartesianCommand, OpenGripperCommand, CloseGripperCommand
# from rammp.safety.watchdog import WATCHDOG_MONITOR_FREQUENCY, PeekableQueue


if __name__ == "__main__":

    rclpy.init()
    node = rclpy.create_node("home_action")

    # make sure watchdog is running
    print("Waiting for Watchdog status...")
    received = False
    def cb(msg):
        nonlocal received
        received = True
    sub = node.create_subscription(Bool, "/watchdog_status", cb, 10)
    while not received:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
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

    node.destroy_node()
    rclpy.shutdown()