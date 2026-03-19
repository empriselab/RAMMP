'''
Entrypoint for controlling the robot arm on compute machine. Additionally runs two important threads:
1. A thread that checks no safety anomalies have occurred using the watchdog
2. A thread that publishes joint states to ROS
'''

import threading
import time
import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool
    from geometry_msgs.msg import Pose
    RCLPY_IMPORTED = True
except ModuleNotFoundError as e:
    # print(f"ROS not imported: {e}")
    RCLPY_IMPORTED = False

from rammp.control.robot_controller.arm_interface import ArmInterface, ArmManager, NUC_HOSTNAME, ARM_RPC_PORT, RPC_AUTHKEY
from rammp.control.robot_controller.command_interface import KinovaCommand, JointTrajectoryCommand, JointCommand, CartesianCommand, OpenGripperCommand, CloseGripperCommand
# from rammp.safety.watchdog import WATCHDOG_MONITOR_FREQUENCY, PeekableQueue

from rammp.control.robot_controller.maintain_home_orientation import MaintainHomeOrientation

class ArmInterfaceClient:
    def __init__(self, node: "Node" = None):

        assert RCLPY_IMPORTED, "ROS is required to run on the real robot"

        self._node = node

        # make sure watchdog is running
        print("Waiting for Watchdog status...")
        if self._node is not None:
            # Wait for a message on /watchdog_status
            self._watchdog_received = False
            self._watchdog_sub = self._node.create_subscription(
                Bool, "/watchdog_status", self._watchdog_callback, 10
            )
            deadline = time.time() + 30.0
            while not self._watchdog_received and time.time() < deadline:
                rclpy.spin_once(self._node, timeout_sec=0.1)
            self._node.destroy_subscription(self._watchdog_sub)
            if not self._watchdog_received:
                raise TimeoutError("Watchdog status not received within 30s")
        print("Watchdog is running, continuing...")

        # Register ArmInterface (no lambda needed on the client-side)
        ArmManager.register("ArmInterface")

        # Client setup
        self.manager = ArmManager(address=(NUC_HOSTNAME, ARM_RPC_PORT), authkey=RPC_AUTHKEY)
        self.manager.connect()

        # This will now use the single, shared instance of ArmInterface
        self._arm_interface = self.manager.ArmInterface()

        self.maintain_home_orientation = MaintainHomeOrientation(self._arm_interface, node=node)

    def _watchdog_callback(self, msg):
        self._watchdog_received = True

    def start_maintain_home_orientation(self):
        self.maintain_home_orientation.start()

    def stop_maintain_home_orientation(self):
        self.maintain_home_orientation.stop()

    def get_state(self):
        return self._arm_interface.get_state()

    def get_speed(self):
        return self._arm_interface.get_speed()

    def set_speed(self, speed: str):
        assert speed in ["low", "medium", "high"], "Speed must be one of 'low', 'medium', 'high'"
        self._arm_interface.set_speed(speed)
        time.sleep(1.0) # Make sure the arm has time to change speed

    def set_tool(self, tool: str):
        self._arm_interface.set_tool(tool)

    def execute_command(self, cmd: KinovaCommand) -> None:

        if cmd.__class__.__name__ == "JointTrajectoryCommand":
            return self._arm_interface.set_joint_trajectory(cmd.traj)

        if cmd.__class__.__name__ == "JointCommand":
            joint_command_pos = cmd.pos
            if isinstance(joint_command_pos, np.ndarray):
                joint_command_pos = joint_command_pos.tolist()  # Convert to a list if it's a NumPy array
            return self._arm_interface.set_joint_position(joint_command_pos)

        if cmd.__class__.__name__ == "CartesianCommand":
            return self._arm_interface.set_ee_pose(cmd.pos, cmd.quat)

        if cmd.__class__.__name__ == "OpenGripperCommand":
            return self._arm_interface.open_gripper()

        if cmd.__class__.__name__ == "CloseGripperCommand":
            return self._arm_interface.close_gripper()

        raise NotImplementedError(f"Unrecognized command: {cmd}")

if __name__ == "__main__":

    rclpy.init()
    node = rclpy.create_node("arm_interface_client")
    arm_client_interface = ArmInterfaceClient(node=node)
    print("Current State:", arm_client_interface.get_state())

    run_commands = input("Press 'y' to run commands")

    if run_commands != "y":
        node.destroy_node()
        rclpy.shutdown()
        exit()

    above_inside_handle_pose = [-0.31010666489601135, -0.018085628747940063, 0.1062610810995102, 0.02322167404813943, 0.7180926934794382, 0.6955542280407613, 0.0028201561070785135]
    arm_client_interface.execute_command(CartesianCommand(pos=above_inside_handle_pose[:3], quat=above_inside_handle_pose[3:]))

    outside_handle_pose = [-0.31010666489601135, -0.118085628747940063, 0.1062610810995102, 0.02322167404813943, 0.7180926934794382, 0.6955542280407613, 0.0028201561070785135]
    arm_client_interface.execute_command(CartesianCommand(pos=outside_handle_pose[:3], quat=outside_handle_pose[3:]))

    node.destroy_node()
    rclpy.shutdown()
