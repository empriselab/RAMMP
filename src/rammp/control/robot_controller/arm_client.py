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

from rammp.control.robot_controller.maintain_home_orientation import MaintainHomeOrientation

class ArmInterfaceClient:
    def __init__(self):

        assert ROSPY_IMPORTED, "ROS is required to run on the real robot"

        # make sure watchdog is running
        print("Waiting for Watchdog status...")
        rospy.wait_for_message("/watchdog_status", Bool)
        print("Watchdog is running, continuing...")

        # Register ArmInterface (no lambda needed on the client-side)
        ArmManager.register("ArmInterface")

        # Client setup
        self.manager = ArmManager(address=(NUC_HOSTNAME, ARM_RPC_PORT), authkey=RPC_AUTHKEY)
        self.manager.connect()

        # This will now use the single, shared instance of ArmInterface
        self._arm_interface = self.manager.ArmInterface()

        self.maintain_home_orientation = MaintainHomeOrientation(self._arm_interface)

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

    rospy.init_node("arm_interface_client", anonymous=True)
    arm_client_interface = ArmInterfaceClient()
    print("Current State:", arm_client_interface.get_state())

    run_commands = input("Press 'y' to run commands")

    if run_commands != "y":
        exit()

    arm_client_interface.start_maintain_home_orientation()
    input("Press enter to stop maintaining home orientation...")
    arm_client_interface.stop_maintain_home_orientation()

    # retract_pos = [0.0, -0.34903602299465675, -3.141591055693139, -2.0, 0.0, -0.872688061814757, 1.57075917569769]
    # arm_client_interface.execute_command(JointCommand(retract_pos))

    # home_pos = [0.0, 0.26191187306569164, -3.1415742777782714, -2.269018308753582, -1.1185276577840852e-05, 0.9598948696060562, 1.5707649014940337]
    # arm_client_interface.execute_command(JointCommand(home_pos))

    # # midpoint_pos = [2.2912525080624357, 0.730991513381838, 2.0830126187361424, -2.1737367965371632, 0.28532185799581516, -0.4648462461578422, -0.29495787389950756]
    # # arm_client_interface.execute_command(JointCommand(midpoint_pos))

    # before_transfer_pos = [-2.86554642, -1.61951779, -2.60986085, -1.37302839, 1.11779249, -1.18028264, 2.05515862]
    # arm_client_interface.execute_command(JointCommand(before_transfer_pos))

    # # drink_gaze_pos = [-0.004187021865822871, 0.6034579885210962, -3.1259047705564633, -2.3538005746884725, 0.01149092320739253, 1.3411586039000891, 1.6825233913747728]
    # # arm_client_interface.execute_command(JointCommand(drink_gaze_pos))