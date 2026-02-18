# This RPC server allows other processes to communicate with the Kinova arm
# low-level controller.

import queue
import time
import threading
from pathlib import Path

import numpy as np
from multiprocess.managers import BaseManager as MPBaseManager

RPC_AUTHKEY = b"secret-key"
NUC_HOSTNAME = "192.168.1.3"
ARM_RPC_PORT = 5000

class ArmInterface:
    def __init__(self, arm_instance):
        self.arm = arm_instance
        # self.arm.set_joint_limits(speed_limits=(7 * (30,)), acceleration_limits=(7 * (80,)))

        self.emergency_stop_active = False

        # log file
        self.log_file = Path(__file__).parent / "safety_log" / "arm_commands_log.txt"
        # clear log file and set time stamp
        with open(self.log_file, "w") as f:
            f.write(f"Log file created at {time.strftime('%Y-%m-%d %H:%M:%S')}\n") 

    def is_alive(self):
        return True

    def get_state(self):
        try:
            current_state = self.arm.get_state()
        except Exception as e:
            print(f"Error in get_state: {e}")
            # Re-raise a simplified exception to avoid pickling issues
            raise Exception(f"Error in get_state: {str(e)}") from None # suppress original exception

        return current_state

    def reset(self):
        # Go to home position
        print("Moving to home position")
        try:
            self.arm.home()
        except Exception as e:
            print(f"Error in reset: {e}")
            # Re-raise a simplified exception to avoid pickling issues
            raise Exception(f"Error in reset: {str(e)}") from None # suppress original exception

    def set_tool(self, tool: str):
        print(f"Setting tool to {tool}")
        try:
            self.arm.set_tool(tool)
        except Exception as e:
            print(f"Error in set_tool: {e}")
            # Re-raise a simplified exception to avoid pickling issues
            raise Exception(f"Error in set_tool: {str(e)}") from None # suppress original exception

    def set_speed(self, speed: str):
        """ speed: "low", "medium", "high" """
        assert speed in ["low", "medium", "high"], "Invalid speed"
        assert not self.emergency_stop_active, "Emergency stop is active"
        
        print(f"Setting speed to {speed}")
        try:
            self.arm.choose_from_speed_presets(speed)
        except Exception as e:
            print(f"Error in choose_from_speed_presets: {e}")
            # Re-raise a simplified exception to avoid pickling issues
            raise Exception(f"Error in choose_from_speed_presets: {str(e)}") from None
        
    def get_speed(self):
        assert not self.emergency_stop_active, "Emergency stop is active"

        try:
            arm_speed = self.arm.get_speed_preset()
        except Exception as e:
            print(f"Error in get_speed: {e}")
            # Re-raise a simplified exception to avoid pickling issues
            raise Exception(f"Error in get_speed: {str(e)}") from None

        return arm_speed

    def set_joint_position(self, command_pos):
        
        assert not self.emergency_stop_active, "Emergency stop is active"

        # save in log file
        with open(self.log_file, "a") as f:
            f.write(f"set_joint_position: {command_pos}\n")

        print(f"Received joint pos command: {command_pos}")

        try:
            self.arm.move_angular(command_pos)
        except Exception as e:
            print(f"Error in set_joint_position: {e}")
            # Re-raise a simplified exception to avoid pickling issues
            raise Exception(f"Error in set_joint_position: {str(e)}") from None # suppress original exception

    def set_joint_trajectory(self, trajectory_command):

        assert not self.emergency_stop_active, "Emergency stop is active"

        print(
            f"Received joint trajectory command with {len(trajectory_command)} waypoints"
        )

        try:
            self.arm.move_angular_trajectory(trajectory_command)
        except Exception as e:
            print(f"Error in set_joint_trajectory: {e}")
            # Re-raise a simplified exception to avoid pickling issues
            raise Exception(f"Error in set_joint_trajectory: {str(e)}") from None # suppress original exception

    def set_ee_pose(self, xyz, xyz_quat):

        assert not self.emergency_stop_active, "Emergency stop is active"

        # save in log file
        with open(self.log_file, "a") as f:
            f.write(f"set_ee_pose: {xyz}, {xyz_quat}\n")

        print(f"Received cartesian pose command: {xyz}, {xyz_quat}")

        try:
            self.arm.move_cartesian(xyz, xyz_quat)
        except Exception as e:
            print(f"Error in set_ee_pose: {e}")
            # Re-raise a simplified exception to avoid pickling issues
            raise Exception(f"Error in set_ee_pose: {str(e)}") from None # suppress original exception

    def set_gripper(self, gripper_pos):

        assert not self.emergency_stop_active, "Emergency stop is active"

        print(f"Received gripper pos command: {gripper_pos}")

        try:
            self.arm._gripper_position_command(gripper_pos)
        except Exception as e:
            print(f"Error in set_gripper: {e}")
            # Re-raise a simplified exception to avoid pickling issues
            raise Exception(f"Error in set_gripper: {str(e)}") from None # suppress original exception

    def open_gripper(self):

        assert not self.emergency_stop_active, "Emergency stop is active"

        print("Received open gripper command")

        try:
            self.arm.open_gripper()
        except Exception as e:
            print(f"Error in open_gripper: {e}")
            # Re-raise a simplified exception to avoid pickling issues
            raise Exception(f"Error in open_gripper: {str(e)}") from None # suppress original exception

    def close_gripper(self):

        assert not self.emergency_stop_active, "Emergency stop is active"

        print("Received close gripper command")

        try:
            self.arm.close_gripper()
        except Exception as e:
            print(f"Error in close_gripper: {e}")
            # Re-raise a simplified exception to avoid pickling issues
            raise Exception(f"Error in close_gripper: {str(e)}") from None # suppress original exception

    def close(self):
        print("Close arm command received")

        try:
            self.arm.stop() # Stop arm incase it is running
            print("Arm stopped")
            self.arm.disconnect()
            print("Arm disconnected")
        except Exception as e:
            print(f"Error in close: {e}")
            # Re-raise a simplified exception to avoid pickling issues
            raise Exception(f"Error in close: {str(e)}") from None # suppress original exception

    def retract(self):

        assert not self.emergency_stop_active, "Emergency stop is active"

        print("Received retract command")

        try:
            self.arm.retract()
        except Exception as e:
            print(f"Error in retract: {e}")
            # Re-raise a simplified exception to avoid pickling issues
            raise Exception(f"Error in retract: {str(e)}") from None # suppress original exception

    def emergency_stop(self):
        assert not self.emergency_stop_active, "Emergency stop is already active"

        # save in log file
        with open(self.log_file, "a") as f:
            f.write("emergency_stop\n")

        self.emergency_stop_active = True
        try:
            self.arm.stop()
        except Exception as e:
            print(f"Error in emergency_stop: {e}")
            # Re-raise a simplified exception to avoid pickling issues
            raise Exception(f"Error in emergency_stop: {str(e)}") from None # suppress original exception

        print("Emergency stop activated by user, will not take any more commands")

class ArmManager(MPBaseManager):
    pass
