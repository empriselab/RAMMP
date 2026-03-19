'''
Runs a server-side (run on NUC) watchdog to ensure robot is not in a state of emergency stop (from the user / experimentor emergency stop button).
'''

import rclpy
from rclpy.node import Node
import numpy as np
import time
from enum import Enum
import queue
import signal
import sys
import os
import paramiko

import threading
import time
import numpy as np
from pathlib import Path

from std_msgs.msg import Bool

from rammp.control.robot_controller.arm_interface import ArmInterface, ArmManager, NUC_HOSTNAME, ARM_RPC_PORT, RPC_AUTHKEY

USER_ESTOP_FREQUENCY_THRESHOLD = 50 # expected is 60 Hz
EXPERIMENTOR_ESTOP_FREQUENCY_THRESHOLD = 50 # expected is 60 Hz

BULLDOG_RUN_FREQUENCY = 1000

from rammp.safety.utils import PeekableQueue, AnomalyStatus

class BullDog(Node):
    def __init__(self):
        super().__init__('BullDog')
        print("BullDog awakening...")
        # Register ArmInterface (no lambda needed on the client-side)
        ArmManager.register("ArmInterface")

        # Client setup
        self.manager = ArmManager(address=(NUC_HOSTNAME, ARM_RPC_PORT), authkey=RPC_AUTHKEY)
        self.manager.connect()

        # This will now use the single, shared instance of ArmInterface
        self._arm_interface = self.manager.ArmInterface()

        queue_size = 1000
        self.user_emergency_stop_sub = self.create_subscription(Bool, '/user_estop', self.userEmergencyStopCallback, queue_size)
        self.user_emergency_stop_timestamps = PeekableQueue()
        self.user_emergency_stop_pressed = False

        self.experimentor_emergency_stop_sub = self.create_subscription(Bool, '/experimentor_estop', self.experimentorEmergencyStopCallback, queue_size)
        self.experimentor_emergency_stop_timestamps = PeekableQueue()
        self.experimentor_emergency_stop_pressed = False

        self.bulldog_status_pub = self.create_publisher(Bool, '/bulldog_status', 1)

        # Path is hardcoded because emprise uses two machines,
        # isacc for compute and nuc for robot control,
        # and we need to transmit logs from nuc (where bulldog runs) to isacc
        self.remote_execution_log_path = "/home/isacc/deployment_ws/src/rammp/src/rammp/integration/log/nuc_execution_log.txt"
        hostname = "192.168.1.12"
        username = "isacc"

        # Get the password from the environment variable
        password = os.getenv('ISACC_PASSWORD')
        if not password:
            print("Error: The environment variable 'ISACC_PASSWORD' must be set.")
            sys.exit(1)

        try:
            # Initialize SSH client
            self.client = paramiko.SSHClient()
            self.client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

            # Connect to the server using the password from the environment variable
            self.client.connect(hostname, username=username, password=password)
        except paramiko.AuthenticationException:
            print("Authentication failed. Check your username and password.")
        except paramiko.SSHException as e:
            print(f"SSH error: {e}")
        except Exception as e:
            print(f"Unexpected error: {e}")

        self.second_counter = 0
        time.sleep(1.0)
        print("BullDog is guarding the robot.")

    def write_to_remote(self, anomaly_message):

        # Open SFTP session
        sftp = self.client.open_sftp()
        with sftp.file(self.remote_execution_log_path, 'a') as f:
            f.write(anomaly_message + '\n')

        sftp.close()

    def userEmergencyStopCallback(self, msg):

        self.user_emergency_stop_timestamps.put(time.time())
        if msg.data:
            self.user_emergency_stop_pressed = True

    def experimentorEmergencyStopCallback(self, msg):

        self.experimentor_emergency_stop_timestamps.put(time.time())
        if msg.data:
            self.experimentor_emergency_stop_pressed = True

    def check_status(self):
        self.second_counter += 1
        self._arm_interface.is_alive()
        anomaly = AnomalyStatus.NO_ANOMALY
        start_time = time.time()
        frequencies = []
        for _queue, _threshold, _anomaly in [(self.user_emergency_stop_timestamps, USER_ESTOP_FREQUENCY_THRESHOLD, AnomalyStatus.USER_ESTOP_FREQUENCY),
                                            (self.experimentor_emergency_stop_timestamps, EXPERIMENTOR_ESTOP_FREQUENCY_THRESHOLD, AnomalyStatus.EXPERIMENTOR_ESTOP_FREQUENCY)]:
            while _queue.peek() < start_time - 1.0:
                _queue.get()
            queue_size = _queue.qsize()
            if queue_size < _threshold:
                print(f"Frequency: {queue_size} for {_anomaly}")
                self.get_logger().info(f"Frequency: {queue_size} for {_anomaly}")
                anomaly = _anomaly
                break
            frequencies.append(queue_size)

        if self.second_counter == BULLDOG_RUN_FREQUENCY:
            print("Bulldog running at expected frequency.")
            print(f"Frequencies: User EStop: {frequencies[0]}, Experimentor EStop: {frequencies[1]}")
            self.second_counter = 0

        for _unexpected, _anomaly in [(self.user_emergency_stop_pressed, AnomalyStatus.USER_ESTOP_PRESSED),
                                    (self.experimentor_emergency_stop_pressed, AnomalyStatus.EXPERIMENTOR_ESTOP_PRESSED)]:
            if _unexpected:
                print(f"Unexpected: {_anomaly}")
                self.get_logger().info(f"Unexpected: {_anomaly}")
                anomaly = _anomaly
                break

        if anomaly != AnomalyStatus.NO_ANOMALY:
            self._arm_interface.emergency_stop()
            print(f"AnomalyStatus detected: {anomaly}")
            self.get_logger().info(f"AnomalyStatus detected: {anomaly}")
            self.write_to_remote(f"Anomaly Detected: {AnomalyStatus.get_error_message(anomaly)}")

        self.bulldog_status_pub.publish(Bool(data=anomaly == AnomalyStatus.NO_ANOMALY))
        return anomaly

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0)
            start_time = time.time()
            status = self.check_status()
            if status != AnomalyStatus.NO_ANOMALY:
                break
            end_time = time.time()
            # print(f"Time taken: {end_time - start_time}")
            time.sleep(max(0, 1.0/BULLDOG_RUN_FREQUENCY - (end_time - start_time)))

if __name__ == '__main__':

    rclpy.init()
    bulldog = BullDog()
    bulldog.run()
    bulldog.destroy_node()
    rclpy.shutdown()
