'''
Runs a client-side (run on compute machine) watchdog for the robot's intended functionality.
It validates the following:
1. All sensors are streaming correctly.
2. All sensor outputs are within the expected range.
    a. The camera perception outputs are within the expected range (ToDo)
    b. The robot's current state is not near collision.
If any of the above is not true, the watchdog will return the corresponding AnomalyStatus.
'''

import rclpy
from rclpy.node import Node
import numpy as np
import time
from enum import Enum
import queue
import signal
import sys

from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Bool

import threading
import time
import numpy as np
from pathlib import Path

from rammp.control.robot_controller.arm_interface import ArmInterface, ArmManager, NUC_HOSTNAME, ARM_RPC_PORT, RPC_AUTHKEY

CAMERA_FREQUENCY_THRESHOLD = 10 # expected is 30 Hz
COLLISION_FREE_FREQUENCY_THRESHOLD = 100 # expected is 350 Hz (empirical)

WATCHDOG_RUN_FREQUENCY = 1000

from rammp.safety.utils import PeekableQueue, AnomalyStatus

class WatchDog(Node):
    def __init__(self):
        super().__init__('WatchDog')

        # Register ArmInterface (no lambda needed on the client-side)
        ArmManager.register("ArmInterface")

        # Client setup
        self.manager = ArmManager(address=(NUC_HOSTNAME, ARM_RPC_PORT), authkey=RPC_AUTHKEY)
        self.manager.connect()

        # This will now use the single, shared instance of ArmInterface
        self._arm_interface = self.manager.ArmInterface()

        queue_size = 1000
        self.camera_info_sub = self.create_subscription(CameraInfo, "/camera/color/camera_info", self.cameraCallback, queue_size)
        self.camera_timestamps = PeekableQueue()

        self.camera_unexpected_sub = self.create_subscription(Bool, "/head_perception/unexpected", self.cameraUnexpectedCallback, queue_size)
        self.camera_unexpected = False

        self.collision_free_sub = self.create_subscription(Bool, '/collision_free', self.collisionFreeCallback, queue_size)
        self.collision_free_timestamps = PeekableQueue()
        self.collision_free_unexpected = False

        self.watchdog_status_pub = self.create_publisher(Bool, "/watchdog_status", 1)

        self.execution_log_path = Path(__file__).parent.parent / "integration" / "log" / "execution_log.txt"

        self.disable_collision_sensor_pub = self.create_publisher(Bool, "/disable_collision_sensor", 1)

        self.second_counter = 0
        time.sleep(5.0) # Wait for all queues to fill up / collision monitor to start

        # make sure collision is enabled
        self.disable_collision_sensor_pub.publish(Bool(data=False))
        print("Initialized.")

    def cameraCallback(self, msg):
        self.camera_timestamps.put(time.time())

    def cameraUnexpectedCallback(self, msg):
        self.camera_unexpected = msg.data

    def collisionFreeCallback(self, msg):

        self.collision_free_timestamps.put(time.time())
        if not msg.data:
            self.collision_free_unexpected = True

    def check_status(self):
        self.second_counter += 1
        self._arm_interface.is_alive()
        anomaly = AnomalyStatus.NO_ANOMALY
        start_time = time.time()
        frequencies = []
        for _queue, _threshold, _anomaly in [(self.camera_timestamps, CAMERA_FREQUENCY_THRESHOLD, AnomalyStatus.CAMERA_FREQUENCY),
                                            (self.collision_free_timestamps, COLLISION_FREE_FREQUENCY_THRESHOLD, AnomalyStatus.COLLISION_FREE_FREQUENCY)]:
            while _queue.peek() < start_time - 1.0:
                _queue.get()
            queue_size = _queue.qsize()
            if queue_size < _threshold:
                print(f"Frequency: {queue_size} for {_anomaly}")
                self.get_logger().info(f"Frequency: {queue_size} for {_anomaly}")
                anomaly = _anomaly
                break
            frequencies.append(queue_size)

        if self.second_counter == WATCHDOG_RUN_FREQUENCY:
            print("Watchdog running at expected frequency.")
            print(f"Frequencies:  Camera: {frequencies[0]}, Collision Free: {frequencies[1]}")
            self.second_counter = 0

        for _unexpected, _anomaly in [
                                    (self.camera_unexpected, AnomalyStatus.CAMERA_UNEXPECTED),
                                    (self.collision_free_unexpected, AnomalyStatus.COLLISION_FREE_UNEXPECTED)]:
            if _unexpected:
                print(f"Unexpected: {_anomaly}")
                self.get_logger().info(f"Unexpected: {_anomaly}")
                anomaly = _anomaly
                break

        if anomaly != AnomalyStatus.NO_ANOMALY:
            self._arm_interface.emergency_stop()
            print(f"AnomalyStatus detected: {anomaly}")
            self.get_logger().info(f"AnomalyStatus detected: {anomaly}")
            with open(self.execution_log_path, 'a') as f:
                f.write(f"Anomaly Detected: {AnomalyStatus.get_error_message(anomaly)}\n")

        self.watchdog_status_pub.publish(Bool(data=anomaly == AnomalyStatus.NO_ANOMALY))
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
            time.sleep(max(0, 1.0/WATCHDOG_RUN_FREQUENCY - (end_time - start_time)))

if __name__ == '__main__':

    rclpy.init()
    watchdog = WatchDog()
    watchdog.run()
    watchdog.destroy_node()
    rclpy.shutdown()
