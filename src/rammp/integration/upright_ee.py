#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from scipy.spatial.transform import Rotation as R
import numpy as np
import time

from rammp.control.robot_controller.arm_client import ArmInterfaceClient
from rammp.control.robot_controller.command_interface import JointCommand

HOME_POS = [0.0, 0.26191187306569164, -3.1415742777782714, -2.269018308753582, -1.1185276577840852e-05, 0.9598948696060562, 1.5707649014940337]
# ROS2: use scipy instead of tf.transformations
_q_imu_cl = R.from_quat([-0.5, 0.5, -0.5, 0.5]).inv()
DEADBAND = math.radians(3.0)   # only correct if > 5 deg
ALPHA = 0.5                    # 1:1 correction

roll = 0.0
pitch = 0.0
roll_ref = None
pitch_ref = None
_node = None


def imu_cb(msg: Imu):
    global roll, pitch, roll_ref, pitch_ref

    q = msg.orientation
    q_g_imu = R.from_quat([q.x, q.y, q.z, q.w])

    # camera_link orientation w.r.t gravity
    q_g_cl = q_g_imu * _q_imu_cl
    r, p, _ = q_g_cl.as_euler('xyz')

    # latch reference once
    if roll_ref is None:
        roll_ref = r
        pitch_ref = p
        _node.get_logger().info(
            "IMU reference locked: roll=%.2f deg pitch=%.2f deg" %
            (math.degrees(roll_ref), math.degrees(pitch_ref))
        )

    # relative roll / pitch
    roll = r - roll_ref
    pitch = p - pitch_ref


if __name__ == "__main__":
    rclpy.init()
    _node = rclpy.create_node("camera_level_home_pose")

    robot = ArmInterfaceClient(node=_node)
    robot.execute_command(JointCommand(HOME_POS))
    set_home = True
    time.sleep(2)

    _node.create_subscription(Imu, "/imu/data", imu_cb, 50)

    rate = _node.create_rate(2)  # slow loop is intentional

    while rclpy.ok():
        rclpy.spin_once(_node, timeout_sec=0)
        # check deadband
        if abs(roll) < DEADBAND and abs(pitch) < DEADBAND:
            rate.sleep()
            continue

        # compute corrected pose from HOME (no accumulation)
        cmd = HOME_POS.copy()
        cmd[5] = HOME_POS[5] + ALPHA * pitch   # joint 6 -> pitch compensation
        cmd[6] = HOME_POS[6] + ALPHA * roll    # joint 7 -> roll compensation

        _node.get_logger().info(
            "Correcting | roll=%.1f deg pitch=%.1f deg" %
            (math.degrees(roll), math.degrees(pitch))
        )

        print("Sending command:", cmd)
        robot.execute_command(JointCommand(cmd))
        set_home = False

        rate.sleep()

    _node.destroy_node()
    rclpy.shutdown()
