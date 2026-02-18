#!/usr/bin/env python3
import math
import rospy
from sensor_msgs.msg import Imu
from tf.transformations import (
    quaternion_inverse,
    quaternion_multiply,
    euler_from_quaternion,
)
import time

from rammp.control.robot_controller.arm_client import ArmInterfaceClient
from rammp.control.robot_controller.command_interface import JointCommand

HOME_POS = [0.0, 0.26191187306569164, -3.1415742777782714, -2.269018308753582, -1.1185276577840852e-05, 0.9598948696060562, 1.5707649014940337]
q_imu_cl = quaternion_inverse([-0.5, 0.5, -0.5, 0.5])
DEADBAND = math.radians(3.0)   # only correct if > 5 deg
ALPHA = 0.5                    # 1:1 correction

roll = 0.0
pitch = 0.0
roll_ref = None
pitch_ref = None


def imu_cb(msg: Imu):
    global roll, pitch, roll_ref, pitch_ref

    q = msg.orientation
    q_g_imu = [q.x, q.y, q.z, q.w]

    # camera_link orientation w.r.t gravity
    q_g_cl = quaternion_multiply(q_g_imu, q_imu_cl)
    r, p, _ = euler_from_quaternion(q_g_cl)

    # 🔹 latch reference once
    if roll_ref is None:
        roll_ref = r
        pitch_ref = p
        rospy.loginfo(
            "IMU reference locked: roll=%.2f° pitch=%.2f°",
            math.degrees(roll_ref), math.degrees(pitch_ref)
        )

    # 🔹 relative roll / pitch
    roll = r - roll_ref
    pitch = p - pitch_ref


if __name__ == "__main__":
    rospy.init_node("camera_level_home_pose")

    robot = ArmInterfaceClient()
    robot.execute_command(JointCommand(HOME_POS))
    set_home = True
    time.sleep(2)

    rospy.Subscriber("/imu/data", Imu, imu_cb, queue_size=50)

    rate = rospy.Rate(2)  # slow loop is intentional

    while not rospy.is_shutdown():
        # check deadband
        if abs(roll) < DEADBAND and abs(pitch) < DEADBAND:
            # if not set_home:
                # robot.execute_command(JointCommand(HOME_POS))
                # set_home = True
            rate.sleep()
            continue

        # compute corrected pose from HOME (no accumulation)
        cmd = HOME_POS.copy()
        cmd[5] = HOME_POS[5] + ALPHA * pitch   # joint 6 → pitch compensation
        cmd[6] = HOME_POS[6] + ALPHA * roll    # joint 7 → roll compensation

        rospy.loginfo(
            "Correcting | roll=%.1f° pitch=%.1f°",
            math.degrees(roll), math.degrees(pitch)
        )

        print("Sending command:", cmd)
        robot.execute_command(JointCommand(cmd))
        set_home = False

        rate.sleep()
