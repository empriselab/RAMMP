#!/usr/bin/env python3
import math
import time
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from sensor_msgs.msg import Imu
from tf.transformations import (
    quaternion_inverse,
    quaternion_multiply,
    euler_from_quaternion,
)


class MaintainHomeOrientation:
    HOME_POS = [
        0.0,
        0.26191187306569164,
        -3.1415742777782714,
        -2.269018308753582,
        -1.1185276577840852e-05,
        0.9598948696060562,
        1.5707649014940337,
    ]

    Q_IMU_CL = quaternion_inverse([-0.5, 0.5, -0.5, 0.5])
    DEADBAND = math.radians(1.0) # 1 degree
    ALPHA = 0.8

    def __init__(self, arm_interface):
        """
        arm_interface: ArmInterface
        """
        self.arm_interface = arm_interface

        self.roll = 0.0
        self.pitch = 0.0
        self.roll_ref = None
        self.pitch_ref = None

        self._active = False
        self._timer = None

        self._imu_sub = rospy.Subscriber(
            "/imu/data", Imu, self._imu_cb, queue_size=50
        )

        self._marker_pub = rospy.Publisher(
            "/gravity_marker",
            Marker,
            queue_size=1
        )


    # ----------------------------
    # IMU callback
    # ----------------------------
    def _imu_cb(self, msg: Imu):
        q = msg.orientation
        q_g_imu = [q.x, q.y, q.z, q.w]

        q_g_cl = quaternion_multiply(q_g_imu, self.Q_IMU_CL)
        r, p, _ = euler_from_quaternion(q_g_cl)

        # Publish gravity arrow
        self.publish_gravity_marker(q_g_cl)

        # latch reference once when active
        if self._active and self.roll_ref is None:
            self.roll_ref = r
            self.pitch_ref = p
            rospy.loginfo(
                "IMU reference locked: roll=%.2f° pitch=%.2f°",
                math.degrees(r),
                math.degrees(p),
            )

        if self.roll_ref is not None:
            self.roll = r - self.roll_ref
            self.pitch = p - self.pitch_ref
        
            # print("--- IMU Callback ---")
            # print(f"IMU Readings | roll: {math.degrees(self.roll):.2f}° | pitch: {math.degrees(self.pitch):.2f}°")
            # print(f"Reference   | roll_ref: {math.degrees(self.roll_ref):.2f}° | pitch_ref: {math.degrees(self.pitch_ref):.2f}°")
            # print(f"Raw IMU    | roll: {math.degrees(r):.2f}° | pitch: {math.degrees(p):.2f}°")
            # print("---------------------")

    def publish_gravity_marker(self, q_g_cl):
        """
        Publish gravity direction as an arrow marker
        """

        # Gravity direction in local frame
        v = [0.0, 0.0, -1.0]
        q_vec = [v[0], v[1], v[2], 0.0]
        q_inv = quaternion_inverse(q_g_cl)
        v_rot = quaternion_multiply(
            quaternion_multiply(q_g_cl, q_vec),
            q_inv
        )
        g_local = v_rot[:3]

        marker = Marker()
        marker.header.frame_id = "base_link"   # or imu_link / world
        marker.header.stamp = rospy.Time.now()
        marker.ns = "gravity"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # Arrow start & end
        p0 = Point(0.0, 0.0, 0.0)
        scale = 0.5  # arrow length
        p1 = Point(
            scale * g_local[0],
            scale * g_local[1],
            scale * g_local[2],
        )

        marker.points = [p0, p1]

        # Arrow thickness
        marker.scale.x = 0.03  # shaft diameter
        marker.scale.y = 0.06  # head diameter
        marker.scale.z = 0.1   # head length

        # Color (red)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.lifetime = rospy.Duration(0)

        self._marker_pub.publish(marker)


    # ----------------------------
    # Control loop (Timer)
    # ----------------------------
    def _control_cb(self, event):
        if not self._active:
            return

        if abs(self.roll) < self.DEADBAND and abs(self.pitch) < self.DEADBAND:
            rospy.loginfo("Within deadband | roll=%.2f° pitch=%.2f°", math.degrees(self.roll), math.degrees(self.pitch))
            return

        # print("Waiting for 10 secs...")
        # time.sleep(10.0) # delay to ensure stability
        # input("Press enter to correct orientation...")
        # cmd = self.HOME_POS.copy()
        cmd = self.arm_interface.get_state()['position'].tolist()
        print("CURRENT POS:", cmd)
        cmd[5] = cmd[5] + self.ALPHA * self.pitch
        cmd[6] = cmd[6] + self.ALPHA * self.roll
        print("Commanded POS:", cmd)

        rospy.loginfo(
            "Correcting | roll=%.1f° %.1f radians | pitch=%.1f° %.1f radians",
            math.degrees(self.roll), 
            self.roll,
            math.degrees(self.pitch),
            self.pitch,
        )

        self.arm_interface.set_joint_position(cmd)

    # ----------------------------
    # Public API
    # ----------------------------
    def start(self):
        """
        Start maintaining upright orientation (non-blocking).
        """
        if self._active:
            return

        rospy.loginfo("Starting MaintainHomeOrientation")

        self.roll_ref = None
        self.pitch_ref = None
        self.roll = 0.0
        self.pitch = 0.0

        # move to home once
        self.arm_interface.set_speed("low")
        self.arm_interface.set_joint_position(self.HOME_POS)
        time.sleep(2.0)


        self._active = True

        # input("Press enter to correct orientation...")

        # rospy.loginfo(
        #     "Correcting | roll=%.1f° %.1f radians | pitch=%.1f° %.1f radians",
        #     math.degrees(self.roll), 
        #     self.roll,
        #     math.degrees(self.pitch),
        #     self.pitch,
        # )

        # self.arm_interface.set_joint_position(cmd)
        self._timer = rospy.Timer(
            rospy.Duration(0.5), self._control_cb
        )  # 2 Hz

        # print('Corrected Position: ', self.arm_interface.get_state()['position'])

    def stop(self):
        """
        Stop maintaining upright orientation.
        """
        if not self._active:
            return

        rospy.loginfo("Stopping MaintainHomeOrientation")

        self._active = False
        if self._timer is not None:
            self._timer.shutdown()
            self._timer = None

        self.arm_interface.set_speed("medium")
        time.sleep(5.0) # Allow time to change speed
