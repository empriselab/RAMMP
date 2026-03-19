#!/usr/bin/env python3
import math
import time
import threading

from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from sensor_msgs.msg import Imu
from scipy.spatial.transform import Rotation


def _quaternion_multiply(q1, q2):
    """Hamilton product of two quaternions [x, y, z, w]."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return [
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    ]


def _quaternion_inverse(q):
    """Inverse of a unit quaternion [x, y, z, w]."""
    return [-q[0], -q[1], -q[2], q[3]]


def _euler_from_quaternion(q):
    """Convert quaternion [x, y, z, w] to euler angles (roll, pitch, yaw)."""
    r = Rotation.from_quat(q)
    return r.as_euler('xyz')


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

    Q_IMU_CL = _quaternion_inverse([-0.5, 0.5, -0.5, 0.5])
    DEADBAND = math.radians(1.0)  # 1 degree
    ALPHA = 0.8

    def __init__(self, arm_interface, node: Node = None):
        """
        arm_interface: ArmInterface
        node: rclpy Node for creating subscriptions/publishers/timers
        """
        self.arm_interface = arm_interface
        self._node = node

        self.roll = 0.0
        self.pitch = 0.0
        self.roll_ref = None
        self.pitch_ref = None

        self._active = False
        self._timer = None

        if self._node is not None:
            self._imu_sub = self._node.create_subscription(
                Imu, "/imu/data", self._imu_cb, 50
            )
            self._marker_pub = self._node.create_publisher(
                Marker, "/gravity_marker", 1
            )
        else:
            self._imu_sub = None
            self._marker_pub = None

    def _log_info(self, msg):
        if self._node is not None:
            self._node.get_logger().info(msg)
        else:
            print(msg)

    # ----------------------------
    # IMU callback
    # ----------------------------
    def _imu_cb(self, msg: Imu):
        q = msg.orientation
        q_g_imu = [q.x, q.y, q.z, q.w]

        q_g_cl = _quaternion_multiply(q_g_imu, self.Q_IMU_CL)
        r, p, _ = _euler_from_quaternion(q_g_cl)

        # Publish gravity arrow
        self.publish_gravity_marker(q_g_cl)

        # latch reference once when active
        if self._active and self.roll_ref is None:
            self.roll_ref = r
            self.pitch_ref = p
            self._log_info(
                f"IMU reference locked: roll={math.degrees(r):.2f}° pitch={math.degrees(p):.2f}°"
            )

        if self.roll_ref is not None:
            self.roll = r - self.roll_ref
            self.pitch = p - self.pitch_ref

    def publish_gravity_marker(self, q_g_cl):
        """Publish gravity direction as an arrow marker."""
        if self._marker_pub is None:
            return

        v = [0.0, 0.0, -1.0]
        q_vec = [v[0], v[1], v[2], 0.0]
        q_inv = _quaternion_inverse(q_g_cl)
        v_rot = _quaternion_multiply(
            _quaternion_multiply(q_g_cl, q_vec),
            q_inv
        )
        g_local = v_rot[:3]

        marker = Marker()
        marker.header.frame_id = "base_link"
        if self._node is not None:
            marker.header.stamp = self._node.get_clock().now().to_msg()
        marker.ns = "gravity"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        p0 = Point(x=0.0, y=0.0, z=0.0)
        scale = 0.5
        p1 = Point(
            x=scale * g_local[0],
            y=scale * g_local[1],
            z=scale * g_local[2],
        )

        marker.points = [p0, p1]

        marker.scale.x = 0.03
        marker.scale.y = 0.06
        marker.scale.z = 0.1

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0

        self._marker_pub.publish(marker)

    # ----------------------------
    # Control loop (Timer)
    # ----------------------------
    def _control_cb(self):
        if not self._active:
            return

        if abs(self.roll) < self.DEADBAND and abs(self.pitch) < self.DEADBAND:
            self._log_info(f"Within deadband | roll={math.degrees(self.roll):.2f}° pitch={math.degrees(self.pitch):.2f}°")
            return

        cmd = self.arm_interface.get_state()['position'].tolist()
        print("CURRENT POS:", cmd)
        cmd[5] = cmd[5] + self.ALPHA * self.pitch
        cmd[6] = cmd[6] + self.ALPHA * self.roll
        print("Commanded POS:", cmd)

        self._log_info(
            f"Correcting | roll={math.degrees(self.roll):.1f}° {self.roll:.1f} radians "
            f"| pitch={math.degrees(self.pitch):.1f}° {self.pitch:.1f} radians"
        )

        self.arm_interface.set_joint_position(cmd)

    # ----------------------------
    # Public API
    # ----------------------------
    def start(self):
        """Start maintaining upright orientation (non-blocking)."""
        if self._active:
            return

        self._log_info("Starting MaintainHomeOrientation")

        self.roll_ref = None
        self.pitch_ref = None
        self.roll = 0.0
        self.pitch = 0.0

        # move to home once
        self.arm_interface.set_speed("low")
        self.arm_interface.set_joint_position(self.HOME_POS)
        time.sleep(2.0)

        self._active = True

        if self._node is not None:
            self._timer = self._node.create_timer(0.5, self._control_cb)
        else:
            # Fallback: run control loop in a thread
            self._stop_event = threading.Event()
            def _loop():
                while not self._stop_event.is_set():
                    self._control_cb()
                    self._stop_event.wait(0.5)
            self._timer_thread = threading.Thread(target=_loop, daemon=True)
            self._timer_thread.start()

    def stop(self):
        """Stop maintaining upright orientation."""
        if not self._active:
            return

        self._log_info("Stopping MaintainHomeOrientation")

        self._active = False
        if self._node is not None and self._timer is not None:
            self._timer.cancel()
            self._timer = None
        elif hasattr(self, '_stop_event'):
            self._stop_event.set()

        self.arm_interface.set_speed("medium")
        time.sleep(5.0)
