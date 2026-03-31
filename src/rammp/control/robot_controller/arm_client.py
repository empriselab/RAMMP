#!/usr/bin/env python3

"""
ROS 2 Humble arm client for Cornell position control.

Supported commands:
- JointCommand           -> publishes to /arm/cornell/joint_position
- CartesianCommand       -> publishes to /arm/cornell/cartesian_pose
- OpenGripperCommand     -> calls /arm/open_gripper
- CloseGripperCommand    -> calls /arm/close_gripper

Notes:
- This version does NOT inherit from Node.
- A parent ROS 2 node must be passed in and spun by the application.
- Cornell position commands are only accepted by the arm control node in
  ORDER_DRINK and DRINKING modes.
"""

from typing import Optional
import time

from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from pybullet_helpers.geometry import Pose

from rammp.control.robot_controller.command_interface import (
    KinovaCommand,
    JointCommand,
    CartesianCommand,
    OpenGripperCommand,
    CloseGripperCommand,
)


class ArmInterfaceClient:
    def __init__(self, node: Node) -> None:
        self.node = node

        self.joint_pub = self.node.create_publisher(
            JointState,
            "/arm/cornell/joint_position",
            10,
        )

        self.cartesian_pub = self.node.create_publisher(
            PoseStamped,
            "/arm/cornell/cartesian_pose",
            10,
        )

        self.open_gripper_client = self.node.create_client(
            Trigger,
            "/arm/open_gripper",
        )

        self.close_gripper_client = self.node.create_client(
            Trigger,
            "/arm/close_gripper",
        )

        # State cache
        self._latest_joint_state: Optional[JointState] = None
        self._latest_ee_pose: Optional[Pose] = None

        # State subscribers
        self.joint_state_sub = self.node.create_subscription(
            JointState,
            "/arm/joint_states",
            self._joint_state_callback,
            10,
        )

        self.ee_pose_sub = self.node.create_subscription(
            PoseStamped,
            "/arm/ee_pose",
            self._ee_pose_callback,
            10,
        )

    def _joint_state_callback(self, msg: JointState) -> None:
        self._latest_joint_state = msg

    def _ee_pose_callback(self, msg: PoseStamped) -> None:
        self._latest_ee_pose = Pose(
            position=(
                float(msg.pose.position.x),
                float(msg.pose.position.y),
                float(msg.pose.position.z),
            ),
            orientation=(
                float(msg.pose.orientation.x),
                float(msg.pose.orientation.y),
                float(msg.pose.orientation.z),
                float(msg.pose.orientation.w),
            ),
        )

    def wait_until_ready(self, timeout_sec: float = 5.0) -> None:
        if not self.open_gripper_client.wait_for_service(timeout_sec=timeout_sec):
            self.node.get_logger().warn("/arm/open_gripper not available yet")

        if not self.close_gripper_client.wait_for_service(timeout_sec=timeout_sec):
            self.node.get_logger().warn("/arm/close_gripper not available yet")

        start_time = time.time()
        while time.time() - start_time <= timeout_sec:
            if self._latest_joint_state is not None:
                return
            time.sleep(0.05)

        self.node.get_logger().warn("/arm/joint_states not available yet")

    def get_state(self) -> dict[str, Optional[object]]:
        return {
            "joint_state": self._latest_joint_state,
            "ee_pose": self._latest_ee_pose,
        }

    def execute_command(self, cmd: KinovaCommand) -> bool:
        if isinstance(cmd, JointCommand):
            return self._send_joint_command(cmd)

        if isinstance(cmd, CartesianCommand):
            return self._send_cartesian_command(cmd)

        if isinstance(cmd, OpenGripperCommand):
            return self._call_trigger(self.open_gripper_client, "/arm/open_gripper")

        if isinstance(cmd, CloseGripperCommand):
            return self._call_trigger(self.close_gripper_client, "/arm/close_gripper")

        raise NotImplementedError(f"Unrecognized command: {cmd}")

    def _send_joint_command(
        self,
        cmd: JointCommand,
        timeout_sec: float = 15.0,
        position_tolerance: float = 0.02,
    ) -> bool:
        if len(cmd.pos) == 0:
            raise ValueError("JointCommand.pos cannot be empty")

        target = [float(x) for x in cmd.pos]

        msg = JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.position = target

        self.joint_pub.publish(msg)
        self.node.get_logger().info(
            f"Published JointCommand with {len(target)} joints"
        )

        start_time = time.time()

        while time.time() - start_time <= timeout_sec:
            joint_state = self._latest_joint_state

            if joint_state is not None and self._joint_goal_reached(
                joint_state=joint_state,
                target=target,
                tolerance=position_tolerance,
            ):
                self.node.get_logger().info(
                    f"JointCommand reached goal within tolerance {position_tolerance}"
                )
                return True

            time.sleep(0.05)

        self.node.get_logger().warn("Timed out waiting for JointCommand to reach goal")
        return False

    def _send_cartesian_command(
        self,
        cmd: CartesianCommand,
        timeout_sec: float = 15.0,
        pose_tolerance: float = 0.01,
    ) -> bool:
        if len(cmd.pos) != 3:
            raise ValueError("CartesianCommand.pos must have length 3: [x, y, z]")
        if len(cmd.quat) != 4:
            raise ValueError(
                "CartesianCommand.quat must have length 4: [qx, qy, qz, qw]"
            )

        target_pose = Pose(
            position=(
                float(cmd.pos[0]),
                float(cmd.pos[1]),
                float(cmd.pos[2]),
            ),
            orientation=(
                float(cmd.quat[0]),
                float(cmd.quat[1]),
                float(cmd.quat[2]),
                float(cmd.quat[3]),
            ),
        )

        msg = PoseStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        msg.pose.position.x = target_pose.position[0]
        msg.pose.position.y = target_pose.position[1]
        msg.pose.position.z = target_pose.position[2]

        msg.pose.orientation.x = target_pose.orientation[0]
        msg.pose.orientation.y = target_pose.orientation[1]
        msg.pose.orientation.z = target_pose.orientation[2]
        msg.pose.orientation.w = target_pose.orientation[3]

        self.cartesian_pub.publish(msg)
        self.node.get_logger().info("Published CartesianCommand")

        start_time = time.time()

        while time.time() - start_time <= timeout_sec:
            ee_pose = self._latest_ee_pose

            if ee_pose is not None and self._cartesian_goal_reached(
                current_pose=ee_pose,
                target_pose=target_pose,
                atol=pose_tolerance,
            ):
                self.node.get_logger().info(
                    f"CartesianCommand reached goal within tolerance {pose_tolerance}"
                )
                return True

            time.sleep(0.05)

        self.node.get_logger().warn(
            "Timed out waiting for CartesianCommand to reach goal"
        )
        return False

    def _joint_goal_reached(
        self,
        joint_state: JointState,
        target: list[float],
        tolerance: float,
    ) -> bool:
        if len(joint_state.position) < len(target):
            return False

        current = list(joint_state.position[: len(target)])
        max_err = max(abs(c - t) for c, t in zip(current, target))
        return max_err <= tolerance

    def _cartesian_goal_reached(
        self,
        current_pose: Pose,
        target_pose: Pose,
        atol: float,
    ) -> bool:
        return current_pose.allclose(target_pose, atol=atol)

    def _call_trigger(
        self,
        client,
        service_name: str,
        timeout_sec: float = 5.0,
    ) -> bool:
        if not client.wait_for_service(timeout_sec=timeout_sec):
            self.node.get_logger().error(f"Service {service_name} not available")
            return False

        request = Trigger.Request()
        future = client.call_async(request)

        start_time = time.time()
        while time.time() - start_time <= timeout_sec:
            if future.done():
                break
            time.sleep(0.05)

        if not future.done():
            self.node.get_logger().error(f"Timed out calling {service_name}")
            return False

        response = future.result()
        if response is None:
            self.node.get_logger().error(f"No response from {service_name}")
            return False

        if response.success:
            self.node.get_logger().info(
                f"{service_name} succeeded: {response.message}"
            )
        else:
            self.node.get_logger().warn(
                f"{service_name} failed: {response.message}"
            )

        return bool(response.success)