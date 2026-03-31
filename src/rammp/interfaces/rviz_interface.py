from __future__ import annotations

import time

from pybullet_helpers.joint import JointPositions
from rclpy.node import Node
from sensor_msgs.msg import JointState
import tf2_ros

from rammp.simulation.scene_description import SceneDescription

class RVizInterface:
    def __init__(self, node: Node, scene_description: SceneDescription) -> None:
        self.node = node
        self.scene_description = scene_description

        self.sim_joint_publisher = self.node.create_publisher(
            JointState,
            "/sim/robot_joint_states",
            10,
        )

        self.static_transform_broadcaster = tf2_ros.StaticTransformBroadcaster(self.node)
        self.transform_broadcaster = tf2_ros.TransformBroadcaster(self.node)

        time.sleep(1.0)

        self.joint_state_update(self.scene_description.initial_joints)

    def joint_state_update(self, joints: JointPositions) -> None:
        msg = JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = [
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
            "joint_6",
            "joint_7",
            "finger_joint",
        ]
        msg.position = list(joints[:7]) + [0.0]
        self.sim_joint_publisher.publish(msg)

    def visualize_plan(self, plan) -> None:
        for sim_state in plan:
            self.joint_state_update(sim_state.robot_joints)
            time.sleep(0.1)