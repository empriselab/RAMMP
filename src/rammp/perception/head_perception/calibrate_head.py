"""Head-perception calibration node.

Records the reference the runtime head perception tracks against:
positions the user comfortably, holds the drink tip at their mouth, and
captures one frame. Run with:

    python -m rammp.perception.head_perception.calibrate_head --tool drink
"""

import argparse
import os

import numpy as np
import rclpy
from rclpy.node import Node

import rammp.simulation.scene_description as scene_description_module
from rammp.control.robot_controller.arm_client import ArmInterfaceClient
from rammp.interfaces.realsense_interface import RealSenseInterface
from rammp.perception.head_perception import head_geometry as hg
from rammp.perception.head_perception.mediapipe_perception import (
    MediaPipeHeadPerception,
    _CONFIG_DIR,
)


def _wait_for_camera(node: Node, realsense: RealSenseInterface) -> dict:
    node.get_logger().info("Waiting for camera data...")
    while realsense.get_camera_data()["rgb_image"] is None:
        rclpy.spin_once(node, timeout_sec=0.1)
    return realsense.get_camera_data()


def calibrate(tool: str, scene_config: str) -> None:
    rclpy.init()
    node = Node("head_calibration")

    realsense = RealSenseInterface(node)
    arm = ArmInterfaceClient(node=node)
    head_perception = MediaPipeHeadPerception()

    scene_config_path = os.path.join(
        os.path.dirname(scene_description_module.__file__),
        "configs",
        f"{scene_config}.yaml",
    )
    scene_description = scene_description_module.create_scene_description_from_config(
        scene_config_path
    )
    tool_frame_to_tip = scene_description.tool_frame_to_drink_tip.to_matrix()

    _wait_for_camera(node, realsense)

    input(
        f"\nPosition the user comfortably and hold the {tool} tip at their "
        f"mouth.\nPress Enter to capture the calibration frame..."
    )

    camera_data = realsense.get_camera_data()
    base_to_camera = realsense.get_base_to_camera_transform()
    if base_to_camera is None:
        node.get_logger().error("base->camera transform unavailable; aborting.")
        rclpy.shutdown()
        return

    arm_state = arm.get_state()
    ee_pose = arm_state["ee_pose"]
    if ee_pose is None:
        node.get_logger().error("End-effector pose unavailable; aborting.")
        rclpy.shutdown()
        return
    ee_pose_matrix = hg.pose_to_matrix(ee_pose.position, ee_pose.orientation)

    result = head_perception.rigid_landmark_points(
        camera_data["rgb_image"],
        camera_data["depth_image"],
        camera_data["camera_info"],
    )
    if result is None:
        node.get_logger().error("No face detected in the calibration frame.")
        rclpy.shutdown()
        return
    rigid_points, _landmarks_px, _jaw = result

    valid_count = int((~np.isnan(rigid_points).any(axis=1)).sum())
    node.get_logger().info(f"Captured {valid_count} valid rigid landmarks.")
    if valid_count < 60:
        node.get_logger().error(
            "Too few valid landmarks; improve lighting/framing and retry."
        )
        rclpy.shutdown()
        return

    reference_points, reference_head_frame, tool_tip_transform = hg.build_calibration(
        rigid_points, ee_pose_matrix, base_to_camera, tool_frame_to_tip
    )

    tool_dir = os.path.join(_CONFIG_DIR, tool)
    os.makedirs(tool_dir, exist_ok=True)
    np.save(os.path.join(tool_dir, "reference_landmarks_camera.npy"), reference_points)
    np.save(os.path.join(tool_dir, "reference_head_frame.npy"), reference_head_frame)
    np.save(os.path.join(tool_dir, "tool_tip_transform.npy"), tool_tip_transform)
    node.get_logger().info(f"Calibration saved to {tool_dir}")

    node.destroy_node()
    rclpy.shutdown()


def main() -> None:
    parser = argparse.ArgumentParser(description="Record head-perception calibration.")
    parser.add_argument("--tool", default="drink")
    parser.add_argument("--scene_config", default="wheelchair")
    args = parser.parse_args()
    calibrate(args.tool, args.scene_config)


if __name__ == "__main__":
    main()
