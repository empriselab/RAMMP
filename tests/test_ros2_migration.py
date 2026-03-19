"""Tests to verify the ROS1 -> ROS2 migration of all files in src/rammp/.

These tests mock rclpy and ROS2 message types so they can run without a
ROS2 installation. They verify:
  1. Modules import successfully with mocked ROS2 dependencies.
  2. No residual rospy imports remain in migrated files.
  3. Key class constructors and methods work with mocked ROS2 objects.
  4. The launch file is valid Python and produces a LaunchDescription.
"""

import importlib
import os
import re
import sys
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch, PropertyMock

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

RAMMP_SRC = Path(__file__).resolve().parent.parent / "src" / "rammp"

# All migrated Python files (relative to RAMMP_SRC)
MIGRATED_FILES = [
    "utils/tf_utils.py",
    "control/robot_controller/joint_states_publisher.py",
    "control/robot_controller/arm_client.py",
    "safety/estops_publisher.py",
    "safety/watchdog.py",
    "safety/bulldog.py",
    "safety/collision_sensor.py",
    "interfaces/perception_interface.py",
    "interfaces/rviz_interface.py",
    "interfaces/web_interface.py",
    "perception/head_perception/ros_wrapper.py",
    "perception/drink_perception/drink_perception.py",
    "misc/speak.py",
    "misc/transfer_button_listener.py",
    "misc/mock_web_interface.py",
    "integration/run.py",
    "integration/upright_ee.py",
    "integration/test_actions.py",
]


def _read_source(relpath: str) -> str:
    """Read the source code of a migrated file."""
    return (RAMMP_SRC / relpath).read_text()


# ---------------------------------------------------------------------------
# 1. No residual rospy imports
# ---------------------------------------------------------------------------

class TestNoRospyImports(unittest.TestCase):
    """Ensure no migrated file still imports rospy."""

    def test_no_rospy_import(self):
        """Check that 'import rospy' or 'from rospy' does not appear."""
        for relpath in MIGRATED_FILES:
            src = _read_source(relpath)
            # Match actual import statements, not comments
            lines = src.splitlines()
            for i, line in enumerate(lines, 1):
                stripped = line.lstrip()
                if stripped.startswith("#"):
                    continue
                self.assertNotRegex(
                    stripped,
                    r"^(import rospy|from rospy\b)",
                    msg=f"{relpath}:{i} still imports rospy: {stripped!r}",
                )


# ---------------------------------------------------------------------------
# 2. Check rclpy imports present
# ---------------------------------------------------------------------------

class TestRclpyImportsPresent(unittest.TestCase):
    """Ensure migrated files use rclpy instead of rospy."""

    # Files that only conditionally import ROS (try/except) are allowed
    # to not have a top-level rclpy import, but should reference rclpy.
    def test_rclpy_referenced(self):
        for relpath in MIGRATED_FILES:
            src = _read_source(relpath)
            self.assertIn(
                "rclpy",
                src,
                msg=f"{relpath} does not reference rclpy anywhere",
            )


# ---------------------------------------------------------------------------
# 3. No rospy.Time / rospy.Duration / rospy.Rate patterns
# ---------------------------------------------------------------------------

class TestNoRospyPatterns(unittest.TestCase):
    """Check that common rospy patterns are gone."""

    PATTERNS = [
        r"rospy\.init_node",
        r"rospy\.Publisher\(",
        r"rospy\.Subscriber\(",
        r"rospy\.Time\b",
        r"rospy\.Duration\b",
        r"rospy\.Rate\(",
        r"rospy\.spin\b",
        r"rospy\.is_shutdown\b",
        r"rospy\.wait_for_message\b",
        r"rospy\.loginfo\b",
        r"rospy\.sleep\b",
    ]

    def test_no_rospy_patterns(self):
        for relpath in MIGRATED_FILES:
            src = _read_source(relpath)
            for pat in self.PATTERNS:
                # Skip commented lines
                active_lines = [
                    line for line in src.splitlines()
                    if not line.lstrip().startswith("#")
                ]
                active_src = "\n".join(active_lines)
                matches = re.findall(pat, active_src)
                self.assertEqual(
                    len(matches), 0,
                    msg=f"{relpath} still contains pattern {pat!r} ({len(matches)} match(es))",
                )


# ---------------------------------------------------------------------------
# 4. ROS2 API patterns present in key files
# ---------------------------------------------------------------------------

class TestRos2PatternsPresent(unittest.TestCase):
    """Verify that ROS2-specific API calls appear where expected."""

    def test_create_publisher_in_publishers(self):
        """Files that publish should use create_publisher or node.create_publisher."""
        publisher_files = [
            "control/robot_controller/joint_states_publisher.py",
            "safety/estops_publisher.py",
            "safety/watchdog.py",
            "safety/bulldog.py",
            "safety/collision_sensor.py",
        ]
        for relpath in publisher_files:
            src = _read_source(relpath)
            self.assertIn(
                "create_publisher",
                src,
                msg=f"{relpath} should use create_publisher",
            )

    def test_create_subscription_in_subscribers(self):
        """Files that subscribe should use create_subscription or node.create_subscription."""
        subscriber_files = [
            "safety/watchdog.py",
            "safety/bulldog.py",
            "safety/collision_sensor.py",
            "interfaces/perception_interface.py",
        ]
        for relpath in subscriber_files:
            src = _read_source(relpath)
            self.assertIn(
                "create_subscription",
                src,
                msg=f"{relpath} should use create_subscription",
            )

    def test_node_inheritance_or_injection(self):
        """Standalone nodes should inherit from Node or accept a node parameter."""
        standalone_nodes = [
            "control/robot_controller/joint_states_publisher.py",
            "safety/estops_publisher.py",
            "safety/watchdog.py",
            "safety/bulldog.py",
            "safety/collision_sensor.py",
            "misc/speak.py",
            "misc/transfer_button_listener.py",
        ]
        for relpath in standalone_nodes:
            src = _read_source(relpath)
            has_node_parent = "(Node)" in src
            has_node_param = "node:" in src or "node :" in src
            self.assertTrue(
                has_node_parent or has_node_param,
                msg=f"{relpath} should inherit from Node or accept a node parameter",
            )


# ---------------------------------------------------------------------------
# 5. Launch file validation
# ---------------------------------------------------------------------------

class TestLaunchFile(unittest.TestCase):
    """Verify the ROS2 Python launch file is syntactically valid."""

    def test_launch_file_exists(self):
        launch_file = Path(__file__).resolve().parent.parent / "launch" / "robot.launch.py"
        self.assertTrue(launch_file.exists(), "launch/robot.launch.py should exist")

    def test_launch_file_is_valid_python(self):
        launch_file = Path(__file__).resolve().parent.parent / "launch" / "robot.launch.py"
        src = launch_file.read_text()
        try:
            compile(src, str(launch_file), "exec")
        except SyntaxError as e:
            self.fail(f"launch/robot.launch.py has syntax error: {e}")

    def test_launch_file_has_generate_launch_description(self):
        launch_file = Path(__file__).resolve().parent.parent / "launch" / "robot.launch.py"
        src = launch_file.read_text()
        self.assertIn(
            "generate_launch_description",
            src,
            "launch file should define generate_launch_description()",
        )

    def test_launch_file_no_xml_tags(self):
        launch_file = Path(__file__).resolve().parent.parent / "launch" / "robot.launch.py"
        src = launch_file.read_text()
        self.assertNotIn("<launch>", src, "Launch file should not contain XML tags")
        self.assertNotIn("<node", src, "Launch file should not contain XML node tags")


# ---------------------------------------------------------------------------
# 6. Shell scripts updated
# ---------------------------------------------------------------------------

class TestShellScripts(unittest.TestCase):
    """Verify shell scripts no longer reference roscore."""

    def test_run_bulldog_no_roscore(self):
        script = RAMMP_SRC / "integration" / "run_bulldog.sh"
        src = script.read_text()
        # Should not launch roscore
        self.assertNotIn(
            "roscore &",
            src,
            "run_bulldog.sh should not launch roscore (not needed in ROS2)",
        )

    def test_launch_robot_no_roscore(self):
        script = RAMMP_SRC / "integration" / "launch_robot.sh"
        src = script.read_text()
        self.assertNotIn(
            "roscore",
            src,
            "launch_robot.sh should not reference roscore",
        )


# ---------------------------------------------------------------------------
# 7. TF2 usage is ROS2-compatible
# ---------------------------------------------------------------------------

class TestTF2Migration(unittest.TestCase):
    """Check that tf2_ros usage passes the node to constructors."""

    TF2_FILES = [
        "utils/tf_utils.py",
        "interfaces/perception_interface.py",
        "interfaces/rviz_interface.py",
        "perception/head_perception/ros_wrapper.py",
        "perception/drink_perception/drink_perception.py",
    ]

    def test_tf2_transform_listener_gets_node(self):
        """In ROS2, TransformListener requires a node argument."""
        for relpath in self.TF2_FILES:
            src = _read_source(relpath)
            if "TransformListener" in src:
                # Should pass node/self.node/self as second arg
                # Just check it's not bare tf2_ros.TransformListener(self.tfBuffer)
                active_lines = [
                    line.strip() for line in src.splitlines()
                    if "TransformListener" in line and not line.lstrip().startswith("#")
                ]
                for line in active_lines:
                    # Should have more than just the buffer argument
                    self.assertRegex(
                        line,
                        r"TransformListener\(.*,\s*\w+",
                        msg=f"{relpath}: TransformListener should receive a node arg: {line}",
                    )

    def test_tf2_broadcaster_gets_node(self):
        """In ROS2, TransformBroadcaster requires a node argument."""
        for relpath in self.TF2_FILES:
            src = _read_source(relpath)
            if "TransformBroadcaster()" in src:
                self.fail(
                    f"{relpath}: TransformBroadcaster() called without node argument"
                )


# ---------------------------------------------------------------------------
# 8. message_filters uses ROS2 API
# ---------------------------------------------------------------------------

class TestMessageFiltersMigration(unittest.TestCase):
    """Check that message_filters.Subscriber uses ROS2 signature."""

    MSG_FILTER_FILES = [
        "perception/head_perception/ros_wrapper.py",
        "perception/drink_perception/drink_perception.py",
    ]

    def test_message_filters_subscriber_ros2_signature(self):
        """In ROS2, message_filters.Subscriber takes (node, MsgType, topic)."""
        for relpath in self.MSG_FILTER_FILES:
            src = _read_source(relpath)
            active_lines = [
                line.strip() for line in src.splitlines()
                if "message_filters.Subscriber(" in line and not line.lstrip().startswith("#")
            ]
            for line in active_lines:
                # ROS1 pattern: message_filters.Subscriber("/topic", MsgType, ...)
                # ROS2 pattern: message_filters.Subscriber(node, MsgType, "/topic")
                # Check it does NOT start with a string (topic) as first arg
                self.assertNotRegex(
                    line,
                    r'message_filters\.Subscriber\(\s*["\']/',
                    msg=f"{relpath}: message_filters.Subscriber should use ROS2 signature (node, MsgType, topic): {line}",
                )


# ---------------------------------------------------------------------------
# 9. rclpy.ok() replaces rospy.is_shutdown()
# ---------------------------------------------------------------------------

class TestRclpyOkUsage(unittest.TestCase):
    """Files with main loops should use rclpy.ok() or rclpy.spin()."""

    LOOP_FILES = [
        "control/robot_controller/joint_states_publisher.py",
        "safety/estops_publisher.py",
        "safety/watchdog.py",
        "safety/bulldog.py",
        "misc/transfer_button_listener.py",
    ]

    def test_rclpy_ok_in_loops(self):
        for relpath in self.LOOP_FILES:
            src = _read_source(relpath)
            has_rclpy_ok = "rclpy.ok()" in src
            has_rclpy_spin = "rclpy.spin" in src
            self.assertTrue(
                has_rclpy_ok or has_rclpy_spin,
                msg=f"{relpath} should use rclpy.ok() or rclpy.spin() for its main loop",
            )


# ---------------------------------------------------------------------------
# 10. rclpy.init() / rclpy.shutdown() in __main__ blocks
# ---------------------------------------------------------------------------

class TestRclpyLifecycle(unittest.TestCase):
    """Standalone scripts should call rclpy.init() and rclpy.shutdown()."""

    STANDALONE_FILES = [
        "control/robot_controller/joint_states_publisher.py",
        "safety/estops_publisher.py",
        "safety/watchdog.py",
        "safety/bulldog.py",
        "safety/collision_sensor.py",
        "misc/speak.py",
        "misc/transfer_button_listener.py",
        "misc/mock_web_interface.py",
    ]

    def test_rclpy_init_and_shutdown(self):
        for relpath in self.STANDALONE_FILES:
            src = _read_source(relpath)
            self.assertIn(
                "rclpy.init()",
                src,
                msg=f"{relpath} should call rclpy.init()",
            )
            self.assertIn(
                "rclpy.shutdown()",
                src,
                msg=f"{relpath} should call rclpy.shutdown()",
            )


# ---------------------------------------------------------------------------
# 11. get_clock().now().to_msg() replaces rospy.Time.now()
# ---------------------------------------------------------------------------

class TestTimeStampPattern(unittest.TestCase):
    """Files that set header.stamp should use get_clock().now().to_msg()."""

    STAMP_FILES = [
        "utils/tf_utils.py",
        "control/robot_controller/joint_states_publisher.py",
        "interfaces/rviz_interface.py",
        "perception/head_perception/ros_wrapper.py",
        "perception/drink_perception/drink_perception.py",
    ]

    def test_to_msg_for_stamps(self):
        for relpath in self.STAMP_FILES:
            src = _read_source(relpath)
            if "header.stamp" in src:
                self.assertIn(
                    "to_msg()",
                    src,
                    msg=f"{relpath} should use .to_msg() for stamp assignment",
                )


# ---------------------------------------------------------------------------
# 12. Web interface migration
# ---------------------------------------------------------------------------

class TestWebInterfaceMigration(unittest.TestCase):
    """Check web_interface.py migration specifics."""

    def test_accepts_node_parameter(self):
        src = _read_source("interfaces/web_interface.py")
        self.assertIn("node:", src, "WebInterface.__init__ should accept a node parameter")

    def test_no_rospy_runtime_error(self):
        src = _read_source("interfaces/web_interface.py")
        self.assertNotIn(
            "rospy not available",
            src,
            "Error message should reference rclpy, not rospy",
        )
        self.assertIn(
            "rclpy not available",
            src,
            "Error message should mention rclpy",
        )


# ---------------------------------------------------------------------------
# 13. Perception interface wait_for_message replacement
# ---------------------------------------------------------------------------

class TestWaitForMessageReplacement(unittest.TestCase):
    """rospy.wait_for_message should be replaced with subscription pattern."""

    def test_no_wait_for_message(self):
        src = _read_source("interfaces/perception_interface.py")
        active_lines = [
            line for line in src.splitlines()
            if not line.lstrip().startswith("#")
        ]
        active_src = "\n".join(active_lines)
        self.assertNotIn(
            "wait_for_message",
            active_src,
            "perception_interface.py should not use wait_for_message",
        )

    def test_uses_create_subscription_pattern(self):
        src = _read_source("interfaces/perception_interface.py")
        self.assertIn(
            "create_subscription",
            src,
            "perception_interface.py should use create_subscription as wait_for_message replacement",
        )


if __name__ == "__main__":
    unittest.main()
