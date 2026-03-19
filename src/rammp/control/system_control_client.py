"""ROS2 client for the SystemControl node in Demo-Software.

This client allows RAMMP's behavior tree HLA methods to trigger state
machine transitions in the SystemControl node and monitor its state.
It is optional: HLA methods fall back to simulation if not available.
"""

from __future__ import annotations

import time
import threading
from typing import Optional

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    RCLPY_AVAILABLE = True
except ModuleNotFoundError:
    RCLPY_AVAILABLE = False


class SystemControlClient:
    """Client for interacting with the SystemControl ROS2 node.

    Usage:
        client = SystemControlClient()
        client.pick_drink()   # blocks until complete
        client.transfer_drink()
        client.stow_drink()
        client.shutdown()
    """

    # States that indicate an operation has completed.
    _PICK_DONE_STATES = {"Arm_OrderDrink_ordered", "Arm_cupStabilize_stable"}
    _STOW_DONE_STATES = {"Arm_retracted", "Arm_home"}
    _TRANSFER_DONE_STATES = {"Arm_Drink_finished"}

    def __init__(self, timeout_sec: float = 30.0) -> None:
        if not RCLPY_AVAILABLE:
            raise RuntimeError("rclpy is not available; cannot use SystemControlClient.")

        if not rclpy.ok():
            rclpy.init()

        self._node = rclpy.create_node("rammp_system_control_client")
        self._timeout_sec = timeout_sec
        self._current_state: Optional[str] = None
        self._state_lock = threading.Lock()

        self._state_sub = self._node.create_subscription(
            String,
            "system_control/state",
            self._state_callback,
            10,
        )

        self._pick_client = self._node.create_client(Trigger, "system_control/pick_drink")
        self._stow_client = self._node.create_client(Trigger, "system_control/stow_drink")
        self._transfer_client = self._node.create_client(
            Trigger, "system_control/transfer_drink"
        )

        # Spin in a background thread so subscriptions are processed.
        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()

    def _spin(self) -> None:
        rclpy.spin(self._node)

    def _state_callback(self, msg: String) -> None:
        with self._state_lock:
            self._current_state = msg.data

    def _get_state(self) -> Optional[str]:
        with self._state_lock:
            return self._current_state

    def _call_service(self, client, description: str) -> bool:
        if not client.wait_for_service(timeout_sec=5.0):
            self._node.get_logger().warn(
                f"Service '{description}' not available after 5s."
            )
            return False
        future = client.call_async(Trigger.Request())
        deadline = time.time() + self._timeout_sec
        while not future.done():
            if time.time() > deadline:
                self._node.get_logger().warn(f"Service '{description}' call timed out.")
                return False
            time.sleep(0.05)
        response = future.result()
        if not response.success:
            self._node.get_logger().warn(
                f"Service '{description}' returned failure: {response.message}"
            )
        else:
            self._node.get_logger().info(
                f"Service '{description}' succeeded: {response.message}"
            )
        return response.success

    def _wait_for_state(self, target_states: set[str]) -> bool:
        """Block until the SystemControl state is one of target_states."""
        deadline = time.time() + self._timeout_sec
        while time.time() < deadline:
            state = self._get_state()
            if state is not None and any(s in state for s in target_states):
                return True
            time.sleep(0.1)
        self._node.get_logger().warn(
            f"Timed out waiting for states {target_states}; current={self._get_state()}"
        )
        return False

    def pick_drink(self) -> bool:
        """Trigger pick-drink sequence and wait for completion."""
        ok = self._call_service(self._pick_client, "system_control/pick_drink")
        if ok:
            ok = self._wait_for_state(self._PICK_DONE_STATES)
        return ok

    def stow_drink(self) -> bool:
        """Trigger stow-drink sequence and wait for completion."""
        ok = self._call_service(self._stow_client, "system_control/stow_drink")
        if ok:
            ok = self._wait_for_state(self._STOW_DONE_STATES)
        return ok

    def transfer_drink(self) -> bool:
        """Trigger transfer-drink sequence and wait for completion."""
        ok = self._call_service(self._transfer_client, "system_control/transfer_drink")
        if ok:
            ok = self._wait_for_state(self._TRANSFER_DONE_STATES)
        return ok

    def notify_arm_home(self) -> None:
        """Advance state machine when arm reaches home."""
        pass  # Could call a dedicated service; state published by SystemControl.

    def shutdown(self) -> None:
        self._node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
