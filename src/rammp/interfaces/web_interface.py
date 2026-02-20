"""
A minimal ROS <-> WebApp interface for drinking
"""

import json
import queue
import time
from pathlib import Path
from typing import Any, Callable, Optional

try:
    import rospy
    from std_msgs.msg import String
except ModuleNotFoundError:
    rospy = None
    String = None


class WebInterface:
    """
    Interface to a web UI via ROS topics:
      - Publishes to:   /ServerComm   (std_msgs/String)  JSON payloads
      - Subscribes to:  WebAppComm    (std_msgs/String)  JSON payloads

    task_selection_queue receives only:
      - {"task": "drink", "type": "sip"}
      - {"task": "reset", "type": "reset"}  (if webapp sends finish_feeding, preserved as reset signal)
    """

    def __init__(self, task_selection_queue: queue.Queue, log_dir: Path) -> None:
        if rospy is None:
            raise RuntimeError("rospy not available. Source your ROS environment before running.")

        self.task_selection_queue = task_selection_queue
        self.received_messages: queue.Queue[dict[str, Any]] = queue.Queue()

        # Logs
        log_dir.mkdir(parents=True, exist_ok=True)
        self.webapp_sent_messages_log = log_dir / "webapp_sent_messages.txt"
        self.webapp_received_messages_log = log_dir / "webapp_received_messages.txt"

        # ROS pub/sub
        self.web_interface_publisher = rospy.Publisher("/ServerComm", String, queue_size=10)
        self.web_interface_sub = rospy.Subscriber("WebAppComm", String, self._message_callback, queue_size=100)
        time.sleep(1.0)  # let connections settle

        # State
        self.active = True
        self.task_selection_jump = False
        self.current_page = "task_selection"
        self.drink_autocontinue_timeout = 10.0

    def stop(self) -> None:
        self.active = False

    def _send_message(self, msg_dict: dict[str, Any]) -> None:
        """Publish JSON to /ServerComm and log it."""
        payload = json.dumps(msg_dict)
        self.web_interface_publisher.publish(String(payload))
        with open(self.webapp_sent_messages_log, "a") as f:
            f.write(payload + "\n")

    def _message_callback(self, msg: "String") -> None:
        """Receive JSON from WebAppComm, log it, and route it."""
        print("Received message on WebAppComm:", msg.data)
        with open(self.webapp_received_messages_log, "a") as f:
            f.write(msg.data + "\n")

        msg_dict = json.loads(msg.data)
        self.task_selection_jump = False

        if msg_dict.get("status") == "finish_feeding":
            self.task_selection_queue.put({"task": "reset", "type": "reset"})
            return

        if msg_dict.get("state") == "task_selection":
            if msg_dict.get("status") == "take_sip":
                self.current_page = "task_selection"
                self.task_selection_queue.put({"task": "meal_assistance", "type": "sip"})
                return
            if msg_dict.get("status") == "jump":
                self.task_selection_jump = True
                return

            print("Ignoring non-drink task selection:", msg_dict.get("status"))
            return

        # Everything else goes into a generic queue for blocking waits.
        self.received_messages.put(msg_dict)

    def clear_received_messages(self) -> None:
        while not self.received_messages.empty():
            self.received_messages.get()

    def get_required_message(
        self, condition: Callable[[dict[str, Any]], bool]
    ) -> Optional[dict[str, Any]]:
        """Block until a message satisfying condition arrives (or jump/stop)."""
        print_once = True
        while self.active:
            if self.task_selection_jump:
                return None
            try:
                msg_dict = self.received_messages.get_nowait()
                try:
                    if condition(msg_dict):
                        return msg_dict
                except Exception as e:
                    print("Condition error:", e)
            except queue.Empty:
                if print_once:
                    print("Waiting for required message from web interface ...")
                    print_once = False
                time.sleep(0.1)
        return None

    def set_drink_autocontinue_timeout(self, timeout: float) -> None:
        self.drink_autocontinue_timeout = float(timeout)

    def ready_for_task_selection(self, last_task_type: Optional[str] = None) -> None:
        """
        Move UI to task selection.
        """
        self.current_page = "task_selection"
        print("Moving to task selection page. last_task_type =", last_task_type)

        if last_task_type == "sip":
            self._send_message({"state": "afterdrinktransfer", "status": "jump"})
            time.sleep(0.5)
            self._send_message({"state": "auto_time", "status": str(self.drink_autocontinue_timeout)})
        else:
            self._send_message({"state": "task_selection", "status": "jump"})

    def get_drink_transfer_confirmation(self) -> bool:
        """
        Jump to "transferdrinks" page and wait for confirmation.

        Returns:
            True if confirmed, False if interrupted (jump/back/stop).
        """
        self.current_page = "drink"
        self._send_message({"state": "transferdrinks", "status": "jump"})

        msg = self.get_required_message(
            lambda m: (m.get("state") == "post_drink_pickup" and m.get("status") == "drink_transfer")
        )
        return msg is not None


if __name__ == "__main__":
    rospy.init_node("web_interface_drink_only")

    log_dir = Path(__file__).parent / "web_interface_log"
    task_selection_queue: queue.Queue = queue.Queue()
    web_interface = WebInterface(task_selection_queue, log_dir)

    # Example usage pattern:
    # 1) show task selection
    web_interface.ready_for_task_selection()

    # 2) wait for the user to click "take_sip" (your main loop would do this)
    print("Waiting for a drink task selection...")
    task = task_selection_queue.get()  # blocks
    print("Selected:", task)

    if task["task"] == "drink" and task["type"] == "sip":
        ok = web_interface.get_drink_transfer_confirmation()
        print("Drink transfer confirmed:", ok)

        # 3) go back to task selection (optionally with sip autocontinue)
        web_interface.ready_for_task_selection(last_task_type="sip")