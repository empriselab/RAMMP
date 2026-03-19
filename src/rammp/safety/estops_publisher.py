
import time
import argparse
import pyaudio

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from rammp.safety.button import Button as EStop
ESTOP_FREQUENCY = 100

class EStopsPublisher(Node):
    def __init__(self, user_estop_id: int, experimentor_estop_id: int):
        super().__init__('estop_publisher')

        self.user_estop = EStop(user_estop_id)
        self.experimentor_estop = EStop(experimentor_estop_id)

        self.user_estop_pub = self.create_publisher(Bool, "/user_estop", 1)
        self.experimentor_estop_pub = self.create_publisher(Bool, "/experimentor_estop", 1)

    def run(self):
        while rclpy.ok():
            start_time = time.time()
            user_estop_pressed = self.user_estop.check()
            experimentor_estop_pressed = self.experimentor_estop.check()

            self.user_estop_pub.publish(Bool(data=user_estop_pressed))
            self.experimentor_estop_pub.publish(Bool(data=experimentor_estop_pressed))

            if user_estop_pressed:
                print("User E-Stop pressed")

            if experimentor_estop_pressed:
                print("Experimentor E-Stop pressed")

            time.sleep(max(0, 1.0/ESTOP_FREQUENCY - (time.time() - start_time)))

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--user_id", type=int)
    parser.add_argument("--exp_id", type=int)

    args = parser.parse_args()

    if args.user_id is None or args.exp_id is None:
        audio = pyaudio.PyAudio()
        device_indices = []
        for i in range(audio.get_device_count()):
            device_info = audio.get_device_info_by_index(i)
            if device_info["maxInputChannels"] > 0:  # Only consider input devices
                device_indices.append(i)
                print(f"Device {i}: {device_info['name']}")
        raise ValueError("Please provide the input device index")

    rclpy.init()
    estop_publisher = EStopsPublisher(user_estop_id=args.user_id, experimentor_estop_id=args.exp_id)
    estop_publisher.run()
    estop_publisher.destroy_node()
    rclpy.shutdown()
