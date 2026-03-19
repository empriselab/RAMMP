import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tempfile
from gtts import gTTS
from playsound import playsound

class Speak(Node):
    def __init__(self):
        super().__init__('Speak')
        self.create_subscription(String, "/speak", self.callback, 10)
        print("Speak node initialized")

    # Speak the text
    def callback(self, msg):
        text = msg.data
        print("Speaking: ", text)

        # Convert text to speech and play
        with tempfile.NamedTemporaryFile(suffix=".mp3", delete=True) as voice:
            gTTS(text=text, lang="en").write_to_fp(voice)
            voice.flush()  # Ensure data is written to file
            playsound(voice.name)

if __name__ == "__main__":
    rclpy.init()
    speak_node = Speak()
    rclpy.spin(speak_node)
    speak_node.destroy_node()
    rclpy.shutdown()
