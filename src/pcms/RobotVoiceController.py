import rclpy
from rclpy.node import Node 
from std_msgs.msg import String
import edge_tts
from playsound3 import playsound
import asyncio


class RobotVoiceController(Node):
    def __init__(self):
        super().__init__(__class__.__name__)

        self.sub = self.create_subscription(
            String,
            "/pcms/say",
            self.callback_say,
            10
        )
        self.output_path = "tts.wav"

    def callback_say(self, msg):
        asyncio.run(self.generate_speech(msg.data))

    async def generate_speech(self, text):
        communicate = edge_tts.Communicate(text, "zh-HK-HiuMaanNeural", rate="+30%", volume="+0%")
        await communicate.save(self.output_path)
        print(f"Speech generated successfully and saved to {self.output_path}")
        playsound(self.output_path)


if __name__ == "__main__":
    rclpy.init()
    node = RobotVoiceController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
