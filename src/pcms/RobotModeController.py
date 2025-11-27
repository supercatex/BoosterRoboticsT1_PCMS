import rclpy
from rclpy.node import Node
from booster_robotics_sdk_python import ChannelFactory, B1LocoClient, GetModeResponse, RobotMode
from std_msgs.msg import Int32


class RobotModeController(Node):
    def __init__(self):
        super().__init__(__class__.__name__)

        ChannelFactory.Instance().Init(0, "192.168.10.102")
        self.client = B1LocoClient()
        self.client.Init()

        self.pub = self.create_publisher(
            Int32,
            "/pcms/GetMode",
            10
        )
        self.sub = self.create_subscription(
            Int32,
            "/pcms/ChangeMode",
            self.callback_change_mode,
            10
        )

        self.timer = self.create_timer(0.5, self.update)
    
    def update(self):
        gm = GetModeResponse()
        res = self.client.GetMode(gm)
        msg = Int32()
        msg.data = int(gm.mode)
        self.pub.publish(msg)

    def callback_change_mode(self, msg):
        res = self.client.ChangeMode(RobotMode(msg.data))
        print(f"Change to {msg} mode.")


if __name__ == "__main__":
    rclpy.init()
    node = RobotModeController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    