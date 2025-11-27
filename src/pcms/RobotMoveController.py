import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from booster_robotics_sdk_python import ChannelFactory, B1LocoClient


class RobotMoveController(Node):
    def __init__(self):
        super().__init__(__class__.__name__)

        ChannelFactory.Instance().Init(0, "192.168.10.102")
        self.client = B1LocoClient()
        self.client.Init()

        self.sub = self.create_subscription(
            Twist,
            "/pcms/cmd_vel",
            self.callback_cmd_vel,
            10
        )

    def callback_cmd_vel(self, msg: Twist):
        vx = msg.linear.x 
        vy = msg.linear.y
        vyaw = msg.angular.z 
        self.client.Move(vx, vy, vyaw)


if __name__ == "__main__":
    rclpy.init()
    node = RobotMoveController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
