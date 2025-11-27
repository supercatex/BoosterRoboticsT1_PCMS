import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from booster_robotics_sdk_python import ChannelFactory, B1LocoClient
import time


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
        self.last_vx = 0.0
        self.last_vy = 0.0
        self.last_vyaw = 0.0
        self.last_update_time = time.time()
        self.timer = self.create_timer(1.0, self.update)

    def callback_cmd_vel(self, msg: Twist):
        self.last_update_time = time.time()
        vx = msg.linear.x 
        vy = msg.linear.y
        vyaw = msg.angular.z 
        self.client.Move(vx, vy, vyaw)
        self.last_vx = vx 
        self.last_vy = vy
        self.last_vyaw = vyaw


    def update(self):
        if time.time() - self.last_update_time > 2.0:
            if self.last_vx != 0.0 or self.last_vy != 0.0 or self.last_vyaw != 0.0:
                self.client.Move(0.0, 0.0, 0.0)
                self.last_vx = 0.0
                self.last_vy = 0.0
                self.last_vyaw = 0.0
                self.last_update_time = time.time()

if __name__ == "__main__":
    rclpy.init()
    node = RobotMoveController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
