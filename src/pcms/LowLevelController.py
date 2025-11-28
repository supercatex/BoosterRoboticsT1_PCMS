import rclpy
from rclpy.node import Node 
from geometry_msgs.msg import Vector3
from booster_robotics_sdk_python import (
    ChannelFactory, 
    B1JointCnt, B1JointIndex, 
    MotorCmd, 
    B1LowCmdPublisher, LowCmd, LowCmdType
)
import numpy as np 


class LowLevelController(Node):
    def __init__(self):
        super().__init__(__class__.__name__)

        ChannelFactory.Instance().Init(0)
        self.channel_publisher = B1LowCmdPublisher()
        self.channel_publisher.InitChannel()

        self.low_cmd = LowCmd()
        self.clear_LowCmd()
        self.sub_move_head = self.create_subscription(Vector3, "/pcms/MoveHead", self.callback_move_head, 10)

        self.timer = self.create_timer(0.1, self.update)

    def clear_LowCmd(self):
        self.low_cmd = LowCmd()
        self.low_cmd.cmd_type = LowCmdType.SERIAL
        self.low_cmd.motor_cmd = [MotorCmd() for _ in range(B1JointCnt)]

    def set_LowCmd_by_index(self, id, q, dq=0.0, tau=0.0, kp=4.0, kd=2.0, weight=1.0):
        self.low_cmd.motor_cmd[id].q = q 
        self.low_cmd.motor_cmd[id].dq = dq 
        self.low_cmd.motor_cmd[id].tau = tau 
        self.low_cmd.motor_cmd[id].kp = kp 
        self.low_cmd.motor_cmd[id].kd = kd 
        self.low_cmd.motor_cmd[id].weight = weight

    def callback_move_head(self, msg: Vector3):
        pitch, yaw = msg.y, msg.z
        pitch = max(-18 * np.pi / 180, min(pitch, 48 * np.pi / 180))
        yaw = max(-58 * np.pi / 180, min(yaw, 58 * np.pi / 180))

        self.set_LowCmd_by_index(B1JointIndex.kHeadPitch.value, pitch)
        self.set_LowCmd_by_index(B1JointIndex.kHeadYaw.value, yaw)

        print("MoveHead: pitch = %.2f, yaw = %.2f" % (pitch, yaw))
        # self.channel_publisher.Write(self.low_cmd)

    def update(self):
        self.channel_publisher.Write(self.low_cmd)


if __name__ == "__main__":
    rclpy.init()
    node = LowLevelController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
