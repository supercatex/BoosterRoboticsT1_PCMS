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


class RobotHeadController(Node):
    def __init__(self):
        super().__init__(__class__.__name__)

        ChannelFactory.Instance().Init(0)
        self.channel_publisher = B1LowCmdPublisher()
        self.channel_publisher.InitChannel()

        self.sub = self.create_subscription(
            Vector3,
            "/pcms/MoveHead",
            self.callback_move_head,
            10
        )

    def callback_move_head(self, msg: Vector3):
        pitch, yaw = msg.y, msg.z
        pitch = max(-18 * np.pi / 180, min(pitch, 48 * np.pi / 180))
        yaw = max(-58 * np.pi / 180, min(yaw, 58 * np.pi / 180))

        low_cmd = LowCmd()
        low_cmd.cmd_type = LowCmdType.SERIAL
        low_cmd.motor_cmd = [MotorCmd() for _ in range(B1JointCnt)]

        low_cmd.motor_cmd[B1JointIndex.kHeadPitch.value].q = pitch
        low_cmd.motor_cmd[B1JointIndex.kHeadPitch.value].dq = 0.0
        low_cmd.motor_cmd[B1JointIndex.kHeadPitch.value].tau = 0.0
        low_cmd.motor_cmd[B1JointIndex.kHeadPitch.value].kp = 4.0
        low_cmd.motor_cmd[B1JointIndex.kHeadPitch.value].kd = 2.0
        low_cmd.motor_cmd[B1JointIndex.kHeadPitch.value].weight = 1.0

        low_cmd.motor_cmd[B1JointIndex.kHeadYaw.value].q = yaw
        low_cmd.motor_cmd[B1JointIndex.kHeadYaw.value].dq = 0.0
        low_cmd.motor_cmd[B1JointIndex.kHeadYaw.value].tau = 0.0
        low_cmd.motor_cmd[B1JointIndex.kHeadYaw.value].kp = 4.0
        low_cmd.motor_cmd[B1JointIndex.kHeadYaw.value].kd = 2.0
        low_cmd.motor_cmd[B1JointIndex.kHeadYaw.value].weight = 1.0

        print("MoveHead: pitch = %.2f, yaw = %.2f" % (pitch, yaw))
        self.channel_publisher.Write(low_cmd)


    def create_motor_cmd(self, q, dq=0.0, tau=0.0, kp=4.0, kd=2.0, weight=1.0):
        data = MotorCmd()
        data.q = q 
        data.dq = dq 
        data.tau = tau
        data.kp = kp 
        data.kd = kd 
        data.weight = weight
        return data


if __name__ == "__main__":
    rclpy.init()
    node = RobotHeadController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
