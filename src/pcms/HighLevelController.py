import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from booster_robotics_sdk_python import (
    ChannelFactory, B1LocoClient,
    RobotMode, GetModeResponse,
    B1HandAction
)
from std_srvs.srv import Empty, Trigger, SetBool
import time


class HighLevelController(Node):
    def __init__(self):
        super().__init__(__class__.__name__)

        ChannelFactory.Instance().Init(0, "192.168.10.102")
        self.client = B1LocoClient()
        self.client.Init()

        self.sub_cmd_vel = self.create_subscription(Twist, "/pcms/cmd_vel", self.callback_cmd_vel, 10)
        self.last_vx = 0.0
        self.last_vy = 0.0
        self.last_vyaw = 0.0
        self.last_move_update_time = time.time()

        self.pub_get_mode = self.create_publisher(Int32, "/pcms/GetMode", 10)

        self.srv_switch_hand_end = self.create_service(SetBool, "/pcms/SwitchHandEnd", self.callback_switch_hand_end)
        # self.srv_dance_0 = self.create_service(Empty, "/pcms/dance0", self.callback_dance_0)
        # self.srv_dance_1 = self.create_service(Empty, "/pcms/dance1", self.callback_dance_1)
        # self.srv_dance_2 = self.create_service(Empty, "/pcms/dance2", self.callback_dance_2)
        # self.srv_dance_3 = self.create_service(Empty, "/pcms/dance3", self.callback_dance_3)
        # self.srv_dance_4 = self.create_service(Empty, "/pcms/dance4", self.callback_dance_4)
        # self.srv_dance_5 = self.create_service(Empty, "/pcms/dance5", self.callback_dance_5)
        # self.srv_dance_6 = self.create_service(Empty, "/pcms/dance6", self.callback_dance_6)
        # self.srv_dance_7 = self.create_service(Empty, "/pcms/dance7", self.callback_dance_7)
        # self.srv_dance_1000 = self.create_service(Empty, "/pcms/dance1000", self.callback_dance_1000)
        self.srv_handshake = self.create_service(SetBool, "/pcms/Handshake", self.callback_handshake)
        self.srv_prepare_mode = self.create_service(Trigger, "/pcms/PrepareMode", self.callback_prepare_mode)
        self.srv_walking_mode = self.create_service(Trigger, "/pcms/WalkingMode", self.callback_walking_mode)
        self.srv_wavehand = self.create_service(SetBool, "/pcms/WaveHand", self.callback_wavehand)
        self.srv_getup = self.create_service(Trigger, "/pcms/GetUp", self.callback_getup)

        self.timer = self.create_timer(0.5, self.update)
        self.get_logger().info("HighLevel SDK is Ready.")

    def do_Move(self, vx, vy, vyaw):
        res = self.client.Move(vx, vy, vyaw)
        if res != 0:
            self.get_logger().warning("移動控制發生問題：%d" % res)
        self.last_vx = vx 
        self.last_vy = vy
        self.last_vyaw = vyaw
        self.last_move_update_time = time.time()

    def do_ChangeMode(self, mode: RobotMode):
        res = self.client.ChangeMode(mode)
        if res != 0:
            self.get_logger().warn("改變模式出現問題：%d" % res)
        else:
            self.get_logger().info("模式改變為：%s" % mode)

    def do_WaveHand(self, action: B1HandAction):
        res = self.client.WaveHand(action)
        if res != 0:
            self.get_logger().warn("揮手動作出現問題：%d" % res)
        else:
            self.get_logger().info("開/關揮手動作：%s" % action)
        return res 

    def do_Handshake(self, action: B1HandAction):
        res = self.client.Handshake(action)
        if res != 0:
            self.get_logger().warn("握手動作出現問題：%d" % res)
        else:
            self.get_logger().info("開/關握手動作：%s" % action)
        return res
    
    # def do_dance(self, dance_id):
    #     self.client.Dance(dance_id)

    def do_SwitchHandEnd(self, switch_on: bool):
        res = self.client.SwitchHandEndEffectorControlMode(switch_on)
        if res != 0:
            self.get_logger().warn("開/關手臂控制模式出現問題：%d, %d" % (switch_on, res))
        else:
            self.get_logger().info("開/關手臂控制模式：%s" % switch_on)
        return res

    def callback_cmd_vel(self, msg: Twist):
        vx = msg.linear.x 
        vy = msg.linear.y
        vyaw = msg.angular.z 
        self.do_Move(vx, vy, vyaw)

    # def callback_dance_0(self, request, response):
    #     res = self.do_dance(0)

    def callback_switch_hand_end(self, request, response):
        res = self.do_SwitchHandEnd(request.data)
        response.success = res == 0
        return response

    def callback_handshake(self, request, response):
        if request.data:
            res = self.do_Handshake(B1HandAction.kHandOpen)
        else:
            res = self.do_Handshake(B1HandAction.kHandClose)
        response.success = res == 0
        return response

    def callback_prepare_mode(self, request, response):
        res = self.do_ChangeMode(RobotMode.kPrepare)
        response.success = res == 0
        return response
    
    def callback_walking_mode(self, request, response):
        res = self.do_ChangeMode(RobotMode.kWalking)
        response.success = res == 0
        return response
    
    def callback_wavehand(self, request, response):
        if request.data:
            res = self.do_WaveHand(B1HandAction.kHandOpen)
        else:
            res = self.do_WaveHand(B1HandAction.kHandClose)
        response.success = res == 0
        return response

    def callback_getup(self, request, response):
        res = self.client.GetUp()
        if res != 0:
            self.get_logger().warn("起身動作出現問題：" % res)
        else:
            self.get_logger().info("開始起身動作。")
        response.success = res == 0
        return response


    def update(self):
        # 如果多於1秒沒有收到控制指令會停止
        if time.time() - self.last_move_update_time > 1.0:  
            if self.last_vx != 0.0 or self.last_vy != 0.0 or self.last_vyaw != 0.0:
                self.do_Move(0.0, 0.0, 0.0)

        # 更新
        gm = GetModeResponse()
        res = self.client.GetMode(gm)
        msg = Int32()
        msg.data = int(gm.mode)
        self.pub_get_mode.publish(msg)


if __name__ == "__main__":
    rclpy.init()
    node = HighLevelController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
