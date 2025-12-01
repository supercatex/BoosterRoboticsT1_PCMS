import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from booster_interface.msg import FallDownState, Odometer, LowState
from booster_robotics_sdk_python import (
    ChannelFactory, B1LocoClient,
    RobotMode, B1HandAction, B1HandIndex,
    Position, Orientation, Transform, Frame
)
from sensor_msgs.msg import Joy
from cv_bridge import CvBridge
from std_msgs.msg import Int32, String, Float64MultiArray
from geometry_msgs.msg import Twist, Vector3
from std_srvs.srv import Empty, Trigger, SetBool
import time
import cv2
import numpy as np


class MainController(Node):
    def __init__(self, func_setup, func_update, update_time: float = 0.05):
        super().__init__(__class__.__name__)

        self.func_setup = func_setup
        self.func_update = func_update
        self.update_time = update_time

        self.image, self.depth = None, None
        self.falldown: FallDownState = None
        self.odometer: Odometer = None
        self.low_state: LowState = None
        self.joy: Joy = None
        self.mode:RobotMode = None
        self.cb = CvBridge()

        self.sub_image = self.create_subscription(Image, "/camera/color/image_raw", self.callback_image, 10)
        self.sub_depth = self.create_subscription(Image, "/camera/depth/image_raw", self.callback_depth, 10)
        self.sub_falldown = self.create_subscription(FallDownState, "/fall_down", self.callback_falldown, 10)
        self.sub_odometer = self.create_subscription(Odometer, "/odometer_state", self.callback_odometer, 10)
        self.sub_low_state = self.create_subscription(LowState, "/low_state", self.callback_low_state, 10)
        self.sub_joy = self.create_subscription(Joy, "/joy", self.callback_joy, 10)
        self.sub_get_mode = self.create_subscription(Int32, "/pcms/GetMode", self.callback_get_mode, 10)

        self.pub_say = self.create_publisher(String, "/pcms/say", 10)
        self.pub_cmd_vel = self.create_publisher(Twist, "/pcms/cmd_vel", 10)
        self.pub_move_head = self.create_publisher(Vector3, "/pcms/MoveHead", 10)
        self.pub_move_hand_end = self.create_publisher(Float64MultiArray, "/pcms/move_hand_end", 10)

        self.cli_switch_hand_end = self.create_client(SetBool, "/pcms/SwitchHandEnd")
        # self.cli_dance_0 = self.create_client(Empty, "/pcms/dance0")
        self.cli_handshake = self.create_client(SetBool, "/pcms/Handshake")
        self.cli_prepare_mode = self.create_client(Trigger, "/pcms/PrepareMode")
        self.cli_walking_mode = self.create_client(Trigger, "/pcms/WalkingMode")
        self.cli_wave_hand = self.create_client(SetBool, "/pcms/WaveHand")
        self.cli_getup = self.create_client(Trigger, "/pcms/GetUp")
        self.cli_liedown = self.create_client(Trigger, "/pcms/LieDown")

        ChannelFactory.Instance().Init(0, "192.168.10.102")
        self.client = B1LocoClient()
        self.client.Init()

        time.sleep(3)
        self.func_setup(self)

        self.fps = 0.0
        self.last_update_time = time.time()
        self.timer = self.create_timer(self.update_time, self.update)


    def callback_image(self, msg):
        self.image = self.cb.imgmsg_to_cv2(msg, "bgr8")

    def callback_depth(self, msg):
        self.depth = self.cb.imgmsg_to_cv2(msg, "passthrough")

    def callback_falldown(self, msg):
        self.falldown = msg

    def callback_odometer(self, msg):
        self.odometer = msg

    def callback_low_state(self, msg):
        self.low_state = msg

    def callback_joy(self, msg):
        self.joy = msg

    def callback_get_mode(self, msg):
        self.mode = msg

    def say(self, text):
        msg = String()
        msg.data = text
        self.pub_say.publish(msg)

    def cmd_vel(self, vx, vy, vz):
        msg = Twist()
        msg.linear.x = vx 
        msg.linear.y = vy
        msg.angular.z = vz 
        self.pub_cmd_vel.publish(msg)

    def move_hand_end(self, px, py, pz, ox, oy, oz, t, i: B1HandIndex):
        msg = Float64MultiArray()
        msg.data = [px, py, pz, ox, oy, oz, float(t), float(i)]
        self.pub_move_hand_end.publish(msg)

    def switch_hand_end(self, switch_on: bool):
        self.cli_switch_hand_end.call_async(SetBool.Request(data=switch_on))

    def move_head(self, pitch, yaw):
        msg = Vector3()
        msg.y = pitch
        msg.z = yaw
        self.pub_move_head.publish(msg)

    def handshake(self, is_open: bool, delay=3.0):
        self.cli_handshake.call_async(SetBool.Request(data=is_open))
        time.sleep(delay)
    
    def prepare_mode(self, delay=3.0):
        self.cli_prepare_mode.call_async(Trigger.Request())
        time.sleep(delay)
    
    def walking_mode(self, delay=3.0):
        self.cli_walking_mode.call_async(Trigger.Request())
        time.sleep(delay)

    def wave_hand(self, is_open: bool, delay=3.0):
        self.cli_wave_hand.call_async(SetBool.Request(data=is_open))
        time.sleep(delay)

    def getup(self, delay=10.0):
        self.cli_getup.call_async(Trigger.Request())
        time.sleep(delay)

    def liedown(self, delay=10.0):
        self.cli_liedown.call_async(Trigger.Request())
        time.sleep(delay)

    def euler_from_quaternion(self, quaternion):
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def quaternion_from_euler(self, roll, pitch, yaw):
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        q = [0] * 4
        q[0] = cy * cp * cr + sy * sp * sr
        q[1] = cy * cp * sr - sy * sp * cr
        q[2] = sy * cp * sr + cy * sp * cr
        q[3] = sy * cp * cr - cy * sp * sr

        return q
    
    def rot_x(self, theta, point):
        m = np.array([
            [1, 0, 0],
            [0, np.cos(theta), -np.sin(theta)],
            [0, np.sin(theta), np.cos(theta)]
        ])
        return np.dot(point, m.T)

    def rot_y(self, theta, point):
        m = np.array([
            [np.cos(theta), 0, np.sin(theta)],
            [0, 1, 0],
            [-np.sin(theta), 0, np.cos(theta)]
        ])
        return np.dot(point, m.T)

    def rot_z(self, theta, point):
        m = np.array([
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta), np.cos(theta), 0],
            [0, 0, 1]
        ])
        return np.dot(point, m.T)
    
    def get_xyz_from_rgbd(self, x, y, d, base: Frame = Frame.kHead):
        h, w, c = self.image.shape
        rx = (x - w / 2) * 2 * d * np.tan(94 * np.pi / 180 / 2) / w
        ry = (y - h / 2) * 2 * d * np.tan(68 * np.pi / 180 / 2) / h

        wx = d 
        wy = -rx 
        wz = -ry

        tf = Transform()
        self.client.GetFrameTransform(Frame.kHead, base, tf)
        # print("RES: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f" % (
        #     tf.position.x, tf.position.y, tf.position.z,
        #     tf.orientation.x, tf.orientation.y, tf.orientation.z, tf.orientation.w
        # ))
        roll, pitch, yaw = self.euler_from_quaternion(tf.orientation)
        p = np.array([wx, wy, wz])
        p = self.rot_z(-yaw, p)
        p = self.rot_y(-pitch, p)
        p = self.rot_x(-roll, p)
        p[0] -= tf.position.x 
        p[1] -= tf.position.y 
        p[2] -= tf.position.z
        return p

    def transform(self, px, py, pz, roll, pitch, yaw, src: Frame = Frame.kHead, dst: Frame = Frame.kBody):
        res = Transform()
        self.client.GetFrameTransform(src, dst, res)
        print("RES: %.2f, %.2f, %.2f" % (
            res.position.x, res.position.y, res.position.z,
            res.orientation.x, res.orientation.y, res.orientation.z, res.orientation.w
        ))

    def update(self):
        self.func_update(self)
        # try:
        #     self.fps = 1.0 / (time.time() - self.last_update_time)
        #     self.func_update(self)
        # except Exception as e:
        #     print(e)
        # finally:
        #     self.last_update_time = time.time()
