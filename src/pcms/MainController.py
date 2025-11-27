import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from booster_interface.msg import FallDownState, Odometer, LowState
from booster_robotics_sdk_python import RobotMode
from sensor_msgs.msg import Joy
from cv_bridge import CvBridge
from std_msgs.msg import Int32, String
from geometry_msgs.msg import Twist, Vector3
import time
import cv2


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
        self.pub_change_mode = self.create_publisher(Int32, "/pcms/ChangeMode", 10)
        self.pub_cmd_vel = self.create_publisher(Twist, "/pcms/cmd_vel", 10)
        self.pub_move_head = self.create_publisher(Vector3, "/pcms/MoveHead", 10)

        self.fps = 0.0
        self.last_update_time = time.time()
        self.timer = self.create_timer(self.update_time, self.update)

        time.sleep(1)
        self.func_setup(self)

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

    def change_mode(self, mode):
        msg = Int32()
        msg.data = mode
        self.pub_change_mode.publish(msg)

    def cmd_vel(self, vx, vy, vz):
        msg = Twist()
        msg.linear.x = vx 
        msg.linear.y = vy
        msg.angular.z = vz 
        self.pub_cmd_vel.publish(msg)

    def move_head(self, pitch, yaw):
        msg = Vector3()
        msg.y = pitch
        msg.z = yaw
        self.pub_move_head.publish(msg)

    def update(self):
        try:
            self.fps = 1.0 / (time.time() - self.last_update_time)
            self.func_update(self)
        except Exception as e:
            print(e)
        finally:
            self.last_update_time = time.time()
