import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from booster_interface.msg import FallDownState, Odometer, LowState
from sensor_msgs.msg import Joy
from cv_bridge import CvBridge
from std_msgs.msg import Int32, String
from geometry_msgs.msg import Twist, Vector3
import time
import cv2
import numpy as np
from ultralytics import YOLO


class Main(Node):
    def __init__(self):
        super().__init__(__class__.__name__)

        self.image, self.depth = None, None
        self.falldown, self.odometer,  = None, None
        self.low_state, self.joy, self.mode = None, None, None
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

        self.model_pose = YOLO("yolo11n-pose.pt")

        self.update_time = time.time()
        self.timer = self.create_timer(0.05, self.update)

        time.sleep(1)
        self.say("我準備好啦！")

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
            if self.image is None or self.depth is None: return

            info = f"""
     FPS: {1.0 / (time.time() - self.update_time)}
    Mode: {self.mode}
Falldown: {self.falldown}
Odometer: {self.odometer}
LowState: {self.low_state}
     Joy: {self.joy}
   Image: {self.image.shape}
   Depth: {self.depth.shape}
"""
            # print(info)

            h, w, c = self.image.shape
            poses = self.model_pose(self.image, verbose=False)[0]
            cx, cy, d = -1, -1, 10000
            for pose in poses.keypoints.xy:
                x1, y1, x2, y2 = w, h, -1, -1
                for x, y in pose.cpu().numpy():
                    if x == 0 and y == 0: continue
                    x1, y1 = min(x1, x), min(y1, y)
                    x2, y2 = max(x2, x), max(y2, y)
                if x1 < x2:
                    mx, my = (x1 + x2) // 2, (y1 + y2) // 2
                    mx, my = map(int, (mx, my))
                    mz = self.depth[my][mx]
                    if mz > 0 and mz < d:
                        cx, cy, d = mx, my, mz
                    x1, y1, x2, y2 = map(int, (x1, y1, x2, y2))
                    cv2.rectangle(self.image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            # print(cx, cy, d)
            vx, vy, vz = 0.0, 0.0, 0.0
            if cx != -1:
                mz = 1.2
                ez = w // 2 - cx
                pz = mz / (w // 2)
                vz = pz * ez
                vz = max(-mz, min(vz, mz))

                mx = 0.8
                ed = d - 600
                pd = mx / 2000
                vx = pd * ed 
                if d == 0: vx = 0
                vx = max(-mx, min(vx, mx))

            print(vx, vy, vz)
            self.cmd_vel(vx, vy, vz)
        except ZeroDivisionError as e:
            print(e)
        finally:
            self.update_time = time.time()


if __name__ == "__main__":
    rclpy.init()
    node = Main()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
