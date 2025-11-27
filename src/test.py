import rclpy
from pcms.MainController import MainController
from ultralytics import YOLO
import cv2
import time
from std_srvs.srv import Empty
from booster_robotics_sdk_python import B1HandIndex

def setup(self: MainController):
    print("SETUP BEGIN")
    self.walking_mode()
    self.switch_hand_end(True)
    time.sleep(1)
    self.move_hand_end(0.35, 0.25, 0.1, -1.57, -1.57, 0.0, 2000, B1HandIndex.kLeftHand)
    time.sleep(5)
    self.switch_hand_end(False)
    self.prepare_mode()
    print("SETUP END")
    

def update(self: MainController):
    print("\r", self.mode, end="")


if __name__ == "__main__":
    rclpy.init()
    node = MainController(setup, update)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
