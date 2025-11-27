import rclpy
from pcms.MainController import MainController
from ultralytics import YOLO
import cv2
import time
from std_srvs.srv import Empty

def setup(self: MainController):
    print("SETUP BEGIN")
    self.walking_mode()
    self.switch_hand_end(True)
    time.sleep(5)
    self.switch_hand_end(False)
    self.prepare_mode()
    print("SETUP END")
    

def update(self: MainController):
    print("\r", self.mode, end="")


if __name__ == "__main__":
    try:
        rclpy.init()
        node = MainController(setup, update)
        rclpy.spin(node)
    except ZeroDivisionError as e:
        print(e)
    finally:
        node.destroy_node()
        rclpy.shutdown()
