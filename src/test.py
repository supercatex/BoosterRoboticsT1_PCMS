import rclpy
from pcms.MainController import MainController
from ultralytics import YOLO
import cv2
import numpy as np
import time
from std_srvs.srv import Empty
from booster_robotics_sdk_python import B1HandIndex, B1JointIndex

def setup(self: MainController):
    self.model_pose = YOLO("yolo11n-pose.pt")
    self.say("我準備好啦！")
    self.is_waving_hand = False

def update(self: MainController):
    h, w, c = self.image.shape
    poses = self.model_pose(self.image, verbose=False, imgsz=(736, 1280))[0]
    cx, cy, rd = -1, -1, 10000
    cx8, cy8 = -1, -1
    for pose in poses.keypoints.xy:
        if len(pose) == 0: continue
        x0, y0 = map(int, pose[0])
        x8, y8 = map(int, pose[8])
        if (x0 != 0 and y0 != 0) and (cx == -1 or self.depth[y0][x0] < rd):
            cx, cy, rd = x0, y0, self.depth[y0][x0]
            cx8, cy8 = x8, y8
        
    if cx != -1:
        rx = (cx - w / 2) * 2 * rd * np.tan(90 * np.pi / 180 / 2) / w
        ry = (cy - h / 2) * 2 * rd * np.tan(65 * np.pi / 180 / 2) / h
        
        pitch = self.low_state.motor_state_serial[B1JointIndex.kHeadPitch].q + np.arctan2(ry, rd)
        yaw = self.low_state.motor_state_serial[B1JointIndex.kHeadYaw].q + np.arctan2(-rx, rd)
        self.move_head(pitch, yaw)

        print(cy, cy8)
        if cx8 != 0 and cy8 != 0 and cy8 < cy:
            if self.is_waving_hand == False:
                self.switch_hand_end(False)
                self.wave_hand(True)
            self.is_waving_hand = True
        else:
            if self.is_waving_hand == True:
                self.switch_hand_end(False)
                self.wave_hand(False)
            self.is_waving_hand = False


if __name__ == "__main__":
    rclpy.init()
    node = MainController(setup, update)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
