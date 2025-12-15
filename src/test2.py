import rclpy
from pcms.MainController import MainController
from ultralytics import YOLO
import cv2
import numpy as np
import time
from std_srvs.srv import Empty
from booster_robotics_sdk_python import B1HandIndex, B1JointIndex, Frame


def setup(self: MainController):
    self.model_pose = YOLO("yolo11n-football.pt")
    self.say("我準備好啦！")


def update(self: MainController):
    h, w, c = self.image.shape
    res = self.model_pose(self.image, verbose=False, imgsz=(736, 1280))[0]
    cx, cy, rd = -1, -1, 10000
    for conf, box in zip(res.boxes.conf, res.boxes.xyxy):
        x1, y1, x2, y2 = map(int, box)
        x, y = (x1 + x2) // 2, (y1 + y2) // 2
        if cx == -1 or self.depth[y][x] < rd:
            cx, cy, rd = x, y, self.depth[y][x]
        cv2.rectangle(self.image, (x1, y1), (x2, y2), (0, 255, 0), 2)
    # cv2.imshow("image", self.image)
    # cv2.waitKey(1)
        
    # if cx != -1:
        # rx = (cx - w / 2) * 2 * rd * np.tan(90 * np.pi / 180 / 2) / w
        # ry = (cy - h / 2) * 2 * rd * np.tan(65 * np.pi / 180 / 2) / h
        
        # pitch = self.low_state.motor_state_serial[B1JointIndex.kHeadPitch].q + np.arctan2(ry, rd)
        # yaw = self.low_state.motor_state_serial[B1JointIndex.kHeadYaw].q + np.arctan2(-rx, rd)
        # self.move_head(pitch, yaw)

        # print("%.2f: (%.2f, %.2f, %.2f) (%.4f, %.4f)" % (conf, rx, ry, rd, pitch, yaw))
    vx, vy, vz = 0.0, 0.0, 0.0
    pitch, yaw = 0.5, 0.0
    if cx != -1:
        hx, hy, hz = self.get_xyz_from_rgbd(cx, cy, rd, Frame.kHead)
        cur_pitch = self.low_state.motor_state_serial[B1JointIndex.kHeadPitch].q
        cur_yaw = self.low_state.motor_state_serial[B1JointIndex.kHeadYaw].q
        pitch = cur_pitch + np.arctan2(-hz, rd)
        yaw = cur_yaw + np.arctan2(hy, rd)
        # print(rx, ry, rz)

        bx, by, bz = self.get_xyz_from_rgbd(cx, cy, rd, Frame.kBody)
        print(bx, by, bz)

        mz = 0.8
        vz = 0.005 * (by - 0.0)
        vz = max(-mz, min(vz, mz))

        mx = 1.2
        vx = 0.0007 * (bx - 600)
        if rd == 0: vx = 0.0
        vx = max(-mx, min(vx, mx))

    # vx = 0.0
    self.cmd_vel(vx, vy, vz)
    self.move_head(pitch, yaw)


if __name__ == "__main__":
    rclpy.init()
    node = MainController(setup, update, 0.05)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
