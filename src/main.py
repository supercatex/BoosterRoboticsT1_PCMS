import rclpy
from pcms.MainController import MainController
from ultralytics import YOLO
import cv2
import numpy as np
from booster_robotics_sdk_python import B1JointIndex, Transform, Frame


def setup(self: MainController):
    self.model_pose = YOLO("yolo11n-pose.pt")
    self.say("我準備好喇！")


def update(self: MainController):
    h, w, c = self.image.shape
    poses = self.model_pose(self.image, verbose=False)[0]
    cx, cy, rd = -1, -1, 10000
    for pose in poses.keypoints.xy:
        # if len(pose) == 0: continue 
        # cx, cy = map(int , pose[0])

        # if self.depth[cy][cx] == 0: continue
        # rd = self.depth[cy][cx]
        x1, y1, x2, y2 = w, h, -1, -1
        for x, y in pose.cpu().numpy():
            if x == 0 and y == 0: continue
            x1, y1 = min(x1, x), min(y1, y)
            x2, y2 = max(x2, x), max(y2, y)
        if x1 < x2:
            mx, my = (x1 + x2) // 2, (y1 + y2) // 2
            mx, my = map(int, (mx, my))
            mz = self.depth[my][mx]
            if mz > 0 and mz < rd:
                cx, cy, rd = mx, my, mz
            x1, y1, x2, y2 = map(int, (x1, y1, x2, y2))
            cv2.rectangle(self.image, (x1, y1), (x2, y2), (0, 255, 0), 2)
    print(cx, cy, rd)

    vx, vy, vz = 0.0, 0.0, 0.0
    pitch, yaw = 0.0, 0.0
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
        if rd == 0: vx = 0
        vx = max(-mx, min(vx, mx))

    self.cmd_vel(vx, vy, vz)
    pitch = 0.0
    self.move_head(pitch, yaw)


if __name__ == "__main__":
    try:
        rclpy.init()
        node = MainController(setup, update)
        rclpy.spin(node)
    except Exception as e:
        print(e)
    finally:
        node.destroy_node()
        rclpy.shutdown()
