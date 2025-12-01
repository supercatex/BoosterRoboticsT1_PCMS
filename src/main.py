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
        if len(pose) == 0: continue 
        cx, cy = map(int , pose[0])
        if self.depth[cy][cx] == 0: continue
        rd = self.depth[cy][cx]
        # x1, y1, x2, y2 = w, h, -1, -1
        # for x, y in pose.cpu().numpy():
        #     if x == 0 and y == 0: continue
        #     x1, y1 = min(x1, x), min(y1, y)
        #     x2, y2 = max(x2, x), max(y2, y)
        # if x1 < x2:
        #     mx, my = (x1 + x2) // 2, (y1 + y2) // 2
        #     mx, my = map(int, (mx, my))
        #     mz = self.depth[my][mx]
        #     if mz > 0 and mz < rd:
        #         cx, cy, rd = mx, my, mz
        #     x1, y1, x2, y2 = map(int, (x1, y1, x2, y2))
        #     cv2.rectangle(self.image, (x1, y1), (x2, y2), (0, 255, 0), 2)
    # print(cx, cy, d)
    vx, vy, vz = 0.0, 0.0, 0.0
    pitch, yaw = 0.0, 0.0
    if cx != -1:
        rx = (cx - w / 2) * 2 * rd * np.tan(94 * np.pi / 180 / 2) / w
        ry = (cy - h / 2) * 2 * rd * np.tan(68 * np.pi / 180 / 2) / h
        cur_pitch = self.low_state.motor_state_serial[B1JointIndex.kHeadPitch].q
        cur_yaw = self.low_state.motor_state_serial[B1JointIndex.kHeadYaw].q
        pitch = cur_pitch + np.arctan2(ry, rd)
        yaw = cur_yaw + np.arctan2(-rx, rd)

        mz = 0.8
        ez = w // 2 - (cx - yaw * 400)
        pz = mz / (w // 2) * 5
        vz = pz * ez
        vz = max(-mz, min(vz, mz))

        mx = 1.2
        ed = rd - 600
        pd = mx / 2000
        vx = pd * ed 
        if rd == 0: vx = 0
        vx = max(-mx, min(vx, mx))

    # print("\rFPS: ", self.fps, end=" ")
    pitch = self.low_state.motor_state_serial[B1JointIndex.kHeadPitch].q
    yaw = self.low_state.motor_state_serial[B1JointIndex.kHeadYaw].q
    rx = (cx - w / 2) * 2 * rd * np.tan(94 * np.pi / 180 / 2) / w
    ry = (cy - h / 2) * 2 * rd * np.tan(68 * np.pi / 180 / 2) / h
    
    # wy, wx = ry, rx  
    # if abs(np.pi / 2 - pitch) > 0.01:
    #     wy = (rd - np.tan(np.pi / 2 - pitch) * ry) * np.sin(pitch)
    # if abs(np.pi / 2 - yaw) > 0.01:
    #     wx = (np.tan(np.pi / 2 - yaw) * rx - rd) * np.sin(yaw)
    # wd = rd / (np.cos(pitch) * np.cos(yaw)) + wy * np.tan(pitch) + wx * np.tan(yaw)

    # print("ORG: %.2f, %.2f, %.2f" % (rx, ry, rd))
    # print("PITCH: %.2f, YAW: %.2f" % (pitch, yaw))
    # print("NEW: %.2f, %.2f, %.2f" % (wx, wy, wd))

    # self.transform(vx, vy, vz, 0.0, pitch, yaw)

    # res = Transform()
    # res.position.x = 100
    # self.client.GetFrameTransform(Frame.kHead, Frame.kBody, res)
    # print("RES: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f" % (
    #     res.position.x, res.position.y, res.position.z,
    #     res.orientation.x, res.orientation.y, res.orientation.z, res.orientation.w
    # ))
    
    if cx != -1:
        p2 = self.get_xyz_from_rgbd(cx, cy, rd, Frame.kBody)
        print("%.2f %.2f %.2f" % (p2[0], p2[1], p2[2]))

    # self.cmd_vel(vx, vy, vz)
    # self.move_head(pitch, yaw)


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
