import rclpy
from pcms.MainController import MainController
from ultralytics import YOLO
import cv2


def setup(self: MainController):
    self.model_pose = YOLO("yolo11n-pose.pt")
    self.say("我準備好啦！")


def update(self: MainController):
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

    # print(vx, vy, vz)
    print("\rFPS: ", self.fps, end="")
    self.cmd_vel(vx, vy, vz)


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
