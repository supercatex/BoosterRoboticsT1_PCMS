import rclpy
from ultralytics import YOLO
from pcms.MainController import MainController
import cv2


def setup(self: MainController):
    self.model = YOLO("yolo11n-pose.pt")
    # self.model = YOLO("/home/booster/Workspace/Booster_T1_3v3_Demo/src/vision/model/best_orin_10.3.engine", task="detect")
    # self.model.classname = ["Ball", "Goalpost", "Person", "LCross", "TCross", "XCross", "PenaltyPoint", "Opponent", "BRMarker"]

def update(self: MainController):
    if self.image is None: return 

    res = self.model.predict(self.image, verbose=False)[0]
    for cls, box in zip(res.boxes.cls, res.boxes.xyxy):
        cls = int(cls)
        x1, y1, x2, y2 = map(int, box)
        cv2.rectangle(self.image, (x1, y1), (x2, y2), (0, 255, 0), 2)
    print(res.boxes.cls)
    cv2.imwrite("demo.jpg", self.image)

if __name__ == "__main__":
    rclpy.init()
    node = MainController(setup, update)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
