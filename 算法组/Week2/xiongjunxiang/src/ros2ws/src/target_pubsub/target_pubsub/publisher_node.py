import rclpy
from rclpy.node import Node
from target_interfaces.msg import TargetPosition

from pathlib import Path
import argparse
import csv
import cv2 as cv
import numpy as np

LOWER = np.array([10, 170, 150])
UPPER = np.array([20, 255, 255])
MIN_AREA = 1000
KERNEL_SIZE = 7

image_path = Path(__file__).resolve().parent.parent / "images"

print (image_path)

class TargetPublisher(Node):
    def __init__(self):
        super().__init__('target_publisher')
        self.pub = self.create_publisher(TargetPosition, '/target_position', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)  # 0.5 s 一条
        self.count = 0

    def timer_callback(self):
        rows = []
        msg = TargetPosition()
        for p in image_path.glob(f"orange{str(self.count%3+1)}.jpg"):
            name = p.stem
            tmp = np.fromfile(str(p), dtype=np.uint8)
            img_bgr = cv.imdecode(tmp, 1)
            if img_bgr is None:
                print("读取失败")
                continue
            h, w = img_bgr.shape[:2]
            img_hsv = cv.cvtColor(img_bgr, cv.COLOR_BGR2HSV)
            mask = cv.inRange(img_hsv, LOWER, UPPER)
            kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (KERNEL_SIZE, KERNEL_SIZE))
            opened = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel, iterations=1)  # 去散点
            clean_mask = cv.morphologyEx(opened, cv.MORPH_CLOSE, kernel, iterations=2)  # 补孔

            contours, _ = cv.findContours(clean_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
            cands = []
            for contour in contours:
                area = cv.contourArea(contour)
                if area < MIN_AREA:
                    continue
                moments = cv.moments(contour)
                if moments["m00"] == 0:  # m00=0 的退化轮廓不能做除法，跳过
                    continue
                center_x = int(moments["m10"] / moments["m00"])
                center_y = int(moments["m01"] / moments["m00"])
                cands.append((area, center_x, center_y, cv.boundingRect(contour), contour))
            cands.sort(key=lambda t: -t[0])

            result = img_bgr.copy()
            for i, (area, cx, cy, (x, y, bw, bh), contour) in enumerate(cands):
                if i == 0:
                    cv.drawContours(result, [contour], -1, (0, 255, 255), 3)
                    cv.rectangle(result, (x, y), (x + bw, y + bh), (0, 0, 255), 3)
                    cv.circle(result, (cx, cy), 10, (0, 0, 255), -1)
                    cv.putText(result, f"u={cx}, v={cy}", (x, max(50, y - 15)),
                            cv.FONT_HERSHEY_SIMPLEX, 1.4, (0, 0, 255), 3)
                else:
                    cv.drawContours(result, [contour], -1, (128, 128, 128), 1)  # 次要候选画细线

            if not cands:
                msg.detected = False
                msg.area = 0.0
                msg.u = 0.0
                msg.v = 0.0
            else:
                area, cx, cy, box, _ = cands[0]
                msg.detected = True
                msg.area = float(area)
                msg.u = float(cx)
                msg.v = float(cy)
        

        self.get_logger().info(
            f'Publishing: detected={msg.detected}, '
            f'u={msg.u:.2f}, v={msg.v:.2f}, area={msg.area:.2f}'
        )
        self.pub.publish(msg)
        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    node = TargetPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()