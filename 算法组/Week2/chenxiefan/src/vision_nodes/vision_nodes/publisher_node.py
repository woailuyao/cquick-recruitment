#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2 as cv
import numpy as np
import os
import sys
import glob

import rclpy
from rclpy.node import Node

from vision_interfaces.msg import TargetPosition

#建立路径、导入图片
input_dir = os.path.expanduser('~/ros2_ws/data/images')

#定义处理函数（与 task2.py 的 process 相同的流程，只是把结果返回，不做显示）
def process(img_name):
    img_path = os.path.join(input_dir, img_name)
    img = cv.imread(img_path, 1)
    if img is None:
        print("读取失败")
        return None
    else:
        print("读取成功")

    #BGR转HSV,生成掩膜
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    lower = np.array([0, 60, 120])
    upper = np.array([5, 255, 255])
    mask1 = cv.inRange(hsv, lower, upper)
    lower = np.array([170, 60, 120])
    upper = np.array([179, 255, 255])
    mask2 = cv.inRange(hsv, lower, upper)

    #合并掩膜
    print("生成掩膜")
    mask = cv.bitwise_or(mask1, mask2)

    #执行开闭运算
    print("进行开闭运算")
    kernel = np.ones((10, 10), np.uint8)
    mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)
    mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)

    #识别面积最大的轮廓
    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    if len(contours) > 0:
        max_contour = max(contours, key=cv.contourArea)
        max_area = cv.contourArea(max_contour)
    else:
        print("未识别到目标")
        return None

    #寻找中心点
    print("寻找中心点")
    if max_area > 500:
        x, y, w, h = cv.boundingRect(max_contour)
        cx = x + w // 2
        cy = y + h // 2
        print(f"中心坐标: ({cx}, {cy}), 轮廓面积: {max_area:.2f}")
        return cx, cy, max_area
    else:
        print("未识别到目标")
        return None


class TargetPublisher(Node):
    def __init__(self):
        super().__init__('target_publisher')
        self.pub = self.create_publisher(TargetPosition, '/target_position', 10)
        self.imgs = ['image1.jpg', 'image2.jpg', 'image3.jpg', 'image4.jpg', 'image5.jpg']
        self.index = 0
        self.get_logger().info(
            f'数据来源: {input_dir}，共 {len(self.imgs)} 张图片按顺序循环，10 Hz 发布到 /target_position'
        )
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        msg = TargetPosition()
        img_name = self.imgs[self.index % len(self.imgs)]
        self.index += 1

        result = process(img_name)
        if result is None:
            msg.detected = False
            msg.u = 0.0
            msg.v = 0.0
            msg.area = 0.0
        else:
            cx, cy, max_area = result
            msg.detected = True
            msg.u = float(cx)
            msg.v = float(cy)
            msg.area = float(max_area)

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TargetPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
