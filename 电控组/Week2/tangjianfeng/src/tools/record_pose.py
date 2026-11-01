#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
轨迹记录器：订阅 /turtle1/pose，把乌龟真实走过的 (x, y) 存成 CSV。
用法: ./record_pose.py [输出路径，默认当前目录下的 pose.csv]
"""
import csv
import sys
import time

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose


class PoseRecorder(Node):
    def __init__(self, path):
        super().__init__('pose_recorder')
        self.f = open(path, 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow(['t', 'x', 'y', 'theta'])
        self.t0 = time.time()
        self.create_subscription(Pose, '/turtle1/pose', self.cb, 20)
        self.get_logger().info(f'记录轨迹到 {path}')

    def cb(self, msg):
        self.w.writerow([f'{time.time() - self.t0:.3f}',
                         f'{msg.x:.4f}', f'{msg.y:.4f}',
                         f'{msg.theta:.4f}'])

    def close(self):
        self.f.flush()
        self.f.close()


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else 'pose.csv'
    rclpy.init()
    node = PoseRecorder(path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
