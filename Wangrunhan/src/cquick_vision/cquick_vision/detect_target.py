#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from cquick_interfaces.msg import TargetPosition
from pathlib import Path

class TargetDetector(Node):
    def __init__(self):
        super().__init__('target_detector')
        self.pub = self.create_publisher(TargetPosition, '/target_position', 10)
        
        # 绿色阈值
        self.lower = np.array([25, 30, 30])
        self.upper = np.array([100, 255, 255])
        self.min_area = 50
        
        # 最稳妥的本地绝对路径
        self.image_dir = Path.home() / 'cquick_ws/images'
        self.output_dir = Path.home() / 'cquick_ws'
        
        self.image_paths = sorted(self.image_dir.glob('*.jpg')) + sorted(self.image_dir.glob('*.png'))
        self.idx = 0
        if not self.image_paths:
            self.get_logger().warn(f'文件夹里没有图片！请检查路径: {self.image_dir}')

    def process_next(self):
        msg = TargetPosition()
        if self.idx >= len(self.image_paths):
            self.get_logger().info('所有图片处理完毕')
            return

        img_path = self.image_paths[self.idx]
        frame = cv2.imread(str(img_path))
        if frame is None:
            msg.detected = False
            msg.u = msg.v = msg.area = 0.0
            self.pub.publish(msg)
            self.idx += 1
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower, self.upper)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        valid = [c for c in contours if cv2.contourArea(c) >= self.min_area]
        annotated = frame.copy()

        if not valid:
            msg.detected = False
            msg.u = msg.v = msg.area = 0.0
            self.get_logger().info(f'{img_path.name}: 未检测到目标')
            cv2.putText(annotated, 'No target', (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        else:
            target = max(valid, key=cv2.contourArea)
            M = cv2.moments(target)
            area = cv2.contourArea(target)
            cx = int(M['m10'] / M['m00']) if M['m00'] != 0 else 0
            cy = int(M['m01'] / M['m00']) if M['m00'] != 0 else 0
            msg.detected = True
            msg.u = float(cx)
            msg.v = float(cy)
            msg.area = float(area)
            cv2.drawContours(annotated, [target], -1, (0, 255, 0), 2)
            cv2.circle(annotated, (cx, cy), 5, (0, 0, 255), -1)
            cv2.putText(annotated, f'u={cx}, v={cy}', (cx + 10, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            self.get_logger().info(f'{img_path.name}: detected=True, u={cx}, v={cy}, area={area:.0f}')

        out_path = self.output_dir / f'annotated_{img_path.name}'
        cv2.imwrite(str(out_path), annotated)
        self.pub.publish(msg)
        self.idx += 1

def main(args=None):
    rclpy.init(args=args)
    node = TargetDetector()
    node.create_timer(1.0, node.process_next)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
