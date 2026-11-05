import rclpy
from rclpy.node import Node
from interfaces.msg import TargetPosition
import cv2
import numpy as np
import os

class VisionPublisher(Node):
    def __init__(self):
        super().__init__('vision_publisher')
        self.publisher_ = self.create_publisher(TargetPosition, '/target_position', 10)
        self.image_path = os.path.expanduser('~/ros2_ws/src/vision_pkg/images/soccer1.jpg')
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('视觉发布节点已启动！')

    def timer_callback(self):
        msg = TargetPosition()
        if not os.path.exists(self.image_path):
            return
        img = cv2.imread(self.image_path)
        if img is None:
            return

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower = np.array([0, 0, 150])
        upper = np.array([180, 40, 255])
        mask = cv2.inRange(hsv, lower, upper)
        kernel = np.ones((25, 25), np.uint8)
        mask_clean = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        valid_contours = [c for c in contours if cv2.contourArea(c) > 5000]
        if len(valid_contours) > 0:
            cnt = max(valid_contours, key=cv2.contourArea)
            area = cv2.contourArea(cnt)
            M = cv2.moments(cnt)
            u = int(M["m10"] / M["m00"]) if M["m00"] != 0 else 0
            v = int(M["m01"] / M["m00"]) if M["m00"] != 0 else 0
            msg.detected = True
            msg.u = float(u)
            msg.v = float(v)
            msg.area = float(area)
            self.get_logger().info(f'发布: u={u}, v={v}, area={int(area)}')
        else:
            msg.detected = False
            self.get_logger().info('未检测到目标')
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VisionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
