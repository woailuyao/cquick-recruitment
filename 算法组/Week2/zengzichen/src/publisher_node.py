import rclpy
from rclpy.node import Node
from target_interfaces.msg import TargetPosition
import cv2
import os
import numpy as np

class VisionPublisher(Node):
    def __init__(self):
        super().__init__('vision_publisher_node')
        self.publisher_ = self.create_publisher(TargetPosition, '/target_position', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.get_logger().info('视觉发布节点已启动！')

    def timer_callback(self):
        msg = TargetPosition()
        image_path = os.path.expanduser("~/ros2_ws/images/lid.jpg")
        img = cv2.imread(image_path)

        if img is None:
            msg.detected = False
            msg.u = 0.0; msg.v = 0.0; msg.area = 0.0
            self.get_logger().error(f"读取失败: {image_path}")
            self.publisher_.publish(msg)
            return

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower1, upper1 = np.array([0, 150, 150]), np.array([10, 255, 255])
        lower2, upper2 = np.array([170, 150, 150]), np.array([180, 255, 255])
        mask = cv2.inRange(hsv, lower1, upper1) + cv2.inRange(hsv, lower2, upper2)

        kernel = np.ones((5, 5), np.uint8)
        opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(opening, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        b = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 500:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            a = w / h if h > 0 else 0
            if 0.5 < a < 2:
                b.append(cnt)
                
        if len(b) == 0:
            msg.detected = False
            msg.u = 0.0
            msg.v = 0.0
            msg.area = 0.0
            self.get_logger().warn('未检测到目标')
        else:
            c = max(b, key=cv2.contourArea)
            area_c = cv2.contourArea(c)
            M = cv2.moments(c)
            
            if M['m00'] != 0:
                u = int(M['m10'] / M['m00'])
                v = int(M['m01'] / M['m00'])
                
                msg.detected = True
                msg.u = float(u)
                msg.v = float(v)
                msg.area = float(area_c)
                
                self.get_logger().info(f'检测成功！坐标: u={u}, v={v}, area={int(area_c)}')

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VisionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
