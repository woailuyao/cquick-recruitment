import rclpy
from rclpy.node import Node
from cquick_interfaces.msg import TargetPosition
import cv2
import numpy as np

class VisionPublisher(Node):
    def __init__(self):
        super().__init__('vision_publisher')
        self.publisher_ = self.create_publisher(TargetPosition, '/target_position', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = TargetPosition()
        
        # 拿任务一的第一张图来测试
        img = cv2.imread('src/cquick_vision/cquick_vision/task1_01.jpg')
        
        if img is None:
            self.get_logger().error("图片读取失败")
            return

        #  HSV 找橙色笔逻辑
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([5, 120, 50]), np.array([22, 255, 255]))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        valid_contours = [c for c in contours if cv2.contourArea(c) > 2500]

        if not valid_contours:
            msg.detected = False
            msg.u = 0.0
            msg.v = 0.0
            msg.area = 0.0
        else:
            best_c = max(valid_contours, key=cv2.contourArea)
            M = cv2.moments(best_c)
            msg.detected = True
            msg.u = float(M["m10"] / M["m00"])
            msg.v = float(M["m01"] / M["m00"])
            msg.area = float(cv2.contourArea(best_c))

        self.publisher_.publish(msg)
        self.get_logger().info(f'发布: u={msg.u:.1f}, v={msg.v:.1f}, area={msg.area:.1f}')

def main(args=None):
    rclpy.init(args=args)
    node = VisionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()