import rclpy
from rclpy.node import Node
from custom_interfaces.msg import TargetPosition
import cv2
import numpy as np

class VisionPublisher(Node):
    def __init__(self):
        super().__init__('vision_publisher')
        # 创建发布者，话题名为 /target_position
        self.publisher_ = self.create_publisher(TargetPosition, 'target_position', 10)
        # 每 1 秒执行一次
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.img_path = '/home/yangzhexiong/vision_ws/week1/images/color1.jpg'
        self.get_logger().info('发布节点已启动...')

    def timer_callback(self):
        msg = TargetPosition()
        img = cv2.imread(self.img_path)
        
        if img is not None:
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, np.array([0,100,100]), np.array([10,255,255]))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 500:
                    x, y, w, h = cv2.boundingRect(cnt)
                    msg.detected = True
                    msg.u = float(x + w // 2)
                    msg.v = float(y + h // 2)
                    msg.area = float(area)
                    break
            else:
                msg.detected = False
                msg.u = 0.0
                msg.v = 0.0
                msg.area = 0.0
                
            self.publisher_.publish(msg)
            self.get_logger().info(f'发送: detected={msg.detected}, u={msg.u}, v={msg.v}')

def main(args=None):
    rclpy.init(args=args)
    node = VisionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
