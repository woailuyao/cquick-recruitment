import rclpy
from rclpy.node import Node
from target_interfaces.msg import TargetPosition
import cv2
import numpy as np

class PublisherNode(Node):
    def __init__(self):
        super().__init__('target_publisher')
        self.pub = self.create_publisher(TargetPosition, '/target_position', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        # 轮流发清晰图和干扰图
        self.image_paths = [
            "/home/camellia/cquick-recruitment/视觉组/Week2/yuanjiaxin/img3.jpg",
            "/home/camellia/cquick-recruitment/视觉组/Week2/yuanjiaxin/img1.jpg"
        ]
        self.index = 0

    def timer_callback(self):
        img_path = self.image_paths[self.index % len(self.image_paths)]
        self.index += 1
        
        msg = TargetPosition()
        img = cv2.imread(img_path)
        
        found = False
        if img is not None:
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, np.array([35, 43, 46]), np.array([85, 255, 255]))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                contours = sorted(contours, key=cv2.contourArea, reverse=True)
                for c in contours:
                    area = cv2.contourArea(c)
                    if 500 <= area <= 300000:
                        M = cv2.moments(c)
                        if M["m00"] != 0:
                            msg.detected = True
                            msg.u = float(int(M["m10"] / M["m00"]))
                            msg.v = float(int(M["m01"] / M["m00"]))
                            msg.area = float(area)
                            found = True
                            break
                            
        if not found:
            msg.detected = False
            msg.u = 0.0
            msg.v = 0.0
            msg.area = 0.0

        self.pub.publish(msg)
        img_name = img_path.split('/')[-1]
        self.get_logger().info(f'发布 {img_name}: detected={msg.detected}, u={msg.u}, v={msg.v}, area={msg.area}')

def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()