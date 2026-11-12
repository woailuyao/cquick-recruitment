import rclpy
from rclpy.node import Node
from my_interface.msg import TargetPosition
import cv2
import numpy as np

class TargetPublisher(Node):
    def __init__(self):
        super().__init__('target_publisher')
        self.publisher_ = self.create_publisher(TargetPosition, '/target_position', 10)
        
        self.img_path ='/mnt/d/camera_ws/live_frame.jpg'
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = TargetPosition()
        img = cv2.imread(self.img_path)
        
        if img is None:
            self.get_logger().error("图片读取失败！")
            msg.detected = False
            msg.u, msg.v, msg.area = 0.0, 0.0, 0.0
            self.publisher_.publish(msg)
            return

        #视觉代码（提取蓝色）
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([100, 80, 80]), np.array([130, 255, 255]))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if len(contours) > 0:
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)
            if area > 50:
                M = cv2.moments(largest)
                if M['m00'] != 0:
                    msg.detected = True
                    msg.u = float(int(M['m10'] / M['m00']))
                    msg.v = float(int(M['m01'] / M['m00']))
                    msg.area = float(area)
                    self.get_logger().info(f'发送真实坐标: u={msg.u}, v={msg.v}, area={msg.area}')
                    self.publisher_.publish(msg)
                    return
        
        msg.detected = False
        msg.u, msg.v, msg.area = 0.0, 0.0, 0.0
        self.get_logger().warn('未检测到目标')
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TargetPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()