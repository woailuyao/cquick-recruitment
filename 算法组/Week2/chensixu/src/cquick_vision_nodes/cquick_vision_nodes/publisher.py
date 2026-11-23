import rclpy
from rclpy.node import Node
from cquick_interfaces.msg import TargetPosition
import cv2
import numpy as np
import os

class VisionPublisher(Node):
    def __init__(self):
        super().__init__('vision_publisher')
        self.publisher_ = self.create_publisher(TargetPosition, '/target_position', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.image_path = os.path.expanduser("~/ros2_ws/ball1.jpg")

        # 【加分项】声明 ROS2 参数
        self.declare_parameter('lower_h', 20)
        self.declare_parameter('upper_h', 35)
        self.declare_parameter('min_area', 300.0)

    def timer_callback(self):
        # 【加分项】每次循环都动态读取最新参数
        lower_h = self.get_parameter('lower_h').value
        upper_h = self.get_parameter('upper_h').value
        min_area = self.get_parameter('min_area').value

        msg = TargetPosition()
        img = cv2.imread(self.image_path)
        if img is None:
            self.get_logger().error("图片读取失败")
            return

        img = cv2.resize(img, (640, 480))
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        lower = np.array([lower_h, 100, 100])
        upper = np.array([upper_h, 255, 255])
        mask = cv2.inRange(hsv, lower, upper)
        
        kernel = np.ones((15, 15), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if len(contours) > 0:
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            
            if area > min_area:
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    msg.detected = True
                    msg.u = float(cx)
                    msg.v = float(cy)
                    msg.area = float(area)
                    self.publisher_.publish(msg)
                    self.get_logger().info(f'✅ 检测: u={msg.u}, v={msg.v}, area={msg.area}')
                    return

        msg.detected = False
        self.publisher_.publish(msg)
        self.get_logger().info('❌ 未检测到目标, 发送默认值')

def main(args=None):
    rclpy.init(args=args)
    node = VisionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()