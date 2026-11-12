import cv2
import numpy as np
import os
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from cquick_interfaces.msg import TargetPosition

# 最小轮廓面积（像素平方），小于该值视为噪点
MIN_AREA = 100.0

class TargetPublisher(Node):
    def __init__(self):
        super().__init__('target_publisher')
        self.publisher_ = self.create_publisher(TargetPosition, '/target_position', 10)
        self.timer = self.create_timer(1.0, self.timer_callback) 
        
        # 测试图随包安装到 share 目录，不写死绝对路径
        self.image_path = os.path.join(
            get_package_share_directory('cquick_vision'), "test3_orange.jpg")

    def timer_callback(self):
        msg = TargetPosition()
        
        image = cv2.imread(self.image_path)
        if image is None:
            self.get_logger().error("读取失败！")
            msg.detected = False
            msg.u, msg.v, msg.area = 0.0, 0.0, 0.0
            self.publisher_.publish(msg)
            return

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        lower_green = np.array([0, 50, 50])
        upper_green = np.array([15, 255, 255])
        mask = cv2.inRange(hsv, lower_green, upper_green)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 先按最小面积过滤噪点，再取面积最大的作为目标
        candidates = [c for c in contours if cv2.contourArea(c) >= MIN_AREA]

        if candidates:
            largest_contour = max(candidates, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            M=cv2.moments(largest_contour)
            if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    
                    msg.detected = True
                    msg.u = float(cx)
                    msg.v = float(cy)
                    msg.area = float(area)
            else:
                    msg.detected = False
                    msg.u, msg.v, msg.area = 0.0, 0.0, 0.0
        else:
                msg.detected = False
                msg.u, msg.v, msg.area = 0.0, 0.0, 0.0

        self.publisher_.publish(msg)
        
        if msg.detected:
            self.get_logger().info(f'检测成功: u={msg.u}, v={msg.v}, area={msg.area}')
        else:
            self.get_logger().info('未检测到目标 (detected=False)')

def main(args=None):
    rclpy.init(args=args)
    node = TargetPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
