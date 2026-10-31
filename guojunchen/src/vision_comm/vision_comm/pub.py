import rclpy
from rclpy.node import Node
from vision_interfaces.msg import TargetPosition
import cv2
import numpy as np
import os

class TargetPublisher(Node):
    def __init__(self):
        super().__init__('target_publisher')
        self.pub = self.create_publisher(TargetPosition, '/target_position', 10)
        self.declare_parameter('h_min', 35)
        self.declare_parameter('h_max', 77)
        self.declare_parameter('min_area', 500)
        # 🌟 指定测试图片列表
        self.image_paths = [
            '/mnt/c/guojunchen/ceshi1.jpg',
            '/mnt/c/guojunchen/ceshi2.webp',
            '/mnt/c/guojunchen/ceshi3.png'
        ]
        
        self.image_index = 0 
        
        if len(self.image_paths) > 0:
            self.get_logger().info(f"准备测试指定的 {len(self.image_paths)} 张图片...")
            self.timer = self.create_timer(1.0, self.publish_msg)
        else:
            self.get_logger().error("没有指定任何图片！")

    def publish_msg(self):
        if self.image_index >= len(self.image_paths):
            self.timer.cancel()
            self.get_logger().info("=== 指定的图片已全部测试完毕，停止发布 ===")
            return
            
        current_img_path = self.image_paths[self.image_index]
        
        if not os.path.exists(current_img_path):
            self.get_logger().error(f"找不到图片: {current_img_path}，跳过...")
            self.image_index += 1
            return

        self.get_logger().info(f"--- 正在处理: {os.path.basename(current_img_path)} ---")

        h_min = self.get_parameter('h_min').value
        h_max = self.get_parameter('h_max').value
        min_area = self.get_parameter('min_area').value
        img = cv2.imread(current_img_path) 
        
        if img is not None:
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, np.array([h_min, 43, 46]), np.array([h_max, 255, 255]))
            
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            valid = [cnt for cnt in contours if cv2.contourArea(cnt) > min_area]
            
            if len(valid) > 0:
                for i, cnt in enumerate(valid):
                    msg = TargetPosition()
                    msg.detected = True
                    msg.area = float(cv2.contourArea(cnt))
                    
                    M = cv2.moments(cnt)
                    if M["m00"] != 0:
                        msg.u = float(int(M["m10"] / M["m00"]))
                        msg.v = float(int(M["m01"] / M["m00"]))
                    else:
                        x, y, w, h = cv2.boundingRect(cnt)
                        msg.u = float(x + w // 2)
                        msg.v = float(y + h // 2)
                        
                    self.pub.publish(msg)
                    self.get_logger().info(f'  -> 发送目标 {i+1}: u={msg.u}, v={msg.v}')
            else:
                msg = TargetPosition()
                msg.detected = False
                msg.u = msg.v = msg.area = 0.0
                self.pub.publish(msg)
                self.get_logger().warn("  -> 该图未检测到目标")
                
        self.image_index += 1 

def main():
    rclpy.init()
    rclpy.spin(TargetPublisher())
    rclpy.shutdown()

if __name__ == '__main__':
    main()