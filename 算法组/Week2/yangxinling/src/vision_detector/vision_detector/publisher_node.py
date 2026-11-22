import rclpy
from rclpy.node import Node
from vision_interfaces.msg import TargetPosition
import cv2
import numpy as np
import os

class VisionPublisher(Node):
    def __init__(self):
        super().__init__('vision_publisher')
        self.publisher_ = self.create_publisher(TargetPosition, '/target_position', 10)
        # 定时器，每秒发布一次
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        # 获取当前脚本的绝对路径，确保能找到图片
        self.current_dir = os.path.dirname(os.path.abspath(__file__))
        
        self.image_path = os.path.join(self.current_dir, 'IMG3.jpg')
        
        # HSV 参数
        self.lower_white = np.array([0, 0, 120])
        self.upper_white = np.array([180, 40, 255])
        
        self.get_logger().info("视觉发布节点已启动，正在读取图片...")

    def timer_callback(self):
        img = cv2.imread(self.image_path)
        msg = TargetPosition()
        
        if img is None:
            self.get_logger().error(f"无法读取图片: {self.image_path}")
            return

        # 1. 转 HSV
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        # 2. 生成掩膜
        mask = cv2.inRange(hsv, self.lower_white, self.upper_white)
        
        # 3. 形态学清理
        kernel = np.ones((5,5), np.uint8) 
        mask_clean = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        kernel = np.ones((7,7), np.uint8) 
        mask_clean = cv2.morphologyEx(mask_clean, cv2.MORPH_CLOSE, kernel)
        
        # 4. 找轮廓
        contours, _ = cv2.findContours(mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # 5. 筛选目标
        target_found = False
        if len(contours) > 0:
            contours = sorted(contours, key=cv2.contourArea, reverse=True)
            
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < 1000:
                    break
                    
                x, y, w, h = cv2.boundingRect(cnt)
                aspect_ratio = float(w) / h
                
                # 如果长宽比符合球体特征
                if 0.75 < aspect_ratio < 1.25:
                    # 计算中心点
                    M = cv2.moments(cnt)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        
                        # 填充 ROS2 消息
                        msg.detected = True
                        msg.u = float(cx)
                        msg.v = float(cy)
                        msg.area = float(area)
                        
                        self.get_logger().info(f'发送: 找到球! u={cx}, v={cy}, area={area:.1f}')
                        target_found = True
                        break # 找到最符合的就结束寻找
        
        # 如果没找到目标，设置为 False，并将坐标归零
        if not target_found:
            msg.detected = False
            msg.u = 0.0
            msg.v = 0.0
            msg.area = 0.0
            self.get_logger().info('发送: 未检测到目标球')
            
        # 发布消息
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VisionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()