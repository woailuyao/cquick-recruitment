#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_interfaces.msg import TargetPosition
import cv2
import numpy as np

class VisionPublisher(Node):
    def __init__(self):
        super().__init__('vision_publisher')
        # 创建发布者，话题名 /target_position
        self.publisher_ = self.create_publisher(TargetPosition, '/target_position', 10)
        # 每 0.5 秒触发一次
        self.timer = self.create_timer(0.5, self.timer_callback)
        
        # 读取图片路径 (识别哪张图就删除注释)
        #self.image_path = "/home/chensiyou29/ros2_ws/test1.jpg"
        self.image_path = "/home/chensiyou29/ros2_ws/test2.jpg"
        #self.image_path = "/home/chensiyou29/ros2_ws/test3.jpg"
        
        

        self.get_logger().info("视觉检测发布节点已启动！")

    def timer_callback(self):
        frame = cv2.imread(self.image_path)
        msg = TargetPosition()
        msg.u = 0.0
        msg.v = 0.0
        msg.area = 0.0
        
        if frame is None:
            self.get_logger().error(f"找不到图片: {self.image_path}")
            return

        # 转换为 HSV 空间
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # 找出绿色区域（草地）
        lower_green = np.array([35, 40, 40])
        upper_green = np.array([85, 255, 255])
        #lower_green = np.array([35, 100, 100])
        #upper_green = np.array([85, 255, 255])


        green_mask = cv2.inRange(hsv, lower_green, upper_green)
        
        # 反转掩膜：除了绿色，其他的都留下来（找足球）
        mask = cv2.bitwise_not(green_mask)
        kernel=np.ones((5,5),np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # 查找轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        msg.detected = False

        if len(contours) > 0:
            valid_contours = []
            for c in contours:
                x, y, w, h = cv2.boundingRect(c)
                if w < frame.shape[1] * 0.5 and h < frame.shape[0] * 0.5:
                    valid_contours.append(c)
            if len(valid_contours) > 0:
                largest_contour = max(valid_contours, key=cv2.contourArea)
                area = cv2.contourArea(largest_contour)
        
                if area > 100:
                    M = cv2.moments(largest_contour)
                    if M["m00"] != 0:
                        u = float(M["m10"] / M["m00"])
                        v = float(M["m01"] / M["m00"])
                
                        msg.detected = True
                        msg.u = u
                        msg.v = v
                        msg.area = float(area)
                        self.get_logger().info(f'检测到目标: u={u:.1f}, v={v:.1f}, area={area:.1f}')
                        x, y, w, h = cv2.boundingRect(largest_contour)
                        
                        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)
                        cv2.circle(frame, (int(u), int(v)), 5, (0, 0, 255), -1)
                        cv2.putText(frame, f"u={u:.1f}, v={v:.1f}", (int(u) + 10, int(v) - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                        cv2.putText(frame, "Name: chensiyou", (50, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)


                        
                        cv2.imwrite("result_image.jpg", frame)
                    else:
                        self.get_logger().info('未检测到目标 (m00异常)')
                else:
                    self.get_logger().info('未检测到目标 (面积太小)')
            else:
                self.get_logger().info('未检测到目标 (只有背景)')
        else:
            self.get_logger().info('未检测到目标 (无轮廓)')                
        self.publisher_.publish(msg)
                        
            

        
        
# 分隔线，下面是主函数
def main(args=None):
    rclpy.init(args=args)
    node = VisionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
