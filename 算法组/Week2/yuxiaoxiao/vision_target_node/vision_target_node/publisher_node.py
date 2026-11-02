import rclpy
from rclpy.node import Node
from target_interfaces.msg import TargetPosition
import cv2
import numpy as np

class TargetPublisher(Node):
    def __init__(self):
        super().__init__('target_publisher')
        self.publisher_ = self.create_publisher(TargetPosition, '/target_position', 10)
        # 把频率改成 2 秒一次，方便我们看清输出
        self.timer = self.create_timer(2.0, self.timer_callback)
        
        # 加载图片 (确保 test2.png 被复制到了 ~/ros2_ws/src/vision_target_node/ 目录下)
        self.image_path = "test2.png"
        self.img = cv2.imread(self.image_path)
        if self.img is None:
            self.get_logger().error("图片读取失败，请检查路径！")

    def timer_callback(self):
        msg = TargetPosition()
        
        if self.img is not None:
            # --- 这里就是咱们之前的 OpenCV 核心代码 ---
            hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
            lower_orange = np.array([5, 100, 100])
            upper_orange = np.array([22, 255, 255])
            mask = cv2.inRange(hsv, lower_orange, upper_orange)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if len(contours) > 0:
                c = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(c)
                if area > 500:
                    M = cv2.moments(c)
                    if M["m00"] != 0:
                        u = float(M["m10"] / M["m00"])
                        v = float(M["m01"] / M["m00"])
                        
                        # 将检测到的结果赋值给消息
                        msg.detected = True
                        msg.u = u
                        msg.v = v
                        msg.area = float(area)
                    else:
                        msg.detected = False
                else:
                    msg.detected = False
            else:
                msg.detected = False

        # 发布消息
        self.publisher_.publish(msg)
        
        # 终端打印
        if msg.detected:
            self.get_logger().info(f'发布: detected={msg.detected}, u={msg.u:.1f}, v={msg.v:.1f}, area={msg.area:.1f}')
        else:
            self.get_logger().info(f'发布: 未检测到目标 (detected=False)')

def main(args=None):
    rclpy.init(args=args)
    node = TargetPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': main()


