import rclpy
from rclpy.node import Node
from target_interfaces.msg import TargetPosition
import cv2 as cv
import numpy as np

class TargetPublisher(Node):
    def __init__(self):
        super().__init__('target_publisher')
        self.publisher_ = self.create_publisher(TargetPosition, '/target_position', 10)
        self.timer = self.create_timer(2.0, self.timer_callback) # 改成2秒发一次，方便看
        self.get_logger().info('真实视觉检测发布者已启动...')
        
        # 图片路径（注意这里用的是相对路径，确保你能跑通）
        self.img_path = '/home/gsyu/ros2_ws/src/lantern1.jpg'

    def timer_callback(self):
        msg = TargetPosition()
        
        # 读取图片并检测
        img = cv.imread(self.img_path)
        if img is None:
            self.get_logger().error('图片读取失败！')
            msg.detected = False
            msg.u = 0.0
            msg.v = 0.0
            msg.area = 0.0
            self.publisher_.publish(msg)
            return

        hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        
        # 红色阈值
        lower_red1 = np.array([0, 120, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 100])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv.inRange(hsv, lower_red2, upper_red2)
        mask = cv.bitwise_or(mask1, mask2)

        kernel = np.ones((3, 3), np.uint8)
        mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
        mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)

        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            target = max(contours, key=cv.contourArea)
            area = cv.contourArea(target)
            
            if area > 500:  # 面积过滤
                m = cv.moments(target)
                u = int(m["m10"] / m["m00"])
                v = int(m["m01"] / m["m00"])
                
                msg.detected = True
                msg.u = float(u)
                msg.v = float(v)
                msg.area = float(area)
            else:
                msg.detected = False
                msg.u = 0.0
                msg.v = 0.0
                msg.area = 0.0
        else:
            msg.detected = False
            msg.u = 0.0
            msg.v = 0.0
            msg.area = 0.0

        self.publisher_.publish(msg)
        self.get_logger().info(f'发送: detected={msg.detected}, u={msg.u}, v={msg.v}, area={msg.area}')

def main(args=None):
    rclpy.init(args=args)
    node = TargetPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
