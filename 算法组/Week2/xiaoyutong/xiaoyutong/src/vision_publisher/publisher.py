import rclpy
from rclpy.node import Node
from my_vision_interfaces.msg import TargetPosition
import cv2
import numpy as np
import os

class VisionPublisher(Node):
    def __init__(self):
        super().__init__('vision_publisher')
        self.publisher_ = self.create_publisher(TargetPosition, '/target_position', 10)
        img_path = os.path.expanduser("~/ros2_ws/src/test1.jpg")
        self.img = cv2.imread(img_path)
        self.lower_blue = np.array([90, 50, 50])
        self.upper_blue = np.array([130, 255, 255])
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = TargetPosition()
        if self.img is None:
            msg.detected = False
        else:
            hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, self.lower_blue, self.upper_blue)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            found = False
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 500:
                    x, y, w, h = cv2.boundingRect(cnt)
                    msg.detected = True
                    msg.u = float(x + w // 2)
                    msg.v = float(y + h // 2)
                    msg.area = float(area)
                    found = True
                    break
            if not found:
                msg.detected = False
                msg.u = 0.0
                msg.v = 0.0
                msg.area = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info(f'Sending: Detected={msg.detected}')

def main(args=None):
    rclpy.init(args=args)
    node = VisionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
