import os, glob
import rclpy
from rclpy.node import Node
from cquick_interfaces.msg import TargetPosition

def detect(bgr):
    import cv2
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (0, 144, 66), (10, 255, 255)) | cv2.inRange(hsv, (170, 144, 66), (179, 255, 255))
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    valid = sorted((c for c in cnts if cv2.contourArea(c) >= 500), key=cv2.contourArea, reverse=True)
    if not valid: return False, 0.0, 0.0, 0.0
    M = cv2.moments(valid[0])
    if M["m00"] > 1e-6: return True, M["m10"]/M["m00"], M["m01"]/M["m00"], cv2.contourArea(valid[0])
    return False, 0.0, 0.0, 0.0

class Publisher(Node):
    def __init__(self):
        super().__init__("target_publisher")
        self.pub = self.create_publisher(TargetPosition, "/target_position", 10)
        self.timer = self.create_timer(0.5, self.tick) # 2Hz
        # 你的图片目录
        d = "img"
        self.files = sorted(glob.glob(os.path.join(d, "*.jpg")) + glob.glob(os.path.join(d, "*.png")))
        self.i = 0
        self.get_logger().info(f"找到 {len(self.files)} 张图片，开始发布...")

    def tick(self):
        import cv2
        msg = TargetPosition()
        if not self.files:
            self.get_logger().warn("图片目录为空")
            return
        path = self.files[self.i % len(self.files)]; self.i += 1
        img = cv2.imread(path)
        if img is not None:
            msg.detected, msg.u, msg.v, msg.area = detect(img)
            self.pub.publish(msg)
            self.get_logger().info(f"发送 {os.path.basename(path)}: detected={msg.detected}, u={msg.u:.1f}, v={msg.v:.1f}")

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Publisher())
    rclpy.shutdown()
