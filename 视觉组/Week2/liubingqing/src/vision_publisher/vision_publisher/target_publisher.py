import rclpy
from rclpy.node import Node
from vision_interfaces.msg import TargetPosition
import os
import glob
import cv2
import numpy as np
class TargetPublisher(Node):
    def __init__(self):
        super().__init__('target_publisher')
        self.publisher_ = self.create_publisher(TargetPosition, '/target_position', 10)
        self.timer = self.create_timer(1.0, self.publish_detection)
        self.images = self._load_images()
        self.idx = 0
        self.count = 0
        self.get_logger().info(f"发布者已启动，共加载 {len(self.images)} 张图片")
    def _load_images(self):
        img_dir = os.path.expanduser("~/test_project/test_images")
        imgs = glob.glob(os.path.join(img_dir, "*.png"))
        imgs = [f for f in imgs if not os.path.basename(f).startswith("task2_result_")]
        return sorted(imgs)
    def publish_detection(self):
        if not self.images:
            self.get_logger().warn("没有找到图片，停止发布")
            return
        img_path = self.images[self.idx]
        img = cv2.imread(img_path)
        msg = TargetPosition()
        if img is None:
            self.get_logger().error(f"无法读取图片: {img_path}")
            msg.detected = False
            msg.u = 0.0
            msg.v = 0.0
            msg.area = 0.0
        else:
            h, w, _ = img.shape
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            lower_bound = [0, 0, 0]
            upper_bound = [180, 255, 60] 
            min_area = 500
            mask = cv2.inRange(hsv, np.array(lower_bound), np.array(upper_bound))
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            valid_contours = []
            for cnt in contours:
                area = cv2.contourArea(cnt)
                x, y, cw, ch = cv2.boundingRect(cnt)
                if area < min_area:
                    continue
                if 0.7 > cw/ch or cw/ch > 1.3: 
                    continue
                if area / (cw * ch) < 0.5: 
                    continue
                valid_contours.append((area, cnt))
            result_img = img.copy()
            if not valid_contours:
                msg.detected = False
                msg.u = 0.0
                msg.v = 0.0
                msg.area = 0.0
                self.get_logger().info(f"第 {self.count} 条 | 图片: {os.path.basename(img_path)} | 结果: 未检测到目标")
                cv2.putText(result_img, "No Target Detected", (50, 100),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
            else:        
                valid_contours.sort(key=lambda x: x[0], reverse=True)
                max_area, best_contour = valid_contours[0]
                M = cv2.moments(best_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    msg.detected = True
                    msg.u = float(cx)
                    msg.v = float(cy)
                    msg.area = float(max_area)
                    cv2.drawContours(result_img, [best_contour], -1, (0, 255, 0), 3)
                    cv2.circle(result_img, (cx, cy), 8, (0, 0, 255), -1)
                    text = f"u={cx}, v={cy}"
                    cv2.putText(result_img, text, (cx + 20, cy - 20), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                    self.get_logger().info(f"第 {self.count} 条 | 图片: {os.path.basename(img_path)} | 结果: 成功, u={cx}, v={cy}")
                else:
                    msg.detected = False
                    msg.u = 0.0
                    msg.v = 0.0
                    msg.area = 0.0
            save_dir = os.path.dirname(img_path)
            result_path = os.path.join(save_dir, f"task2_result_{os.path.basename(img_path)}")
            cv2.imwrite(result_path, result_img)
        self.publisher_.publish(msg)
        self.count += 1
        self.idx = (self.idx + 1) % len(self.images)
def main(args=None):
    rclpy.init(args=args)
    node = TargetPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
