import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import cv2
import numpy as np

class ColorDetector(Node):
    def __init__(self):
        super().__init__("color_detector")
        self.pub = self.create_publisher(Float32MultiArray, "/target_center", 10)
        self.cap = cv2.VideoCapture(0)

    def run(self):
        while rclpy.ok():
            ret, frame = self.cap.read()
            if not ret:
                continue
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            lower = np.array([0, 120, 120])
            upper = np.array([10, 255, 255])
            mask = cv2.inRange(hsv, lower, upper)

            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            msg = Float32MultiArray()
            if contours:
                c = max(contours, key=cv2.contourArea)
                M = cv2.moments(c)
                if M["m00"] != 0:
                    cx = M["m10"] / M["m00"]
                    cy = M["m01"] / M["m00"]
                    msg.data = [cx, cy]
                    self.pub.publish(msg)
                    cv2.circle(frame, (int(cx), int(cy)), 6, (0,255,0), -1)
            cv2.imshow("frame", frame)
            cv2.imshow("mask", mask)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = ColorDetector()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
