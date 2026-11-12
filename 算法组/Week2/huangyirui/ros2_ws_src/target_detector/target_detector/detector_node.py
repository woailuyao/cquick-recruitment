import glob
import os

import cv2
import numpy as np
import rclpy

#import
from rclpy.executors import ExternalShutdownException

from rclpy.node import Node

from target_position.msg import TargetPosition

#创建类,阈值参数化
class DetectorNode(Node):

    def __init__(self):
        super().__init__("target_detector")

        self.declare_parameter("use_camera", False) #True开摄像头，False轮播图片
        self.declare_parameter(
            "image_dir",
            "/mnt/d/DeepSeek/Alcode/W2_黄一芮_视觉算法/data/ball",
        )#创建图片目录

        #土拍筛选规定
        self.declare_parameter("lower", [36, 85, 60])#HSV三通道下限
        self.declare_parameter("upper", [60, 255, 255])#HSV三通道上限
        self.declare_parameter("min_area", 300)#面积阈值
        self.declare_parameter("frame_period", 0.1)#每帧间隔

        #利用value把摄像头获取的parameter对象提取出python原生的类型
        self.use_camera = self.get_parameter("use_camera").value
        self.image_dir = self.get_parameter("image_dir").value
        #arrry转类型为uint8
        self.lower = np.array(self.get_parameter("lower").value, np.uint8)
        self.upper = np.array(self.get_parameter("upper").value, np.uint8)
        # 统一转类型，防止浮点暴毙
        self.min_area = int(self.get_parameter("min_area").value)
        period = float(self.get_parameter("frame_period").value)

        self.pub = self.create_publisher(TargetPosition, "/target_position", 10)#声明发布信息，格式频道频率
        self.timer = self.create_timer(period, self.tick)#自动轮询检测、发布内容

        self.images = []#创建图片列表
        if not self.use_camera:
            self.images = sorted(
                glob.glob(os.path.join(self.image_dir, "*.png"))
                + glob.glob(os.path.join(self.image_dir, "*.jpg"))
            )#根据参数填充，利用glob找文件，让jpg和png都能适配
            self.get_logger().info(
                f"共 {len(self.images)} 张：{self.image_dir}")#打印读取内容，利用logger加上时间戳

        self.idx = 0#播放指针，记录播放位置
        self.cap = cv2.VideoCapture(0) if self.use_camera else None#用None标识读取摄像头失败
        self.kernel = np.ones((5, 5), np.uint8)#降噪

    #取帧
    def next_frame(self):
        if self.use_camera:#摄像头模式
            check,frame = self.cap.read()#check返回bool值，判断读到与否；frame是图像
            return frame if check else None #若读到返回值
        if not self.images:                # 目录里一张图都没找到
            return None

        path = self.images[self.idx % len(self.images)]#模运算，防止无尽循环爆RE
        self.idx += 1#idx++
        return cv2.imread(path)

    #detect检测（同task1）
    def detect(self, img):

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower, self.upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best, best_area = None, 0.0
        for c in contours:
            area = cv2.contourArea(c)
            if area >= self.min_area and area > best_area:
                best, best_area = c, area

        
        if best is None:
            return False, 0.0, 0.0, 0.0#若无返回值，则要返回完整的四元组，防止报错

        m = cv2.moments(best)

        if m["m00"] == 0:
            return False, 0.0, 0.0, 0.0#若无返回值，则要返回完整的四元组，防止报错

        #转换成float，与消息字段float32统一
        return (True,
                float(m["m10"] / m["m00"]),
                float(m["m01"] / m["m00"]),
                float(best_area))

    #定时回调，一边算一边发送出去
    def tick(self):

        frame = self.next_frame()

        if frame is None:#读不到图/摄像头爆了
            msg = TargetPosition()
            msg.detected = False
            self.pub.publish(msg)#发布信息（显示结果）
            return

        detected, u, v, area = self.detect(frame)

        #每一条信息都对应一个对象，一一发布
        msg = TargetPosition()
        msg.detected = bool(detected)   
        msg.u = float(u)                
        msg.v = float(v)
        msg.area = float(area)

        self.pub.publish(msg)#发布信息

        #日志
        self.get_logger().info(
            f"publish detected={msg.detected} "
            f"u={msg.u:.0f} v={msg.v:.0f} area={msg.area:.0f}")#取整

#主函数main
def main():
    rclpy.init()#rclpy.init()初始化 ROS 2 客户端库

    node = DetectorNode()#创建节点对象

    try:#检查
        rclpy.spin(node)#永循环
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    finally:#最后退出清理
        try:
            node.destroy_node()#释放节点占用
            rclpy.shutdown() #关闭rclpy
        except Exception:
            pass

if __name__ == "__main__":
    main()
