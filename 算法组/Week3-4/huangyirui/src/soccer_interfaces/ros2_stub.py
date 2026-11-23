from __future__ import annotations
import json
from pathlib import Path


class FramePublisher:
    def __init__(self,jsonl_path=None,prefer_ros2=True):
        self.file = None
        self.node = None
        self.context = None
        self.kind = "jsonl"
        if jsonl_path:
            path = Path(jsonl_path)
            path.parent.mkdir(parents=True,exist_ok=True)
            self.file = path.open("w",encoding="utf-8")
        if prefer_ros2:
            try:
                import rclpy
                from rclpy.context import Context
                from std_msgs.msg import String
                self.context = Context()
                rclpy.init(args=[],context=self.context)
                self.node = rclpy.create_node("soccer_vision_node",context=self.context)
                self.message_type = String
                self.pose_topic = self.node.create_publisher(String,"/localization/field_pose",10)
                self.ball_topic = self.node.create_publisher(String,"/soccer/ball",10)
                self.kind = "ros2"
            except Exception as exc:
                self._close_ros()
                print(f"[ros2] {exc}，使用本地输出")

    def publish(self,pose,balls=()):
        detections = []
        for det in balls:
            u,v = det.center
            detections.append(dict(name=det.name,confidence=float(det.conf),
                                   bbox=list(map(float,det.xyxy)),center_u=float(u),center_v=float(v)))
        record = pose.to_dict()
        if self.file:
            self.file.write(json.dumps(dict(record,balls=detections),allow_nan=False)+"\n")
        if self.node:
            ball_msg = dict(frame_id=pose.frame_id,stamp=pose.stamp,balls=detections)
            for topic,data in ((self.pose_topic,record),(self.ball_topic,ball_msg)):
                msg = self.message_type()
                msg.data = json.dumps(data,allow_nan=False)
                topic.publish(msg)

    def _close_ros(self):
        if self.node is not None:
            self.node.destroy_node()
            self.node = None
        if self.context is not None and self.context.ok():
            self.context.shutdown()

    def close(self):
        try:
            self._close_ros()
        finally:
            if self.file:
                self.file.close()
                self.file = None


def make_publisher(jsonl_path=None,prefer_ros2=True):
    return FramePublisher(jsonl_path,prefer_ros2)
