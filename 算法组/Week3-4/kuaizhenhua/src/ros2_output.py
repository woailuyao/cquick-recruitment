"""Optional ROS2 Humble adapter, std_msgs/String containing documented JSON.

Install rclpy using ROS2 packages, not pip. Source Humble before --ros2.
Geometry status/units are preserved, including metric_valid=False.
"""
import json

class RosOutput:
    def __init__(self):
        import rclpy
        from std_msgs.msg import String
        self.rclpy=rclpy;self.String=String
        rclpy.init();self.node=rclpy.create_node('kuaizhenhua_soccer_vision')
        self.ball=self.node.create_publisher(String,'/soccer/ball',10)
        self.pose=self.node.create_publisher(String,'/localization/field_pose',10)

    def publish(self,record):
        base={k:record[k] for k in ('processed_at','input_mode','frame_index_0based','source_receipt_iso')}
        for publisher,payload in ((self.ball,{**base,'detected':record['ball_detected'],'detections':record['balls'],'unit':'pixels'}),
                                  (self.pose,{**base,**record['pose']})):
            msg=self.String();msg.data=json.dumps(payload,ensure_ascii=False,allow_nan=False);publisher.publish(msg)
        self.rclpy.spin_once(self.node,timeout_sec=0)

    def close(self):
        self.node.destroy_node();self.rclpy.shutdown()
