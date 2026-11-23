"""Use actual DDS topics emitted by the offline launch."""
import json
import math
import time
import rclpy
from std_msgs.msg import String

rclpy.init()
node=rclpy.create_node("cquick_verify_20261119")
seen={"ball":[],"pose":[]}
subscriptions=[node.create_subscription(String,topic,lambda m,key=key:seen[key].append(json.loads(m.data)),10)
               for key,topic in (("ball","/soccer/ball"),("pose","/localization/field_pose"))]
until=time.monotonic()+20
try:
    while time.monotonic()<until:
        rclpy.spin_once(node,timeout_sec=.2)
        if any(x["valid"] for x in seen["ball"]) and any(x["valid"] for x in seen["pose"]) and any(
           not x["valid"] for x in seen["pose"]):
            break
    b=next(x for x in seen["ball"] if x["valid"] and x["detections"])
    p=next(x for x in seen["pose"] if x["valid"])
    n=next(x for x in seen["pose"] if not x["valid"])
    assert b["source"]=="offline_log_replay" and b["header"]["frame_id"]=="camera_optical"
    assert len(b["detections"][0]["center_uv_px"])==2
    assert p["header"]["frame_id"]=="field_map" and p["position_unit"]=="m" and p["residual_unit"]=="px"
    assert p["carrier_origin_in_map"] is None
    expected=p["camera_optical_center_in_map"]
    assert math.isclose(expected["yaw_rad"],math.radians(expected["yaw_deg"]))
    assert n["camera_optical_center_in_map"] is None and n["status"]=="insufficient_points"
    print(json.dumps(dict(result="PASS offline ROS2 DDS",ball=b,pose=p,lost=n),indent=2))
finally:
    node.destroy_node()
    rclpy.shutdown()
