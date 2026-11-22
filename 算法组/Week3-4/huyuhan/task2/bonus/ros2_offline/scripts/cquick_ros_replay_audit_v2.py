import json
import time
from collections import Counter

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class Audit(Node):
    def __init__(self):
        super().__init__("cquick_replay_audit_v2")

        self.stats = {
            "ball": Counter(),
            "pose": Counter()
        }

        self.last = None
        self.errors = 0

        qos = QoSProfile(
            depth=1000,
            reliability=ReliabilityPolicy.RELIABLE
        )

        self.create_subscription(
            String, "/soccer/ball",
            lambda m: self.callback("ball", m), qos
        )

        self.create_subscription(
            String, "/localization/field_pose",
            lambda m: self.callback("pose", m), qos
        )

    def callback(self, kind, msg):
        self.last = time.monotonic()

        try:
            d = json.loads(msg.data)
        except Exception:
            self.errors += 1
            return

        s = self.stats[kind]
        s["total"] += 1
        s[str(d.get("status"))] += 1

        if d.get("valid") is True:
            s["valid"] += 1
        else:
            s["invalid"] += 1

        if kind == "ball":
            detections = d.get("detections") or []
            s["instances"] += len(detections)
            if not detections:
                s["empty"] += 1

        else:
            s["markers_" + str(d.get("marker_count", 0))] += 1


def main():
    rclpy.init()
    node = Audit()

    start = time.monotonic()
    last_report = start
    reason = "unknown"

    print("AUDIT READY：现在可以启动 ROS2 回放", flush=True)

    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.01)
            now = time.monotonic()

            if now - last_report >= 5:
                print(
                    "进度:",
                    {k: v["total"] for k, v in node.stats.items()},
                    flush=True
                )
                last_report = now

            if node.last is None and now - start > 60:
                reason = "no_messages"
                break

            if node.last is not None and now - node.last > 8:
                reason = "replay_finished_or_idle"
                break

            if now - start > 360:
                reason = "timeout"
                break

    finally:
        print("\n结束原因:", reason)
        print("========== 最终审计 ==========")

        for name, stats in node.stats.items():
            print(name, dict(stats))

        print("JSON错误:", node.errors)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
