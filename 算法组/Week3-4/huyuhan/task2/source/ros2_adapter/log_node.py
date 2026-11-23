"""ROS2 adapter for the current Windows v2 JSONL records."""

from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

root = Path(__file__).resolve().parents[2]
sys.path.insert(
    0, str(root / "task2_lab_night_v1_20261116/src")
)

from ros_payload import ball_message, pose_message


def utc_timestamp(record):
    value = record.get("time_utc") or record.get("timestamp")
    if not value:
        raise ValueError("Missing timestamp")

    dt = datetime.fromisoformat(str(value).replace("Z", "+00:00"))

    if dt.tzinfo is None:
        raise ValueError("Timestamp must include timezone")

    return dt.astimezone(timezone.utc).isoformat()


def frame_sequence(record):
    producer = record.get("producer") or {}
    value = record.get("sequence")

    if value is None:
        value = producer.get("sequence", record.get("frame", 0))

    return int(value)


def source_name(record):
    producer = record.get("producer") or {}
    return str(
        record.get("input_kind")
        or producer.get("source")
        or "unknown"
    )


class LogNode(Node):

    def __init__(self, kind):
        super().__init__("cquick_" + kind)

        self.declare_parameter("log_path", "")
        self.declare_parameter("follow", False)
        self.declare_parameter("max_batch_lines", 80)
        self.declare_parameter("stale_after", 0.55)

        self.kind = kind
        log_path = str(self.get_parameter("log_path").value)
        if not log_path:
            raise ValueError("log_path is required")
        self.path = Path(log_path)
        self.follow = bool(self.get_parameter("follow").value)
        self.max_batch = int(
            self.get_parameter("max_batch_lines").value
        )
        self.stale_after = float(
            self.get_parameter("stale_after").value
        )

        if not str(self.path):
            raise ValueError("log_path is required")

        if self.max_batch < 1 or self.stale_after <= 0:
            raise ValueError("Invalid node parameters")

        topic = (
            "/soccer/ball"
            if kind == "ball"
            else "/localization/field_pose"
        )

        self.pub = self.create_publisher(String, topic, 10)

        self.file = None
        self.pending = ""
        self.last_input = time.monotonic()
        self.sent_stale = False
        self.seq = 0
        self.last_producer = {}

        self.timer = self.create_timer(0.02, self.tick)

        self.get_logger().info(
            f"Started {kind}: path={self.path}, "
            f"follow={self.follow}, topic={topic}"
        )

    def base_packet(self, record):
        producer = record.get("producer") or {}

        packet = {
            "timestamp": utc_timestamp(record),
            "frame": frame_sequence(record),
            "source": source_name(record),
            "source_transport": "single_producer_jsonl",
        }

        if producer.get("width") is not None:
            packet["width"] = producer["width"]

        if producer.get("height") is not None:
            packet["height"] = producer["height"]

        return packet

    def make_pose(self, record):
        packet = self.base_packet(record)

        packet.update({
            "valid": record.get("valid") is True,
            "status": record.get("status", "invalid"),
            "marker_count": int(record.get("marker_count") or 0),
            "residual": record.get("residual"),

            "x_camera_map": record.get("x_camera_map"),
            "y_camera_map": record.get("y_camera_map"),
            "yaw_camera_map": record.get("yaw_camera_map"),

            "x_carrier_map": record.get("x_carrier_map"),
            "y_carrier_map": record.get("y_carrier_map"),
            "yaw_carrier_map": record.get("yaw_carrier_map"),
        })

        payload = pose_message(packet)

        payload["input_kind"] = record.get("input_kind")
        payload["source_session"] = (
            record.get("source_session")
            or (record.get("producer") or {}).get("session")
        )
        payload["transport_status"] = record.get("transport_status")

        if record.get("valid") is True and not payload["valid"]:
            payload["status"] = "missing_or_invalid_pose_fields"

        return payload

    def make_ball(self, record):
        packet = self.base_packet(record)

        detections = record.get("detections")
        channel_exists = (
            isinstance(detections, dict)
            and isinstance(detections.get("ball"), list)
        )

        balls = detections["ball"] if channel_exists else []

        try:
            payload = ball_message(packet, balls)
            valid = channel_exists and record.get("status") != "log_stale"

            status = (
                "ok" if balls
                else "no_detection" if valid
                else "ball_channel_missing"
            )
            if record.get("status") == "log_stale":
                valid = False
                status = "log_stale"

        except (KeyError, TypeError, ValueError) as exc:
            payload = ball_message(packet, [])
            valid = False
            status = "malformed_ball_detection"
            self.get_logger().error(str(exc))

        payload.update({
            "valid": valid,
            "status": status,
            "detection_count": len(balls) if valid else 0,
            "input_kind": record.get("input_kind"),
            "source_session": (
                record.get("source_session")
                or (record.get("producer") or {}).get("session")
            ),
            "transport_status": record.get("transport_status"),
        })

        return payload

    def publish_record(self, record):
        try:
            if self.kind == "pose":
                payload = self.make_pose(record)
            else:
                payload = self.make_ball(record)

            msg = String()
            msg.data = json.dumps(
                payload,
                ensure_ascii=False,
                allow_nan=False
            )
            self.pub.publish(msg)

        except (KeyError, TypeError, ValueError) as exc:
            self.get_logger().error(
                f"Record conversion failed: {exc}"
            )

    def publish_stale(self):
        now = datetime.now(timezone.utc).isoformat()

        record = {
            "time_utc": now,
            "sequence": self.seq + 1,
            "input_kind": "ros2_bridge_timeout",
            "valid": False,
            "status": "log_stale",
            "marker_count": 0,
            "detections": {"ball": []},
            "producer": self.last_producer,
        }

        self.seq += 1
        self.publish_record(record)
        self.sent_stale = True

        self.get_logger().warning(
            "Input timeout: published invalid state"
        )

    def open_file(self):
        if self.file is not None or not self.path.is_file():
            return

        self.file = self.path.open(encoding="utf-8")

        if self.follow:
            self.file.seek(0, 2)

        self.get_logger().info(
            f"Opened JSONL: {self.path}"
        )

    def read_records(self):
        records = []

        limit = self.max_batch if self.follow else 1

        for _ in range(limit):
            line = self.file.readline()

            if not line:
                break

            self.pending += line

            if not self.pending.endswith("\n"):
                break

            raw = self.pending
            self.pending = ""

            try:
                records.append(json.loads(raw))
            except json.JSONDecodeError as exc:
                self.get_logger().error(
                    f"Invalid JSONL record: {exc}"
                )

        return records

    def tick(self):
        self.open_file()

        if self.file is None:
            return

        # Do not consume records before a subscriber is available.
        if self.pub.get_subscription_count() == 0:
            return

        records = self.read_records()

        for record in records:
            self.last_input = time.monotonic()
            self.sent_stale = False
            self.last_producer = record.get(
                "producer", self.last_producer
            )
            self.seq = frame_sequence(record)

            self.publish_record(record)

        if (
            self.follow
            and not records
            and not self.sent_stale
            and time.monotonic() - self.last_input > self.stale_after
        ):
            self.publish_stale()

    def destroy_node(self):
        if self.file is not None:
            self.file.close()

        super().destroy_node()


if __name__ == "__main__":
    kind = sys.argv.pop(1)
    assert kind in {"ball", "pose"}

    rclpy.init()
    node = LogNode(kind)

    try:
        rclpy.spin(node)

    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()
