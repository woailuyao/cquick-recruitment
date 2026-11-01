"""接收 /target_position，显示四个字段；可保存真实接收记录。

持续接收：ros2 run color_target_vision target_subscriber
接收 10 条后退出并保存：在上一命令后加
    --ros-args -p max_messages:=10 -p output_dir:=outputs/ros2
默认不保存文件；指定目录时每次新建子目录，不覆盖之前的实验。
"""

from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import ParameterDescriptor
from color_target_interfaces.msg import TargetPosition


class TargetSubscriber(Node):
    """只把真正收到的消息计入验收，未检出消息也保留 detected=false。"""

    def __init__(self):
        super().__init__('target_subscriber')
        self.received_count = 0
        self.done = False
        self._log = None
        self._run_dir = None
        self._closed = False
        self.started_at = datetime.now(timezone.utc).isoformat()
        try:
            self._initialize()
        except Exception:
            self.finish('startup_error')
            self.destroy_node()
            raise

    def _initialize(self):
        self.declare_parameter('max_messages', 0, ParameterDescriptor(
            description='接收条数上限，0 为不限；启动后不可修改', read_only=True))
        self.declare_parameter('output_dir', '', ParameterDescriptor(
            description='记录目录，空字符串不写文件；启动后不可修改', read_only=True))
        self.max_messages = self.get_parameter('max_messages').value
        if type(self.max_messages) is not int or self.max_messages < 0:
            raise ValueError('max_messages 必须是非负整数；验收时可以设为 10。')
        output_dir = self.get_parameter('output_dir').value
        if output_dir:
            parent = Path(output_dir).expanduser()
            stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
            self._run_dir = parent / f'subscriber_{stamp}'
            self._run_dir.mkdir(parents=True, exist_ok=False)
            self._log = (self._run_dir / 'received.jsonl').open('w', encoding='utf-8')
            self.get_logger().info(f'本次记录子目录：{self._run_dir.name}')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.subscription = self.create_subscription(
            TargetPosition, '/target_position', self._receive, qos)
        self.get_logger().info('等待 /target_position；收到第 10 条时提示达到条数要求。')

    def _receive(self, message):
        if self.done:
            return
        # 异常数值不能被静默计入验收；面积为像素平方，不是实际物理面积。
        numbers = (message.u, message.v, message.area)
        if not all(math.isfinite(value) and value >= 0 for value in numbers):
            raise ValueError('收到非有限或负数坐标/面积，请检查发布端。')
        if not message.detected and any(value != 0.0 for value in numbers):
            raise ValueError('detected=false 时 u/v/area 应全部为 0。')
        if message.detected and message.area <= 0:
            raise ValueError('detected=true 时 area 应大于 0。')
        self.received_count += 1
        record = {
            'sequence': self.received_count,
            'received_at_utc': datetime.now(timezone.utc).isoformat(),
            'detected': bool(message.detected),
            'u': float(message.u), 'v': float(message.v), 'area': float(message.area),
        }
        if self._log is not None:
            self._log.write(json.dumps(record, ensure_ascii=False) + '\n')
            self._log.flush()
        self.get_logger().info(
            f'接收 #{self.received_count} detected={record["detected"]} '
            f'u={record["u"]:.3f} v={record["v"]:.3f} area={record["area"]:.3f}'
        )
        if self.received_count == 10:
            self.get_logger().info('已实际接收 10 条有效格式消息，达到本项条数要求。')
        if self.max_messages and self.received_count >= self.max_messages:
            self.done = True

    def finish(self, reason):
        """退出时关闭 JSONL 并保存摘要；摘要只记录文件相对名，不暴露本机路径。"""
        if self._closed:
            return
        self._closed = True
        if self._log is not None:
            self._log.close()
        if self._run_dir is not None:
            summary = {
                'node': 'target_subscriber', 'topic': '/target_position',
                'started_at_utc': self.started_at,
                'finished_at_utc': datetime.now(timezone.utc).isoformat(),
                'received_count': self.received_count,
                'at_least_10_received': self.received_count >= 10,
                'end_reason': reason, 'records_file': 'received.jsonl',
                'scope': '本节点实际接收记录；条数达标不等于视觉识别准确率。',
            }
            (self._run_dir / 'summary.json').write_text(
                json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')


def main(args=None):
    rclpy.init(args=args)
    node = None
    reason = 'shutdown'
    exit_code = 0
    try:
        node = TargetSubscriber()
        # spin_once 便于收到指定条数后退出；未收到消息时仍可用 Ctrl+C 结束。
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
        if node.done:
            reason = 'max_messages'
    except (KeyboardInterrupt, ExternalShutdownException):
        reason = 'interrupted'
    except Exception as exc:
        reason = 'error'
        exit_code = 1
        print(f'订阅节点停止：{exc}', file=sys.stderr)
    finally:
        try:
            if node is not None:
                node.finish(reason)
        finally:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    if exit_code:
        raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
