"""Action 客户端：手动发 goal，打印 feedback 与 result。

两种用法：
  交互（默认）  ros2 run cquick_servo move_client
                输入 `1 1700 800` = id / pwm / duration_ms；`q` 退出
  单发          ros2 run cquick_servo move_client -p interactive:=false -p servo_id:=1 -p pwm:=1700
"""
import queue
import sys
import threading

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from cquick_servo_interfaces.action import MoveServo


class MoveClient(Node):
    def __init__(self):
        super().__init__("move_client")
        self.declare_parameter("interactive", True)
        self.declare_parameter("servo_id", 1)
        self.declare_parameter("pwm", 1500)
        self.declare_parameter("duration_ms", 800)

        self.client = ActionClient(
            self, MoveServo, "move_servo",
            callback_group=ReentrantCallbackGroup())
        self._goal = None
        self._done = False

    def _on_feedback(self, msg):
        fb = msg.feedback
        self.get_logger().info(
            f"  [feedback] pwm={fb.current_pwm} progress={fb.progress:.2f}")

    def _on_result(self, future):
        self._done = True
        res = future.result()
        if res is None:
            self.get_logger().error("  [result] 拿不到 result")
            return
        r = res.result
        self.get_logger().info(
            f"  [result] success={r.success} final_pwm={r.final_pwm} "
            f"code={r.code} message={r.message}")

    def send(self, sid, pwm, dur):
        goal = MoveServo.Goal()
        goal.id = int(sid)
        goal.pwm = int(pwm)
        goal.duration_ms = int(dur)
        self._done = False
        self.get_logger().info(f"--> goal id={sid} pwm={pwm} duration_ms={dur}")
        future = self.client.send_goal_async(goal, feedback_callback=self._on_feedback)
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.02)
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error("<-- goal 被拒绝（id 不在 servo_ids / pwm 超限 / 安全锁）")
            self._done = True
            return
        handle.get_result_async().add_done_callback(self._on_result)


def _stdin_reader(q):
    while True:
        line = sys.stdin.readline()
        if not line:
            q.put(None)
            return
        q.put(line.strip())


def main():
    rclpy.init()
    node = MoveClient()

    if not node.client.wait_for_server(timeout_sec=5.0):
        node.get_logger().error("等不到 move_servo：先起 servo_driver")
        node.destroy_node()
        rclpy.shutdown()
        return

    if node.get_parameter("interactive").value:
        q = queue.Queue()
        threading.Thread(target=_stdin_reader, args=(q,), daemon=True).start()
        print("输入格式： id pwm duration_ms   例如 1 1700 800    q 退出")
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            try:
                line = q.get_nowait()
            except queue.Empty:
                continue
            if line is None or line.lower() in ("q", "quit", "exit"):
                break
            parts = line.replace(",", " ").split()
            if not parts:
                continue
            try:
                sid = int(parts[0])
                pwm = int(parts[1])
                dur = int(parts[2]) if len(parts) > 2 else 800
            except (IndexError, ValueError):
                print("格式不对： id pwm duration_ms")
                continue
            node.send(sid, pwm, dur)
    else:
        node.send(node.get_parameter("servo_id").value,
                  node.get_parameter("pwm").value,
                  node.get_parameter("duration_ms").value)
        while rclpy.ok() and not node._done:
            rclpy.spin_once(node, timeout_sec=0.05)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
