#!/usr/bin/env python3
"""逐个舵机小幅摆动：用来认「哪个 ID 是哪个关节」+ 验证方向对不对。

    ros2 run cquick_servo joint_probe                  # 全部 id 依次 +30 / -30 / 回中位
    ros2 run cquick_servo joint_probe --ros-args -p step:=20 -p dwell_ms:=800

为什么要这个：10 个舵机上电后，光看线是认不出 1 号是左髋还是右髋的。
这个脚本一次只动一个，日志里写清「现在动的是几号」，你眼睛对上关节就能填表。

安全：步长默认只有 30 个 PWM，且每步都等 result 回来才动下一步；
听到嗡嗡声就 Ctrl+C，脚本会把在跑的 goal 撤掉。
"""
from datetime import datetime

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from cquick_servo_interfaces.action import MoveServo


def ts():
    """挂钟时间戳，便于和录屏对齐。"""
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]


class JointProbe(Node):
    def __init__(self):
        super().__init__("joint_probe")
        self.declare_parameter("servo_ids", [1, 2, 3, 4, 5, 6, 7, 8, 9, 10])
        self.declare_parameter("center", 1500)
        self.declare_parameter("step", 30)
        self.declare_parameter("dwell_ms", 600)

        self.cb_group = ReentrantCallbackGroup()
        self.client = ActionClient(
            self, MoveServo, "move_servo", callback_group=self.cb_group)

    def move(self, sid, pwm, dur):
        """发一个 goal 并等 result，返回 (success, final_pwm, message)。"""
        goal = MoveServo.Goal()
        goal.id = sid
        goal.pwm = pwm
        goal.duration_ms = dur
        send_fu = self.client.send_goal_async(goal)
        while rclpy.ok() and not send_fu.done():
            rclpy.spin_once(self, timeout_sec=0.02)
        handle = send_fu.result()
        if handle is None or not handle.accepted:
            return False, 0, "goal 被拒（id 不在 servo_ids / pwm 超限）"

        res_fu = handle.get_result_async()
        while rclpy.ok() and not res_fu.done():
            rclpy.spin_once(self, timeout_sec=0.02)
        if not res_fu.done():
            return False, 0, "等不到 result"
        res = res_fu.result().result
        return res.success, res.final_pwm, res.message

    def run(self):
        ids = list(self.get_parameter("servo_ids").value)
        center = int(self.get_parameter("center").value)
        step = int(self.get_parameter("step").value)
        dwell = int(self.get_parameter("dwell_ms").value)
        up = min(center + step, 2400)
        down = max(center - step, 600)

        self.get_logger().info(
            f"[{ts()}] 逐个摆动 {len(ids)} 个舵机：中位 {center}，"
            f"上行 {up}，下行 {down}，每步 {dwell}ms")
        self.get_logger().info(
            f"[{ts()}] 看到哪个关节在动，就记下这个 ID；方向反了就是接线/装配问题")

        table = []
        for sid in ids:
            self.get_logger().warn(f"[{ts()}] >>> 现在动 id={sid}，看结构 <<<")
            ok1, p1, m1 = self.move(sid, up, dwell)
            self.get_logger().info(
                f"[{ts()}] id={sid} 上行 -> {up}：success={ok1} "
                f"final={p1} msg={m1}")
            ok2, p2, m2 = self.move(sid, down, dwell)
            self.get_logger().info(
                f"[{ts()}] id={sid} 下行 -> {down}：success={ok2} "
                f"final={p2} msg={m2}")
            ok3, p3, m3 = self.move(sid, center, dwell)
            self.get_logger().info(
                f"[{ts()}] id={sid} 回中位 -> {center}：success={ok3} "
                f"final={p3} msg={m3}")
            table.append((sid, p1, p2, p3, ok1 and ok2 and ok3))

        self.get_logger().info(f"[{ts()}] ===== 汇总（填进 README 的关节表）=====")
        dead = []
        for sid, p1, p2, p3, ok in table:
            moved = max(p1, p2, p3) - min(p1, p2, p3)
            flag = "OK" if moved >= 15 else "没响应（位置几乎没变）"
            if moved < 15:
                dead.append(sid)
            self.get_logger().info(
                f"  id={sid:>2}  上行到位={p1:>4}  下行到位={p2:>4}  回中位={p3:>4}  "
                f"位移={moved:>3}  {flag}")

        if dead:
            self.get_logger().warn(
                f"[{ts()}] 下面这些 ID 在 ±{step} 里没动：{dead}")
            self.get_logger().warn(
                f"[{ts()}] 先单独大摆一次确认死活，例如 id={dead[0]}：")
            self.get_logger().warn(
                f"[{ts()}]   ros2 action send_goal /move_servo "
                f"cquick_servo_interfaces/action/MoveServo "
                f"\"{{id: {dead[0]}, pwm: 1300, duration_ms: 2000}}\"")
            self.get_logger().warn(
                f"[{ts()}] 大摆也不动就拔电池，用手转这个关节：手也转不动=结构卡死，"
                f"手能转=供电不足或该颗坏了")


def main():
    rclpy.init()
    node = JointProbe()
    if not node.client.wait_for_server(timeout_sec=5.0):
        node.get_logger().error("等不到 move_servo，先把 servo_driver 起来")
    else:
        try:
            node.run()
        except KeyboardInterrupt:
            node.get_logger().warn("Ctrl+C，停止摆动")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
