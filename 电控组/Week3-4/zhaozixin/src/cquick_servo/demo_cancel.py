#!/usr/bin/env python3
"""取消打断演示客户端（对应截图 02_取消生效.png / 录屏第 2 段）。

流程：发 goal → 打印 feedback → 0.6 s 后 cancel_goal_async() → 打印 cancel 响应
     → 打印 result（success 必须 False，message 必须是 canceled）。

正确用法：
  终端 A  ros2 launch cquick_servo servo.launch.py
  终端 B  cd ~/servo_ws && source install/setup.bash && python3 demo_cancel.py

先看返回码：
  0 接受取消（正常）  1 已在取消中  2 目标不存在  3 目标已结束（说明取消发晚了/有重复节点）
"""
import time

import rclpy
from rclpy.action import ActionClient

from cquick_servo_interfaces.action import MoveServo

SID = 1
TARGET_PWM = 1900      # 离中位远一点，动作时间足够长，便于中途打断
DURATION_MS = 5000     # 给 5 s，0.6 s 时还在运动途中
CANCEL_AFTER = 0.6     # 发出 goal 后多久取消

RETURN_CODE_HINT = {
    0: "已接受取消",
    1: "目标已在取消中",
    2: "目标不存在（goal id 对不上）",
    3: "目标已结束：取消发晚了，或存在两个 servo_driver",
}


def on_feedback(fb):
    print(f"id={SID} pwm={fb.feedback.current_pwm} "
          f"progress={fb.feedback.progress:.2f}")


def check_duplicate_servers(node):
    """两个 servo_driver 同时在线时，goal 和 cancel 可能落到不同节点上。"""
    names = [f"{ns.strip('/')}/{n}".strip('/')
             for n, ns in node.get_node_names_and_namespaces()]
    dup = [n for n in names if n.endswith("servo_driver")]
    if len(dup) > 1:
        print(f"⚠ 检测到 {len(dup)} 个 servo_driver：{dup}")
        print("  先 pkill -f servo_driver 清掉残留，再重启 launch")


def wait_result(node, goal_handle, timeout_sec=20.0):
    future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    return future.result().result


def main():
    rclpy.init()
    node = rclpy.create_node("demo_cancel_client")
    check_duplicate_servers(node)
    cli = ActionClient(node, MoveServo, "move_servo")

    print("等待 /move_servo 上线 ...")
    if not cli.wait_for_server(timeout_sec=10.0):
        print("❌ 找不到 /move_servo，先起 launch")
        node.destroy_node()
        rclpy.shutdown()
        return

    # 预置动作：先把舵机送回中位。否则上一轮已经停在目标位时，
    # 服务端会判「already in place」直接 succeed，goal 瞬间结束，取消必然返回 3。
    prep = MoveServo.Goal()
    prep.id = SID
    prep.pwm = 1500
    prep.duration_ms = 1200
    print("预置：先回中位 1500 ...")
    prep_handle = cli.send_goal_async(prep)
    rclpy.spin_until_future_complete(node, prep_handle, timeout_sec=5.0)
    ph = prep_handle.result()
    if ph.accepted:
        wait_result(node, ph)
        print("预置完成，舵机在中位附近")
    else:
        print("⚠ 预置 goal 被拒，继续（若随后取消返回 3，先查 pwm 范围/重复节点）")

    goal = MoveServo.Goal()
    goal.id = SID
    goal.pwm = TARGET_PWM
    goal.duration_ms = DURATION_MS

    print(f"发送 goal: id={SID} pwm={TARGET_PWM} duration_ms={DURATION_MS}")
    send_future = cli.send_goal_async(goal, feedback_callback=on_feedback)
    rclpy.spin_until_future_complete(node, send_future)
    goal_handle = send_future.result()

    if not goal_handle.accepted:
        print("❌ goal 被拒（检查 pwm 是否在 pwm_min~pwm_max 内）")
        node.destroy_node()
        rclpy.shutdown()
        return

    print("✅ goal 已接收，开始运动")

    t0 = time.time()
    while time.time() - t0 < CANCEL_AFTER:
        rclpy.spin_once(node, timeout_sec=0.05)

    print(f"⏱ {CANCEL_AFTER:.1f}s 到，发出取消请求")
    cancel_future = goal_handle.cancel_goal_async()
    rclpy.spin_until_future_complete(node, cancel_future, timeout_sec=3.0)
    cancel_res = cancel_future.result()
    print(f"取消响应: return_code={cancel_res.return_code} "
          f"goals_canceling={len(cancel_res.goals_canceling)}")
    print(f"          → {RETURN_CODE_HINT.get(cancel_res.return_code, '未知返回码')}")

    res_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, res_future, timeout_sec=20.0)
    result = res_future.result().result

    print("=" * 46)
    print(f"result.success   = {result.success}")
    print(f"result.message   = {result.message}")
    print(f"result.final_pwm = {result.final_pwm}")
    print("=" * 46)

    ok = (not result.success) and result.message == "canceled"
    if ok:
        print("✔ 合格：success=False 且 message=canceled，截图就取这一段")
    else:
        print("❌ 不合格：success 应为 False、message 应为 canceled")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
