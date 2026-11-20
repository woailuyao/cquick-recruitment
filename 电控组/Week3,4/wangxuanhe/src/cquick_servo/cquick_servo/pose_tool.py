#!/usr/bin/env python3
"""姿态微调工具：一颗一颗把整机摆成直立，再把实测值固化成一个 yaml 动作。

    ros2 run cquick_servo pose_tool show                  # 读回 10 颗当前位置（带关节名）
    ros2 run cquick_servo pose_tool set 2:1560 3:1440     # 绝对值，一次下发、并行到位
    ros2 run cquick_servo pose_tool set 2:+20 4:-20       # 相对当前位置微调
    ros2 run cquick_servo pose_tool save home              # 当前姿态存成 home.yaml
    ros2 run cquick_servo pose_tool goto home              # 回放已存姿态

关节表（来自整机实测）：左腿 1-5、右腿 6-10，自上而下。
  1 左胯外摆   2 左髋俯仰   3 左膝   4 左踝俯仰   5 左踝外摆
  6 右胯外摆   7 右髋俯仰   8 右膝   9 右踝俯仰  10 右踝外摆
"""
import os
import sys

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_srvs.srv import Trigger

from cquick_servo_interfaces.action import MoveServo

JOINT_NAMES = {
    1: "左·胯外摆", 2: "左·髋俯仰", 3: "左·膝", 4: "左·踝俯仰", 5: "左·踝外摆",
    6: "右·胯外摆", 7: "右·髋俯仰", 8: "右·膝", 9: "右·踝俯仰", 10: "右·踝外摆",
}


class PoseTool(Node):
    def __init__(self):
        super().__init__("pose_tool")
        self.declare_parameter("duration_ms", 1200)
        self.declare_parameter("actions_dir", "")
        cb = ReentrantCallbackGroup()
        self.client = ActionClient(self, MoveServo, "move_servo", callback_group=cb)
        self.read_client = self.create_client(
            Trigger, "/servo_driver/read_all", callback_group=cb)

    # ---------------- 基础能力 ----------------
    def actions_dir(self):
        d = self.get_parameter("actions_dir").value
        if d:
            return d
        return os.path.join(
            get_package_share_directory("cquick_servo"), "config", "actions")

    def read_all(self):
        """读回 {id: pwm}，读不到记 None。"""
        while not self.read_client.wait_for_service(timeout_sec=0.2):
            self.get_logger().warn("等 /servo_driver/read_all …（driver 起来了吗）")
        fu = self.read_client.call_async(Trigger.Request())
        while rclpy.ok() and not fu.done():
            rclpy.spin_once(self, timeout_sec=0.02)
        msg = fu.result().message if fu.result() else ""
        pose = {}
        for item in msg.split():
            sid, val = item.split(":")
            p = int(val)
            pose[int(sid)] = None if p < 0 else p
        return pose

    def move_many(self, targets, dur=None):
        """并行下发多个 goal，等全部 result 回来，返回 [(id, success, final, msg)]。"""
        dur = int(dur or self.get_parameter("duration_ms").value)
        self.client.wait_for_server(timeout_sec=5.0)
        handles, futures = [], []
        for sid, pwm in targets.items():
            goal = MoveServo.Goal()
            goal.id, goal.pwm, goal.duration_ms = int(sid), int(pwm), dur
            fu = self.client.send_goal_async(goal)
            while rclpy.ok() and not fu.done():
                rclpy.spin_once(self, timeout_sec=0.02)
            handle = fu.result()
            if handle is None or not handle.accepted:
                futures.append(None)
                continue
            handles.append(handle)
            futures.append(handle.get_result_async())
        for fu in futures:
            if fu is not None:
                while rclpy.ok() and not fu.done():
                    rclpy.spin_once(self, timeout_sec=0.02)
        out = []
        for sid, fu in zip(targets, futures):
            if fu is None:
                out.append((sid, False, 0, "goal 被拒"))
                continue
            res = fu.result().result
            out.append((sid, res.success, res.final_pwm, res.message))
        return out

    # ---------------- 子命令 ----------------
    def cmd_show(self, _args):
        pose = self.read_all()
        self.get_logger().info("id  关节       实测  目标差")
        for sid in sorted(pose):
            p = pose[sid]
            diff = "" if p is None or p == 1500 else f"{p - 1500:+d}"
            name = JOINT_NAMES.get(sid, "?")
            self.get_logger().info(
                f"{sid:>2}  {name:<10} {p if p is not None else '读不到':>6}  {diff}")

    def cmd_set(self, args):
        cur = self.read_all()
        targets = {}
        for token in args:
            sid_s, val_s = token.split(":")
            sid = int(sid_s)
            if val_s[0] in "+-":
                base = cur.get(sid)
                if base is None:
                    self.get_logger().error(f"id={sid} 读不到当前位置，不能用相对值")
                    continue
                targets[sid] = base + int(val_s)
            else:
                targets[sid] = int(val_s)
        if not targets:
            self.get_logger().error("没有可执行的 id:pwm")
            return
        self.get_logger().info(f"下发：{targets}")
        for sid, ok, final, msg in self.move_many(targets):
            self.get_logger().info(
                f"id={sid:>2} {JOINT_NAMES.get(sid, '?'):<10} "
                f"success={ok} final={final} msg={msg}")

    def cmd_save(self, args):
        if not args:
            self.get_logger().error("用法：save <名字>，例如 save home")
            return
        name = args[0]
        pose = self.read_all()
        bad = [sid for sid, p in pose.items() if p is None]
        if bad:
            self.get_logger().warn(f"下面这些 ID 读不到，先别存：{bad}")
        dur = int(self.get_parameter("duration_ms").value)
        doc = {
            "name": name,
            "description": "直立站立姿态（pose_tool save 实测固化）",
            "default_duration_ms": dur,
            "steps": [{
                "name": "起立到直立",
                "joints": [
                    {"id": sid, "pwm": p, "duration_ms": dur}
                    for sid, p in sorted(pose.items()) if p is not None
                ],
            }],
        }
        path = os.path.join(self.actions_dir(), f"{name}.yaml")
        with open(path, "w", encoding="utf-8") as f:
            yaml.safe_dump(doc, f, allow_unicode=True, sort_keys=False)
        self.get_logger().info(f"已写入 {path}")
        self.get_logger().warn(
            "这是 install 里的副本，整机验证没问题后记得覆盖回 src 同一路径")

    def cmd_goto(self, args):
        if not args:
            self.get_logger().error("用法：goto <名字>")
            return
        name = args[0]
        if not name.endswith(".yaml"):
            name += ".yaml"
        path = os.path.join(self.actions_dir(), name)
        with open(path, "r", encoding="utf-8") as f:
            doc = yaml.safe_load(f)
        targets = {j["id"]: j["pwm"] for j in doc["steps"][0]["joints"]}
        self.get_logger().info(f"{name}：下发 {len(targets)} 个关节")
        for sid, ok, final, msg in self.move_many(
                targets, doc.get("default_duration_ms")):
            self.get_logger().info(
                f"id={sid:>2} {JOINT_NAMES.get(sid, '?'):<10} "
                f"success={ok} final={final} msg={msg}")


def main():
    rclpy.init()
    node = PoseTool()
    argv = []
    for a in sys.argv[1:]:
        if a == "--ros-args":
            break
        argv.append(a)
    cmd, args = (argv[0], argv[1:]) if argv else ("show", [])
    table = {
        "show": node.cmd_show,
        "set": node.cmd_set,
        "save": node.cmd_save,
        "goto": node.cmd_goto,
    }
    if cmd not in table:
        node.get_logger().error(f"未知子命令 {cmd}，可用：show / set / save / goto")
    else:
        try:
            table[cmd](args)
        except KeyboardInterrupt:
            node.get_logger().warn("Ctrl+C")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
