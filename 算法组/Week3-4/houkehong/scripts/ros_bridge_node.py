# -*- coding: utf-8 -*-
"""
ROS2 转发节点（在 WSL2 里跑）—— **监听模式**

为什么是监听而不是连接：
    WSL2 默认 NAT 网络下，Windows 主动连 WSL 里监听的端口是可以的
    （WSL 会自动做 localhost 端口转发），
    而 WSL 主动连 Windows 的端口会被 Windows 防火墙/NAT 挡住。
    所以这里反过来：WSL 监听，Windows 侧 run.py 连过来。

流程：
    Windows: run.py live --status-port 5056      （主动连接 127.0.0.1:5056）
    WSL2   : python3 scripts/ros_bridge_node.py --port 5056   （监听并转发到 ROS2）

用法（WSL2 内）：

    source /opt/ros/humble/setup.bash
    python3 /mnt/d/vision_ws/k1_soccer_vision/scripts/ros_bridge_node.py --port 5056

另开一个 WSL 终端看结果：

    ros2 topic list
    ros2 topic echo /localization/field_pose
    ros2 topic echo /localization/field_pose_2d
"""

from __future__ import annotations

import argparse
import json
import os
import socket
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.ros_bridge import (ROS_TOPIC_BALL, ROS_TOPIC_POSE,  # noqa: E402
                            ROS_TOPIC_POSE2D, RosBridge)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="0.0.0.0", help="监听地址")
    ap.add_argument("--port", type=int, default=5056)
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    bridge = RosBridge(node_name="k1_field_pose_relay")
    print(f"[relay] ROS2 话题: {ROS_TOPIC_POSE} (JSON)")
    print(f"[relay]            {ROS_TOPIC_POSE2D} (Pose2D)")
    print(f"[relay]            {ROS_TOPIC_BALL} (JSON)")

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((args.host, args.port))
    srv.listen(1)
    print(f"[relay] 监听 {args.host}:{args.port}，等待 Windows 侧 run.py 连接 ...")
    print("[relay] 提示：Windows 侧启动 "
          "run.py live --status-port 5056")

    n_pose = n_ball = 0
    conn = None
    try:
        while True:
            if conn is None:
                srv.settimeout(1.0)
                try:
                    conn, addr = srv.accept()
                except socket.timeout:
                    continue
                conn.settimeout(None)
                print(f"[relay] 已连接: {addr}")

            try:
                chunk = conn.recv(65536)
            except OSError as e:
                print(f"[relay] 接收出错: {e}")
                chunk = b""
            if not chunk:
                print("[relay] 对端断开，等待重连 ...")
                try:
                    conn.close()
                except OSError:
                    pass
                conn = None
                continue

            buf = getattr(main, "_buf", b"") + chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                if not line.strip():
                    continue
                try:
                    rec = json.loads(line.decode("utf-8"))
                except Exception:
                    continue
                kind = rec.pop("_kind", "pose")
                if kind == "pose":
                    bridge.publish_pose(rec)
                    n_pose += 1
                elif kind == "ball":
                    bridge.publish_ball([(b["label"], b["confidence"], b["bbox"])
                                         for b in rec.get("balls", [])])
                    n_ball += 1
                if not args.quiet and (n_pose + n_ball) % 60 == 0:
                    bridge.spin_once()
                    print(f"[relay] pose={n_pose} ball={n_ball}")
            main._buf = buf
    except KeyboardInterrupt:
        print("\n[relay] 退出")
    finally:
        if conn is not None:
            conn.close()
        srv.close()
        bridge.shutdown()


if __name__ == "__main__":
    main()
