"""为一次连续实机运行记录三个静止时间段，并在运行结束后计算误差。"""
import argparse
import csv
import json
import math
from pathlib import Path
import subprocess
import sys
import time
from datetime import datetime
import yaml


ROOT = Path(__file__).resolve().parent


def summarize(records, windows):
    result = []
    for point, start, stop, x, y, yaw in windows:
        frames = [r for r in records if start <= r["stamp"] < stop]
        valid = [r for r in frames if r["valid"]]
        row = [point, x, y, yaw, len(frames), len(valid)]
        if valid:
            first = valid[0]
            a, b, h = (first[key] for key in ("x_camera_map", "y_camera_map", "yaw_camera_map"))
            row += [first["frame_id"], a, b, h, round(math.hypot(a-x, b-y), 3),
                    round(abs((h-yaw+180) % 360-180), 2)]
        else:
            row += [""] * 6
        result.append(row)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera", type=int, default=0)
    args = parser.parse_args()
    if args.camera < 0:
        parser.error("设备编号应为非负整数")
    folder = ROOT/"outputs"/datetime.now().strftime("session_%Y%m%d_%H%M%S_%f")
    folder.mkdir(parents=True)
    cfg = yaml.safe_load((ROOT/"config/camera.yaml").read_text(encoding="utf8"))
    logfile = folder/"frames.jsonl"
    cfg["output"]["jsonl"] = logfile.as_posix()
    cfgfile = folder/"runtime.yaml"
    cfgfile.write_text(yaml.safe_dump(cfg), encoding="utf8")
    print("姓名: huangyirui", datetime.now().isoformat(timespec="seconds"), flush=True)
    print("记录目录", folder, flush=True)
    proc = subprocess.Popen([sys.executable, "-B", str(ROOT/"src/soccer_vision/run_demo.py"),
                             "--root", str(ROOT), "--config", str(cfgfile), "--source", str(args.camera), "--save-video", str(folder/"识别画面.mp4")])
    windows = []
    try:
        for point in range(1, 4):
            while True:
                raw = input(f"点 {point}：摆稳后，在终端输入实测 X Y Yaw（米 米 度，空格分隔）并回车，开始记录5秒：")
                if proc.poll() is not None:
                    print("相机程序已结束，请检查上方设备或依赖提示。")
                    return
                try:
                    values = list(map(float, raw.split()))
                    if len(values) == 3 and all(map(math.isfinite, values)):
                        break
                except ValueError:
                    pass
                print("需要三个数值，例如 -3.44 0.52 90")
            start = time.time()
            time.sleep(5)
            windows.append([point, start, time.time(), *values])
            (folder/"windows.json").write_text(json.dumps(windows), encoding="utf8")
            print("该点时间段已保存。", flush=True)
        print("可继续演示足球、遮挡和 ROS2。结束时点击相机预览窗口按 Q。", flush=True)
        proc.wait()
    finally:
        if proc.poll() is None:
            print("请在相机预览窗口按 Q 保存并关闭。", flush=True)
            proc.wait()
        if logfile.exists():
            records = [json.loads(line) for line in logfile.read_text(encoding="utf8").splitlines() if line.strip()]
            with (folder/"三点结果.csv").open("w", newline="", encoding="utf-8-sig") as stream:
                writer = csv.writer(stream)
                writer.writerow("点位 参考X_m 参考Y_m 参考Yaw_deg 总帧数 有效帧数 首个有效帧 估计X_m 估计Y_m 估计Yaw_deg 位置误差_m 航向误差_deg".split())
                for row in summarize(records, windows):
                    writer.writerow(row)
                    print(row)
            print("记录已保存", folder)


if __name__ == "__main__":
    main()

