"""摄像头采集脚本：录制原始视频，并记录考核要求的采集元数据。

为什么需要它：
  考核要求记录「摄像头接口、设备节点、分辨率、实际帧率、拍摄日期」，
  且 CAP_PROP_FPS 的返回值不一定等于真实采集帧率，必须用连续取帧的实际耗时测量。

输出：
  raw_videos/<tag>_<时间戳>.mp4     原始视频（本人保存备查，不传 GitHub）
  logs/capture_log.csv              采集记录，可直接贴进 README

用法：
  python tools/capture_video.py --list
  python tools/capture_video.py --index 0 --tag ball_run01 --duration 60
  python tools/capture_video.py --index 1 --tag ball_run02 --duration 45 --note "远距离+边缘"
  python tools/capture_video.py --index 0 --tag ball_run03 --duration 30 --no-preview

Linux 下设备节点建议写成 --device /dev/video0（等价于 --index 0）。
"""
import argparse
import csv
import datetime as dt
import sys
import time
from pathlib import Path

import cv2

from cam_open import open_camera

ROOT = Path(__file__).resolve().parents[1]
RAW_DIR = ROOT / "raw_videos"
LOG_CSV = ROOT / "logs" / "capture_log.csv"

BACKENDS = {
    "any": cv2.CAP_ANY,
    "dshow": cv2.CAP_DSHOW,
    "msmf": cv2.CAP_MSMF,
    "v4l2": getattr(cv2, "CAP_V4L2", cv2.CAP_ANY),
}


def resolve_backend(name: str):
    """auto：Windows 用 DSHOW（FFMPEG 后端枚举不到摄像头），Linux 用 V4L2。"""
    if name != "auto":
        return BACKENDS[name]
    if sys.platform == "win32":
        return BACKENDS["dshow"]
    if sys.platform.startswith("linux"):
        return BACKENDS["v4l2"]
    return cv2.CAP_ANY


def list_cameras(max_index, backend):
    print("扫描摄像头：")
    found = []
    for idx in range(max_index):
        cap, info = open_camera(idx, backend=args_backend_name(backend), verify_fps=None)
        if info["opened"] and info["width"] > 0 and info["measured_fps"] > 0:
            print(f"  index={idx}  {info['width']}x{info['height']}  "
                  f"[{info['fourcc']}]  实测 {info['measured_fps']:.1f} fps")
            found.append(idx)
        cap.release()
    if not found:
        print("  未发现可用摄像头：检查 USB 连接、权限（Linux 需加入 video 组）")
    return found


def args_backend_name(backend_int):
    for name, code in BACKENDS.items():
        if code == backend_int:
            return name
    return "auto"


def append_log(row):
    LOG_CSV.parent.mkdir(parents=True, exist_ok=True)
    new_file = not LOG_CSV.exists()
    with LOG_CSV.open("a", newline="", encoding="utf-8-sig") as f:
        writer = csv.writer(f)
        if new_file:
            writer.writerow([
                "日期", "标签", "文件名", "设备索引", "后端",
                "请求分辨率", "实际分辨率", "请求帧率", "实测帧率",
                "帧数", "时长s", "备注",
            ])
        writer.writerow(row)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--list", action="store_true", help="列出可用摄像头后退出")
    ap.add_argument("--max-index", type=int, default=2, help="--list 时扫描的索引上限"
                    "（本机实测 0=内置摄像头 1=考核用 USB 模组，2 是虚拟摄像头，扫到 2 即可；"
                    "换机器后如漏检可调大）")
    ap.add_argument("--index", type=int, default=0, help="摄像头索引（Linux 对应 /dev/videoN）")
    ap.add_argument("--backend", default="auto", choices=["auto", *BACKENDS],
                    help="auto=Windows用dshow / Linux用v4l2")
    ap.add_argument("--tag", default="run", help="片段标签，用于文件名，如 ball_run01")
    ap.add_argument("--duration", type=float, default=30.0, help="录制秒数")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--fps", type=float, default=30.0, help="写入视频的帧率（同时用于请求相机）")
    ap.add_argument("--fourcc", default="MJPG",
                    help="像素格式，默认 MJPG（720p 走 YUY2 在 USB2 上只有 10fps）；传 none 用驱动默认")
    ap.add_argument("--note", default="", help="场景备注（距离/位置/光照等）")
    ap.add_argument("--out", default="", help="视频保存目录（默认 raw_videos；可用绝对路径存到别的盘）")
    ap.add_argument("--no-preview", action="store_true", help="不弹预览窗口，仅录制")
    args = ap.parse_args()

    backend = resolve_backend(args.backend)

    if args.list:
        list_cameras(args.max_index, backend)
        return

    out_dir = Path(args.out).expanduser() if args.out else RAW_DIR
    if not out_dir.is_absolute():
        out_dir = ROOT / out_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    fourcc = None if str(args.fourcc).lower() in ("none", "default", "") else args.fourcc
    cap, cinfo = open_camera(args.index, width=args.width, height=args.height,
                             fps=args.fps, fourcc=fourcc,
                             backend=args.backend, verify_fps=None, verbose=True)
    if not cinfo["opened"]:
        sys.exit(f"错误：无法打开摄像头 index={args.index} backend={args.backend}\n"
                 f"  排查：1) --list 看索引  2) 换 --backend dshow/msmf/v4l2  "
                 f"3) 设备是否被其他程序占用  4) 检查 Windows 相机隐私权限")

    aw, ah = cinfo["width"], cinfo["height"]
    if (aw, ah) != (args.width, args.height):
        print(f"[警告] 请求 {args.width}x{args.height}，实际 {aw}x{ah}。")
        print("       标定、训练、验收必须使用同一分辨率，否则内参作废。")
    if cinfo["measured_fps"] < 20:
        print(f"[警告] 实测帧率仅 {cinfo['measured_fps']:.1f} fps，低于 20。")
        print("       USB2 带宽不足时 720p 只能跑 10fps，确认已走 MJPG（--fourcc MJPG）并插 USB3 口。")

    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    out_path = out_dir / f"{args.tag}_{stamp}.mp4"
    writer = cv2.VideoWriter(
        str(out_path), cv2.VideoWriter_fourcc(*"mp4v"), args.fps, (aw, ah)
    )
    if not writer.isOpened():
        sys.exit(f"错误：无法创建视频文件 {out_path}")

    print(f"录制中 -> {out_path.name}    按 q 提前结束")
    print(f"请求 {args.width}x{args.height}@{args.fps:.0f} | 实际 {aw}x{ah} "
          f"[{cinfo['fourcc']}] 实测 {cinfo['measured_fps']:.1f}fps")

    frames = 0
    t0 = time.time()
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("[警告] 取帧失败，可能设备断开")
                break
            writer.write(frame)
            frames += 1
            if not args.no_preview:
                cv2.imshow("capture  (q = stop)", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            if time.time() - t0 >= args.duration:
                break
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        elapsed = max(time.time() - t0, 1e-6)
        cap.release()
        writer.release()
        if not args.no_preview:
            cv2.destroyAllWindows()

    real_fps = frames / elapsed
    print("-" * 60)
    print(f"完成：{out_path}")
    print(f"  帧数 {frames} | 时长 {elapsed:.1f}s | 实测帧率 {real_fps:.1f} fps")
    if abs(real_fps - args.fps) > 3:
        print(f"  [注意] 实测帧率与目标 {args.fps:.0f} fps 差距较大，"
              f"检查 USB 带宽/光照/分辨率是否过高")

    try:
        rel = out_path.resolve().relative_to(ROOT.resolve())
        saved_as = str(rel).replace("\\", "/")
    except ValueError:
        saved_as = str(out_path.resolve()).replace("\\", "/")

    append_log([
        dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S"), args.tag, saved_as,
        args.index, args.backend, f"{args.width}x{args.height}", f"{aw}x{ah}",
        f"{args.fps:.0f}", f"{real_fps:.1f}", frames, f"{elapsed:.1f}", args.note,
    ])
    print(f"  采集记录已写入 {LOG_CSV}")


if __name__ == "__main__":
    main()
