"""用 ffmpeg（DirectShow）独立抓帧，交叉验证是否为 OpenCV 问题。

ffmpeg 与 OpenCV 的采集路径完全独立：
  - ffmpeg 有画面、OpenCV 黑屏  -> OpenCV / Python 绑定层的问题
  - ffmpeg 也黑屏               -> 系统层拦截（安全软件摄像头保护等）

用法（普通 PowerShell，venv 已激活）：
  python tools\\ffmpeg_cam_check.py
  python tools\\ffmpeg_cam_check.py --device "USB Camera" --size 1280x720
"""
import argparse
import re
import subprocess
import sys
from pathlib import Path

import cv2
import imageio_ffmpeg

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "logs"
FFMPEG = imageio_ffmpeg.get_ffmpeg_exe()


def run(args, timeout=60):
    p = subprocess.run([FFMPEG, "-hide_banner"] + args,
                       capture_output=True, timeout=timeout)
    raw = (p.stdout or b"") + (p.stderr or b"")
    # ffmpeg 按控制台代码页输出，中文系统上是 GBK，这里做兼容解码
    for enc in ("utf-8", "gbk", "mbcs"):
        try:
            text = raw.decode(enc)
            if "\ufffd" not in text:
                break
        except (UnicodeDecodeError, LookupError):
            continue
    else:
        text = raw.decode("utf-8", errors="replace")
    return p.returncode, text


def list_devices():
    _, out = run(["-list_devices", "true", "-f", "dshow", "-i", "dummy"])
    names = re.findall(r'"([^"]+)"\s*\(video\)', out)
    print("DirectShow 视频设备：")
    for n in names:
        print(f"  - {n}")
    if not names:
        print("  (未找到视频设备)")
        print(out[-800:])
    return names


def grab(device, size="1280x720", input_format="mjpeg"):
    OUT.mkdir(parents=True, exist_ok=True)
    jpg = OUT / "ffmpeg_frame.jpg"
    if jpg.exists():
        jpg.unlink()

    vf = f"scale={size.replace('x', ':')}"
    args = ["-f", "dshow", "-rtbufsize", "200M"]
    if input_format:
        args += ["-vcodec", input_format]
    args += ["-video_size", size, "-i", f"video={device}",
             "-frames:v", "3", "-y", str(jpg)]

    print(f"\n抓帧：device={device}  size={size}  codec={input_format or 'default'}")
    try:
        code, log = run(args, timeout=90)
    except subprocess.TimeoutExpired:
        print("  超时：ffmpeg 90 秒内没拿到帧")
        return None

    tail = [l for l in log.splitlines() if l.strip()][-6:]
    for l in tail:
        print(f"    {l}")

    if jpg.exists() and jpg.stat().st_size > 1000:
        img = cv2.imread(str(jpg))
        if img is not None:
            g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            print(f"  ✅ 出图 {img.shape[1]}x{img.shape[0]}  亮度 mean={g.mean():.1f} "
                  f"max={int(g.max())}  -> {jpg.relative_to(ROOT)}")
            return jpg
    print("  ❌ 没出图")
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", default=None)
    ap.add_argument("--list-raw", action="store_true", help="打印 ffmpeg 原始设备列表")
    ap.add_argument("--size", default="1280x720")
    ap.add_argument("--input-format", default="mjpeg",
                    choices=["mjpeg", "rawvideo", "none"])
    args = ap.parse_args()

    print(f"ffmpeg: {FFMPEG}\n")
    if args.list_raw:
        code, out = run(["-list_devices", "true", "-f", "dshow", "-i", "dummy"])
        print("=== ffmpeg 原始设备列表 ===")
        print(out)
        return

    devices = list_devices()
    if not args.device:
        # 优先选名字里带 USB / Camera 且不是内置 HP 摄像头的
        cand = [d for d in devices if "usb" in d.lower()]
        if not cand:
            cand = [d for d in devices if "camera" in d.lower()]
        if not cand:
            sys.exit("\n没有可用的视频设备，请确认相机已插好")
        print(f"\n自动选择：{cand[0]}（可用 --device 指定）")
        dev = cand[0]
    else:
        dev = args.device

    fmt = None if args.input_format == "none" else args.input_format
    img = grab(dev, args.size, fmt)
    if img is None and fmt == "mjpeg":
        print("\n改用原始流再试一次...")
        img = grab(dev, args.size, None)
    if img is None and args.size != "640x480":
        print("\n改 640x480 再试一次...")
        img = grab(dev, "640x480", None)


if __name__ == "__main__":
    main()
