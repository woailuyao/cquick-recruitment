#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
win_cam_bridge.py — Windows 侧摄像头 HTTP 桥（ffmpeg → MJPEG → 多客户端）

背景
----
usbipd 把 UVC 相机透传进 WSL 时，1280x720 MJPG 单帧数据量超出 USB/IP 的等时传输
能力，每帧被截断（画面下半部分全黑，dmesg 里 urb->status -104）。960x540 及以下
完整、只有 720p 坏，属协议层硬限制。因此改为：
    相机留在 Windows 原生采集 → HTTP 提供 JPEG 帧 → WSL 里做 YOLO 推理

为什么不用 ffmpeg 的 `-listen 1`：它只能服务一个连接，客户端一断开进程就退出，
而 Python 反复开窗/重连会很难用。这里让 ffmpeg 只编码一次，Python 服务广播。

接口
----
  GET /snapshot.jpg   最新一帧 JPEG（net_camera.py 用的就是它）
  GET /stream         multipart MJPEG 流（浏览器可直接打开预览）
  GET /status         JSON：帧数、实测 fps、分辨率

用法
----
  python win_cam_bridge.py --device "USB Camera" --port 8090
  python win_cam_bridge.py --device "USB Camera" --width 1280 --height 720 --fps 30
  python win_cam_bridge.py --list          # 先确认设备名
"""
import argparse
import json
import socket
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

SOI = b"\xff\xd8"
EOI = b"\xff\xd9"


def find_ffmpeg():
    try:
        import imageio_ffmpeg
        return imageio_ffmpeg.get_ffmpeg_exe()
    except Exception:
        pass
    from shutil import which
    exe = which("ffmpeg")
    if not exe:
        raise SystemExit("找不到 ffmpeg：请先 pip install imageio-ffmpeg")
    return exe


class FrameGrabber:
    """从 ffmpeg stdout 的 MJPEG 字节流里切出 JPEG 帧，保留最新一帧。"""

    def __init__(self, cmd):
        # bufsize=0 很关键：缓冲读的 read(n) 会等攒满 n 字节才返回，
        # 原始 MJPG 帧只有 ~20KB 时要等几十帧才凑满 1MB，造成秒级卡顿。
        self.proc = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0
        )
        self.latest = None
        self.count = 0
        self.lock = threading.Lock()
        self.err_lines = []
        self.t0 = time.time()
        self._stop = False
        threading.Thread(target=self._read_loop, daemon=True).start()
        threading.Thread(target=self._drain_err, daemon=True).start()

    def _drain_err(self):
        for raw in iter(self.proc.stderr.readline, b""):
            line = raw.decode("utf-8", "replace").rstrip()
            if line:
                self.err_lines.append(line)
                del self.err_lines[:-20]
                if "error" in line.lower() or "failed" in line.lower():
                    print("[ffmpeg]", line, flush=True)

    def _read_loop(self):
        buf = bytearray()
        while not self._stop:
            # bufsize=0 时 stdout 是原始 FileIO，read(n) 有多少返回多少，不会死等
            chunk = self.proc.stdout.read(65536)
            if not chunk:
                break
            buf += chunk
            while True:
                s = buf.find(SOI)
                if s < 0:
                    # 没有帧头，丢弃大部分缓冲但保留最后 1 字节（可能是 0xFF）
                    if len(buf) > 2:
                        del buf[:-1]
                    break
                if s > 0:
                    del buf[:s]
                e = buf.find(EOI, 2)
                if e < 0:
                    break  # 帧还没收完，等下一块
                frame = bytes(buf[: e + 2])
                del buf[: e + 2]
                if len(frame) > 1000:
                    with self.lock:
                        self.latest = frame
                        self.count += 1

    def snapshot(self):
        with self.lock:
            return self.latest

    def fps(self):
        dt = max(time.time() - self.t0, 1e-6)
        with self.lock:
            return self.count / dt

    def stop(self):
        self._stop = True
        try:
            self.proc.terminate()
        except Exception:
            pass


def make_handler(grabber, quality_note):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, fmt, *args):
            pass  # 静音访问日志

        def _jpeg(self):
            return grabber.snapshot()

        def do_GET(self):
            path = self.path.split("?")[0]
            if path in ("/snapshot.jpg", "/frame.jpg", "/"):
                img = self._jpeg()
                if img is None:
                    self.send_response(503)
                    self.send_header("Content-Length", "0")
                    self.end_headers()
                    return
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(img)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(img)
                return

            if path == "/status":
                body = json.dumps({
                    "frames": grabber.count,
                    "measured_fps": round(grabber.fps(), 2),
                    "mode": quality_note,
                    "ffmpeg_alive": grabber.proc.poll() is None,
                }).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return

            if path == "/stream":
                self.send_response(200)
                self.send_header("Content-Type",
                                 "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                last_count = -1
                try:
                    while True:
                        # 只在出现“新帧”时才发，避免把同一帧重复推给客户端
                        # （否则客户端统计的 FPS 会虚高）
                        with grabber.lock:
                            cnt, img = grabber.count, grabber.latest
                        if img is None or cnt == last_count:
                            time.sleep(0.002)
                            continue
                        last_count = cnt
                        self.wfile.write(b"--frame\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(b"Content-Length: %d\r\n\r\n" % len(img))
                        self.wfile.write(img)
                        self.wfile.write(b"\r\n")
                        self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError):
                    return
                return

            self.send_response(404)
            self.send_header("Content-Length", "0")
            self.end_headers()

    return Handler


def main():
    ap = argparse.ArgumentParser(description="Windows 摄像头 HTTP 桥")
    ap.add_argument("--list", action="store_true", help="列出 DirectShow 设备")
    ap.add_argument("--device", default="USB Camera", help="DirectShow 设备名")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8090)
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--fmt", default="mjpeg",
                    help="请求的像素格式；相机只支持 YUYV 时用 none")
    ap.add_argument("--quality", type=int, default=4,
                    help="JPEG 质量 2(最好)-31(最差)，仅重编码时生效")
    args = ap.parse_args()

    ff = find_ffmpeg()

    if args.list:
        p = subprocess.run(
            [ff, "-hide_banner", "-list_devices", "true", "-f", "dshow", "-i", "dummy"],
            capture_output=True, text=True, encoding="utf-8", errors="replace")
        for line in ((p.stderr or "") + (p.stdout or "")).splitlines():
            if "(video)" in line:
                print(line.strip())
        return

    is_mjpeg = (args.fmt or "").lower() == "mjpeg"
    cmd = [ff, "-hide_banner", "-loglevel", "error", "-f", "dshow"]
    if args.fmt and args.fmt.lower() not in ("none", "-"):
        cmd += ["-vcodec", args.fmt]
    cmd += ["-video_size", f"{args.width}x{args.height}",
            "-framerate", str(args.fps), "-rtbufsize", "256M"]
    cmd += ["-i", f"video={args.device}"]
    if is_mjpeg:
        cmd += ["-c", "copy"]
        note = "原始 MJPG 直传（零重压缩）"
    else:
        cmd += ["-c:v", "mjpeg", "-q:v", str(args.quality), "-pix_fmt", "yuvj420p"]
        note = f"重编码为 MJPEG（-q:v {args.quality}）"
    cmd += ["-f", "mjpeg", "pipe:1"]

    print("ffmpeg:", " ".join(f'"{c}"' if " " in c else c for c in cmd))
    grabber = FrameGrabber(cmd)

    # 等第一帧，确认相机真的出画面
    for _ in range(120):  # 最多等 12 秒
        if grabber.snapshot() is not None:
            break
        time.sleep(0.1)
    if grabber.snapshot() is None:
        print("错误：12 秒内没有收到任何帧，ffmpeg 输出：")
        for line in grabber.err_lines:
            print("  ", line)
        grabber.stop()
        sys.exit(1)

    print(f"编码方式: {note}")

    srv = ThreadingHTTPServer((args.host, args.port), make_handler(grabber, note))
    srv.daemon_threads = True
    print(f"\n服务已启动: http://127.0.0.1:{args.port}")
    print(f"  快照: http://127.0.0.1:{args.port}/snapshot.jpg")
    print(f"  流  : http://127.0.0.1:{args.port}/stream  （浏览器可直接看）")
    print(f"  状态: http://127.0.0.1:{args.port}/status")
    print("\nWSL 侧（镜像网络模式下直接用 127.0.0.1）：")
    print(f"  python realtime_detect.py --url http://127.0.0.1:{args.port}")
    print("\nCtrl+C 停止。\n")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\n停止中...")
    finally:
        grabber.stop()
        srv.server_close()


if __name__ == "__main__":
    main()
