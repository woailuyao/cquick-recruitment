"""网络视频源（HTTP MJPEG）读帧器：接口模仿 cv2.VideoCapture。

相机经 usbipd 直通进 WSL 时，UVC 等时传输在转发层会丢包（半帧 / 整片绿 +
内核刷 `urb->status -104`）。改用 Windows 侧 `tools/cam_server.py` 采集推流、
WSL 侧读 HTTP 流，就绕开了 USB 直通，链路稳定且与 usbip 无关。

命令行入口在 `tools/net_camera.py`；ROS 侧由 `video_publisher` 与
`tools/bench_realtime.py`、`localize.py` 的 `--source http://...` 复用本模块。

用法：

    # 单独自检：抓一帧存盘，并打印实际分辨率/帧率
    python tools/net_camera.py --url http://172.30.144.1:8080/stream.mjpg --frames 60

    # 在代码里用
    from tools.net_camera import NetCamera
    cap = NetCamera("http://172.30.144.1:8080/stream.mjpg")
    ok, frame = cap.read()

在 bench_realtime.py / localize.py 里，`--source http://.../stream.mjpg` 会自动走这里。
"""
from __future__ import annotations

import argparse
import sys
import time
import urllib.error
import urllib.request

import cv2
import numpy as np


class NetCamera:
    """把 HTTP multipart MJPEG 流包装成 cv2.VideoCapture 的最小接口。"""

    def __init__(self, url: str, timeout: float = 10.0, reconnect: bool = True):
        self.url = url
        self.timeout = timeout
        self.reconnect = reconnect
        self._resp = None
        self._buf = b""
        self._frame = None
        self._frames = 0
        self._t0 = time.time()
        self._closed = False
        self._open()

    # ---------- 连接 ----------
    def _open(self):
        req = urllib.request.Request(self.url, headers={"User-Agent": "cquick-net-camera"})
        self._resp = urllib.request.urlopen(req, timeout=self.timeout)
        self._buf = b""

    def isOpened(self) -> bool:
        return self._resp is not None and not self._closed

    def release(self):
        self._closed = True
        try:
            if self._resp is not None:
                self._resp.close()
        except Exception:
            pass
        self._resp = None

    # ---------- 取帧 ----------
    def _read_jpeg(self):
        """从 multipart 流里切出一帧 JPEG，返回 np.ndarray 或 None。"""
        if self._resp is None:
            return None
        while True:
            # 先找到 Content-Length
            head_end = self._buf.find(b"\r\n\r\n")
            if head_end < 0:
                chunk = self._resp.read(4096)
                if not chunk:
                    return None
                self._buf += chunk
                if len(self._buf) > 4 << 20:  # 4 MB 还没头，说明流坏了
                    self._buf = b""
                    return None
                continue
            head = self._buf[:head_end]
            body_start = head_end + 4
            length = None
            for line in head.split(b"\r\n"):
                if line.lower().startswith(b"content-length:"):
                    try:
                        length = int(line.split(b":", 1)[1].strip())
                    except ValueError:
                        length = None
            if length is None:
                # 不是我们认识的格式，丢一段重新找边界
                self._buf = self._buf[body_start:]
                continue
            while len(self._buf) < body_start + length:
                chunk = self._resp.read(65536)
                if not chunk:
                    return None
                self._buf += chunk
            data = self._buf[body_start:body_start + length]
            self._buf = self._buf[body_start + length:]
            arr = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
            if arr is not None:
                return arr
            # 解不出图就继续找下一帧

    def read(self):
        if self._closed:
            return False, None
        try:
            frame = self._read_jpeg()
        except (urllib.error.URLError, ConnectionResetError, TimeoutError, OSError):
            frame = None
        if frame is None:
            if not self.reconnect:
                self.release()
                return False, None
            # 断流了就重连一次
            self.release()
            time.sleep(0.3)
            try:
                self._closed = False
                self._open()
                frame = self._read_jpeg()
            except Exception:
                return False, None
            if frame is None:
                return False, None
        self._frames += 1
        self._frame = frame
        return True, frame

    # ---------- VideoCapture 常用属性 ----------
    def get(self, prop):
        if self._frame is None:
            return 0.0
        if prop == cv2.CAP_PROP_FRAME_WIDTH:
            return float(self._frame.shape[1])
        if prop == cv2.CAP_PROP_FRAME_HEIGHT:
            return float(self._frame.shape[0])
        if prop == cv2.CAP_PROP_FPS:
            dt = max(1e-6, time.time() - self._t0)
            return self._frames / dt
        if prop == cv2.CAP_PROP_POS_FRAMES:
            return float(self._frames)
        return 0.0

    def set(self, prop, value) -> bool:
        return False  # 网络流不支持改参数


def open_source(source: str):
    """统一入口：'http(s)://...' 走 NetCamera，其余交给 cv2.VideoCapture。"""
    s = str(source)
    if s.startswith(("http://", "https://")):
        return NetCamera(s)
    return cv2.VideoCapture(int(s) if s.isdigit() else s)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://127.0.0.1:8080/stream.mjpg")
    ap.add_argument("--frames", type=int, default=60)
    ap.add_argument("--out", default="out/net_check.jpg")
    a = ap.parse_args(argv)

    cap = NetCamera(a.url)
    if not cap.isOpened():
        print("打不开流：%s" % a.url)
        return 1
    print("已连上 %s" % a.url)
    n = 0
    t0 = time.time()
    last = None
    while n < a.frames:
        ok, frame = cap.read()
        if not ok:
            print("读帧失败（第 %d 帧）" % n)
            break
        last = frame
        n += 1
    dt = time.time() - t0
    cap.release()
    if last is None:
        return 1
    g = cv2.cvtColor(last, cv2.COLOR_BGR2GRAY)
    black = float((g.mean(axis=1) < 8).mean())
    flat = float(g.std()) < 3.0
    cv2.imwrite(a.out, last)
    print("收到 %d 帧，用时 %.1f s，平均 %.1f fps" % (n, dt, n / max(1e-6, dt)))
    print("分辨率 %dx%d  亮度 %.0f  黑行 %.0f%%  %s" % (
        last.shape[1], last.shape[0], float(g.mean()), black * 100,
        "整片纯色（没数据）" if flat else ("不完整" if black > 0.25 else "看起来正常")))
    print("样张：%s" % a.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
