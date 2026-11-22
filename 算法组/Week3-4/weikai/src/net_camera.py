"""net_camera.py — WSL 侧网络相机客户端（读 Windows 的摄像头桥）

提供与 cv2.VideoCapture 对齐的 read() 接口，realtime_detect.py 用 --url 切换。

两种模式：
  stream  —— 连 /stream 持久连接，按 JPEG SOI/EOI 切帧（默认，速度快）
  poll    —— 逐帧 GET /snapshot.jpg（兼容模式，每帧一次 HTTP，慢）

为什么默认用 stream：实测 poll 模式每帧要 ~80ms（新建连接 + 请求开销），
把 GPU 上只需 15.7ms 的推理拖成 10 FPS；stream 模式复用连接后取帧开销可忽略。

用法（自测）:
  python net_camera.py --host 127.0.0.1 --port 8090 --frames 60 --mode stream
"""
import argparse
import time

import cv2
import numpy as np
import requests

SOI = b"\xff\xd8"
EOI = b"\xff\xd9"


class NetCamera:
    """网络相机：与 cv2.VideoCapture.read() 同签名"""

    def __init__(self, base_url, mode="stream", timeout=5.0, fps_timeout=10.0):
        self.base = base_url.rstrip("/")
        self.mode = mode
        self.timeout = timeout
        self.fps_timeout = fps_timeout
        self._buf = bytearray()
        self._resp = None
        if mode == "stream":
            try:
                self._open_stream()
            except Exception:
                self.mode = "poll"
        if self.mode == "poll":
            self.sess = requests.Session()

    # ---------- stream 模式 ----------
    def _open_stream(self):
        # stream=True 让连接保持，逐块读；timeout 是 (连接, 读块) 超时
        self._resp = requests.get(self.base + "/stream", stream=True,
                                  timeout=(self.timeout, self.fps_timeout))

    def _read_stream(self):
        while True:
            s = self._buf.find(SOI)
            if s < 0:
                if len(self._buf) > 4:
                    del self._buf[:-2]
                need = True
            else:
                if s > 0:
                    del self._buf[:s]
                e = self._buf.find(EOI, 2)
                if e > 0:
                    raw = bytes(self._buf[:e + 2])
                    del self._buf[:e + 2]
                    img = cv2.imdecode(np.frombuffer(raw, np.uint8),
                                       cv2.IMREAD_COLOR)
                    if img is not None:
                        return True, img
                    continue
                need = True
            chunk = self._resp.raw.read(65536, decode_content=False)
            if not chunk:
                return False, None
            self._buf += chunk

    # ---------- poll 模式 ----------
    def _read_poll(self):
        try:
            r = self.sess.get(self.base + "/snapshot.jpg", timeout=self.timeout)
            if r.status_code != 200:
                return False, None
            img = cv2.imdecode(np.frombuffer(r.content, np.uint8),
                               cv2.IMREAD_COLOR)
            return (img is not None), img
        except requests.RequestException:
            return False, None

    # ---------- 公共接口 ----------
    def isOpened(self):
        return self.mode == "stream" or hasattr(self, "sess")

    def read(self):
        if self.mode == "stream":
            try:
                return self._read_stream()
            except Exception:
                # 流断了就重连一次，仍失败则交给上层判定
                try:
                    self._open_stream()
                    self._buf.clear()
                    return self._read_stream()
                except Exception:
                    return False, None
        return self._read_poll()

    def get(self, prop):
        # 兼容 cv2 的 cap.get(3)/cap.get(4) 调用习惯
        if prop in (3, "width"):
            return self._w if hasattr(self, "_w") else 1280
        if prop in (4, "height"):
            return self._h if hasattr(self, "_h") else 720
        return 0

    def release(self):
        try:
            if self._resp is not None:
                self._resp.close()
        except Exception:
            pass


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--host", default="127.0.0.1", help="Windows 主机地址")
    p.add_argument("--port", type=int, default=8090)
    p.add_argument("--frames", type=int, default=60)
    p.add_argument("--mode", default="stream", choices=["stream", "poll"])
    p.add_argument("--save", default=None)
    args = p.parse_args()

    url = "http://%s:%d" % (args.host, args.port)
    # 先看服务状态
    try:
        st = requests.get(url + "/status", timeout=5).json()
        print("status:", st)
    except Exception as e:
        raise SystemExit("无法访问 %s/status: %s（服务是否启动？）" % (url, e))

    cam = NetCamera(url, mode=args.mode)
    print("模式:", cam.mode)

    n, bad, last = 0, 0, None
    t0 = time.time()
    while n < args.frames:
        ok, f = cam.read()
        if not ok:
            bad += 1
            if bad > 30:
                raise SystemExit("连续取帧失败")
            continue
        last = f
        n += 1
    dt = time.time() - t0
    print("frames=%d 取帧 FPS=%.2f 失败=%d 分辨率=%dx%d"
          % (n, n / dt, bad, last.shape[1], last.shape[0]))
    if args.save:
        cv2.imwrite(args.save, last)
        print("saved:", args.save)


if __name__ == "__main__":
    main()
