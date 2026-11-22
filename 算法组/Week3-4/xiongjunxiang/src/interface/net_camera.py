#!/usr/bin/env python3
"""把网络来的 JPEG 流包装成 OpenCV 风格的相机对象。

协议（与 tools/win_camera_sender.py 对应）
    每帧：4 字节大端长度 + JPEG 数据

连接方向（重要）
    这里是【服务端】：先在 WSL 里监听，等 Windows 端的发送脚本连上来。
    Windows -> WSL 的 localhost 转发默认开启，发送端连 127.0.0.1:5599 即可到达，
    不需要查 WSL 的 IP，也不用改 .wslconfig。

行为
    读取时把已到达的旧帧全部丢掉，只返回最新一帧，
    避免发送端帧率高于处理速度时画面越来越滞后。
"""

import select
import socket
import struct

import cv2
import numpy as np


class NetCamera:
    """接收端（服务端）：监听端口，等推流端连接，按帧读 JPEG 并解码。"""

    def __init__(self, port=5599, host="0.0.0.0", timeout=15.0):
        self.port = int(port)
        self.host = host
        self.timeout = timeout
        self.dropped = 0
        self.frames = 0
        self._sock = None
        self._conn = None
        self._w = 0
        self._h = 0
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._sock.bind((self.host, self.port))
            self._sock.listen(1)
            self._sock.settimeout(timeout)
            print("网络相机：等待推流端连接 %s:%d ..." % (self.host, self.port))
            self._conn, addr = self._sock.accept()
            self._conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self._conn.settimeout(10.0)
            print("网络相机：已连接 %s:%d" % addr)
        except OSError as e:
            print("网络相机：监听失败 %s:%d -> %s" % (self.host, self.port, e))
            self._sock = None
            self._conn = None

    # ---------------------------------------------------------- cv2 兼容
    def isOpened(self):
        return self._conn is not None

    def read(self):
        ok, frame = self._decode_one()
        if not ok:
            return False, None
        n = 0
        while n < 100 and self._pending():
            ok2, f2 = self._decode_one()
            if not ok2:
                break
            frame = f2
            n += 1
        self.dropped += n
        self.frames += 1
        return True, frame

    def get(self, prop):
        if prop == cv2.CAP_PROP_FRAME_WIDTH:
            return float(self._w)
        if prop == cv2.CAP_PROP_FRAME_HEIGHT:
            return float(self._h)
        if prop == cv2.CAP_PROP_FPS:
            return 30.0
        return 0.0

    def set(self, prop, value):
        return True

    def release(self):
        for s in (self._conn, self._sock):
            try:
                if s is not None:
                    s.close()
            except OSError:
                pass
        self._conn = None
        self._sock = None

    # ---------------------------------------------------------- 内部
    def _pending(self):
        if self._conn is None:
            return False
        try:
            r, _, _ = select.select([self._conn], [], [], 0)
            return bool(r)
        except (OSError, ValueError):
            return False

    def _decode_one(self):
        if self._conn is None:
            return False, None
        try:
            hdr = self._recv_exact(4)
            if hdr is None:
                return False, None
            n = struct.unpack(">I", hdr)[0]
            if n == 0 or n > 20 * 1024 * 1024:
                print("网络相机：帧长度异常 %d" % n)
                return False, None
            buf = self._recv_exact(n)
            if buf is None:
                return False, None
            frame = cv2.imdecode(np.frombuffer(buf, np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                return False, None
            self._h, self._w = frame.shape[:2]
            return True, frame
        except (socket.timeout, ConnectionResetError, OSError) as e:
            print("网络相机：读帧中断 %s" % e)
            self.release()
            return False, None

    def _recv_exact(self, n):
        buf = b""
        while len(buf) < n:
            try:
                chunk = self._conn.recv(n - len(buf))
            except socket.timeout:
                return None
            if not chunk:
                return None
            buf += chunk
        return buf


def open_camera(spec, default_port=5599):
    """tcp:5599 / tcp://host:port / 纯端口号 -> NetCamera；否则返回 None。"""
    s = str(spec).strip().lower()
    if not (s.startswith("tcp:") or s.startswith("tcp://") or s.isdigit()):
        return None
    port = default_port
    s2 = s.replace("tcp://", "").replace("tcp:", "")
    if ":" in s2:
        port = int(s2.rsplit(":", 1)[1])
    elif s2.isdigit() and len(s2) <= 5:
        port = int(s2)
    return NetCamera(port)