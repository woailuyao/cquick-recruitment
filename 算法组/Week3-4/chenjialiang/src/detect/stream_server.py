# MJPEG 推流服务：把 RGB 帧推成本地网页流
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

class StreamServer:
    def __init__(self, port=8001, quality=80, host="0.0.0.0"):
        self.port = port
        self.quality = quality
        self.host = host
        self._jpeg = None
        self._seq = 0
        self._lock = threading.Lock()
        self._httpd = None
        self._thread = None

    def push(self, frame):
        ok, buf = cv2.imencode(".jpg", frame,
                               [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        if not ok:
            return
        with self._lock:
            self._jpeg = buf.tobytes()
            self._seq += 1

    def latest(self):
        with self._lock:
            return self._jpeg, self._seq

    def _make_handler(self):
        outer = self

        class Handler(BaseHTTPRequestHandler):
            disable_nagle_algorithm = True

            def do_GET(self):
                if self.path not in ("/stream", "/"):
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header(
                    "Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                try:
                    last = -1
                    while True:
                        data, seq = outer.latest()
                        if data is None or seq == last:
                            time.sleep(0.002)
                            continue
                        last = seq
                        self.wfile.write(
                            b"--frame\r\nContent-Type: image/jpeg\r\n"
                            b"Content-Length: " + str(len(data)).encode()
                            + b"\r\n\r\n" + data + b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    pass

            def log_message(self, *args):
                pass

        return Handler

    def start(self):
        self._httpd = ThreadingHTTPServer((self.host, self.port),
                                          self._make_handler())
        self._httpd.daemon_threads = True
        self._thread = threading.Thread(target=self._httpd.serve_forever,
                                        daemon=True)
        self._thread.start()
        print(f"[stream] 推流已启动 http://localhost:{self.port}/stream")
        return self

    def stop(self):
        if self._httpd is not None:
            self._httpd.shutdown()
            self._httpd.server_close()
            print("[stream] 已关闭")

if __name__ == "__main__":
    import numpy as np

    srv = StreamServer(port=8001).start()
    n = 0
    try:
        while True:
            img = np.full((540, 960, 3), 40, np.uint8)
            cv2.putText(img, f"test {n}", (60, 300),
                        cv2.FONT_HERSHEY_SIMPLEX, 3, (0, 255, 255), 6)
            cv2.circle(img, (480 + int(200 * np.sin(n / 10)), 270), 40,
                       (0, 255, 0), -1)
            srv.push(img)
            n += 1
            time.sleep(1 / 30)
    except KeyboardInterrupt:
        srv.stop()
