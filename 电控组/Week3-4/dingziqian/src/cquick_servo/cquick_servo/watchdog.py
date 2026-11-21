import threading
import time


class SafetyWatchdog:
    """Latch a fault when heartbeat callbacks stop for too long."""

    def __init__(self, on_timeout, timeout=1.0):
        self.on_timeout = on_timeout
        self.timeout = timeout
        self.last_heartbeat = time.monotonic()
        self.tripped = False
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.timeout_complete = threading.Event()
        self.timeout_complete.set()
        self.thread = threading.Thread(
            target=self._run, daemon=True
        )

    def start(self):
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        if self.thread.is_alive():
            self.thread.join(timeout=3.0)

    def feed(self):
        with self.lock:
            self.last_heartbeat = time.monotonic()

    def is_tripped(self):
        with self.lock:
            return self.tripped

    def reset(self):
        with self.lock:
            if not self.timeout_complete.is_set():
                return False
            if time.monotonic() - self.last_heartbeat > self.timeout:
                return False
            self.tripped = False
            return True

    def check(self):
        with self.lock:
            if self.tripped:
                return
            if time.monotonic() - self.last_heartbeat <= self.timeout:
                return
            self.tripped = True
            self.timeout_complete.clear()
        try:
            self.on_timeout()
        finally:
            self.timeout_complete.set()

    def _run(self):
        while not self.stop_event.wait(0.05):
            self.check()
