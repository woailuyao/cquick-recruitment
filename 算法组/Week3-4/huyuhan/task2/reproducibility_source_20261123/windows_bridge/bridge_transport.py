"""One atomic latest-frame packet. No video replay or queued stale frames."""
import hashlib
import json
import os
import struct
import time
import uuid
from pathlib import Path

MAGIC = b"CQFRAME1\n"


def encode_packet(metadata, jpeg):
    meta = dict(metadata, jpeg_sha256=hashlib.sha256(jpeg).hexdigest())
    header = json.dumps(meta, ensure_ascii=True, allow_nan=False).encode("ascii")
    if len(header) > 65536 or len(jpeg) > 20000000:
        raise ValueError("Packet too large")
    return MAGIC + struct.pack(">I", len(header)) + header + jpeg


def decode_packet(data):
    if not data.startswith(MAGIC) or len(data) < len(MAGIC) + 4:
        raise ValueError("Invalid frame packet")
    n = struct.unpack(">I", data[len(MAGIC):len(MAGIC)+4])[0]
    offset = len(MAGIC) + 4
    if not 0 < n <= 65536 or not offset + n < len(data) <= 20100000:
        raise ValueError("Invalid packet length")
    meta = json.loads(data[offset:offset+n])
    if not isinstance(meta, dict):
        raise ValueError("Packet metadata must be an object")
    jpeg = data[offset+n:]
    if meta.get("source") != "windows_live_camera":
        raise ValueError("Input is not the live camera bridge")
    if not isinstance(meta.get("sequence"), int) or meta["sequence"] < 1:
        raise ValueError("Invalid sequence")
    if not isinstance(meta.get("session"), str) or not meta["session"]:
        raise ValueError("Missing source session")
    for key in ("capture_unix_ns", "width", "height"):
        if not isinstance(meta.get(key), int) or meta[key] <= 0:
            raise ValueError("Invalid " + key)
    if hashlib.sha256(jpeg).hexdigest() != meta.get("jpeg_sha256"):
        raise ValueError("JPEG checksum mismatch")
    return meta, jpeg


def atomic_write(path, data):
    path = Path(path)
    tmp = path.with_name(path.name + "." + str(os.getpid()) + ".tmp")
    try:
        tmp.write_bytes(data)
        for attempt in range(4):
            try:
                os.replace(tmp, path)
                return True
            except PermissionError:
                if attempt == 3:
                    return False
                time.sleep(.005)
    finally:
        tmp.unlink(missing_ok=True)


class ClockResponder:
    """Windows side: answer a fresh nonce using the SAME clock as capture."""
    def __init__(self, folder, session):
        self.folder = Path(folder)
        self.session = session
        self.last_nonce = None

    def poll(self):
        try:
            data = (self.folder / "clock_request.json").read_bytes()
            if len(data) > 4096:
                return
            request = json.loads(data)
            nonce = request.get("nonce")
            if not isinstance(nonce, str) or len(nonce) != 32 or nonce == self.last_nonce:
                return
            reply = dict(nonce=nonce, session=self.session,
                         windows_monotonic_ns=time.perf_counter_ns(),
                         windows_unix_ns=time.time_ns())
            if atomic_write(self.folder / "clock_reply.json", json.dumps(reply).encode()):
                self.last_nonce = nonce
        except (OSError, ValueError, AttributeError):
            return


class ClockClient:
    """Bound clock offset by a real file round trip, not matching wall clocks.

    Linux-minus-Windows monotonic offset lies between t_send-win_sample and
    t_receive-win_sample. Measurements expire; no offset is invented from
    incoming image ages. The additional 1% drift allowance is reported.
    """
    def __init__(self, folder):
        self.folder = Path(folder)
        self.pending = None
        self.sample = None
        self.last_request_ns = 0
        self.status = "clock_sync_waiting"

    def service(self):
        now = time.perf_counter_ns()
        if self.pending is not None:
            nonce, sent = self.pending
            try:
                data = (self.folder / "clock_reply.json").read_bytes()
                if len(data) > 4096:
                    raise ValueError("Clock reply too large")
                reply = json.loads(data)
                received = time.perf_counter_ns()
                if reply.get("nonce") == nonce:
                    sample_ns = reply.get("windows_monotonic_ns")
                    session = reply.get("session")
                    rtt = received-sent
                    if isinstance(sample_ns, int) and sample_ns > 0 and isinstance(session,str) and session and 0 <= rtt <= 750000000:
                        self.sample = dict(session=session, low_ns=sent-sample_ns,
                                           high_ns=received-sample_ns, measured_ns=received,
                                           rtt_ms=rtt/1e6)
                        self.status = "clock_synced"
                    else:
                        self.status = "clock_sync_roundtrip_too_slow_or_invalid"
                    self.pending = None
            except (OSError, ValueError, AttributeError):
                pass
            if self.pending is not None and now-self.pending[1] > 1000000000:
                self.pending = None
                self.status = "clock_sync_no_reply_restart_windows_v2"
        # Refresh every second. An unanswered request does not erase a recent
        # valid calibration, but that calibration always expires after 3 s.
        if self.pending is None and now-self.last_request_ns >= 1000000000:
            nonce = uuid.uuid4().hex
            sent = time.perf_counter_ns()
            try:
                self.folder.mkdir(parents=True, exist_ok=True)
                if atomic_write(self.folder/"clock_request.json",json.dumps(dict(nonce=nonce)).encode()):
                    self.pending = (nonce, sent)
                    self.last_request_ns = sent
            except OSError:
                self.status = "clock_sync_request_write_failed"

    def age_bounds(self, meta):
        sample = self.sample
        now = time.perf_counter_ns()
        if not isinstance(meta.get("capture_monotonic_ns"),int) or meta.get("protocol_version") != 2:
            return None, "bridge_version_mismatch_restart_windows_v2"
        if sample is None:
            return None, self.status
        if sample["session"] != meta["session"]:
            return None, "clock_sync_waiting_for_new_camera_session"
        elapsed = now-sample["measured_ns"]
        if elapsed < 0 or elapsed > 3000000000:
            return None, "clock_sync_expired"
        # Conservatively widen the measured bounds as calibration ages.
        drift = int(elapsed*.01)
        cap = meta["capture_monotonic_ns"]
        low_ms = (now-cap-sample["high_ns"]-drift)/1e6
        high_ms = (now-cap-sample["low_ns"]+drift)/1e6
        return dict(age_lower_ms=low_ms, age_upper_ms=high_ms,
                    clock_roundtrip_ms=sample["rtt_ms"],
                    calibration_age_ms=elapsed/1e6, drift_allowance_fraction=.01), "ok"


class LatestReader:
    def __init__(self, folder, max_age=1.5, clock=None):
        self.path = Path(folder) / "latest_frame.bin"
        self.max_age = max_age
        self.last = None
        self.clock = clock or ClockClient(folder)
        self.diagnostics = {}

    def check_age(self, meta):
        bounds, status = self.clock.age_bounds(meta)
        self.diagnostics = bounds or {}
        if bounds is None:
            return False, status
        if bounds["age_upper_ms"] < -50:
            return False, "clock_sync_inconsistent"
        if bounds["age_upper_ms"] > self.max_age*1000:
            return False, "stale_capture"
        return True, "ok"

    def poll(self):
        self.clock.service()
        try:
            # Read packet sequence directly; DrvFS stat caching must not be
            # used to decide whether a Windows-produced frame changed.
            data = self.path.read_bytes()
            meta, jpeg = decode_packet(data)
        except FileNotFoundError:
            return None, "waiting_for_windows_camera"
        except (OSError, ValueError, KeyError, TypeError, UnicodeError, struct.error) as e:
            return None, "invalid_packet: " + str(e)
        identity = (meta["session"], meta["sequence"])
        if self.last and identity[0] == self.last[0] and identity[1] <= self.last[1]:
            return None, "duplicate_or_out_of_order"
        fresh, status = self.check_age(meta)
        if not fresh:
            return None, status
        self.last = identity
        return (meta, jpeg), "ok"
