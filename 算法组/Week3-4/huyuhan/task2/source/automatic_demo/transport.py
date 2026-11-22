"""Use the installed v2 protocol and retain both age measurements."""
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
import time

import cv2
import numpy as np

from localize import ROOT
from pose import MAX_CAPTURE_AGE_S
from prototype import other_readers, sha
from session import Frame


def load_protocol(path):
    spec = spec_from_file_location("cquick_auto_bridge_protocol", path)
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def observed_reader(protocol, folder, clock=None):
    class ObservedReader(protocol.LatestReader):
        def check_age(self, meta):
            good, status = super().check_age(meta)
            self.checked = dict(source_session=meta["session"], sequence=meta["sequence"],
                                capture_monotonic_ns=meta["capture_monotonic_ns"],
                                status=status, **self.diagnostics)
            return good, status

    reader = ObservedReader(folder, max_age=MAX_CAPTURE_AGE_S, clock=clock)
    reader.checked = {}
    return reader


class Bridge:
    def __init__(self, folder):
        self.folder = folder.resolve()
        self.protocol_path = self.folder.parent / "bridge_transport.py"
        protocol = load_protocol(self.protocol_path)
        self.reader = observed_reader(protocol, self.folder)
        self.provenance = dict(source="windows_bridge_v2", bridge_folder=str(self.folder),
                               protocol_path=str(self.protocol_path), protocol_sha256=sha(self.protocol_path),
                               capture_age_limit_ms=MAX_CAPTURE_AGE_S * 1000)

    def next_frame(self):
        start = time.perf_counter()
        pids = other_readers(self.folder)
        if pids:
            return None, "reader_conflict", dict(conflicting_pids=pids)
        self.reader.checked = {}
        last_status = "waiting_for_windows_camera"
        rejected = None
        while time.perf_counter() - start < .22:
            item, status = self.reader.poll()
            if item is None:
                if status != "duplicate_or_out_of_order" or rejected is None:
                    last_status = status
                if status != "duplicate_or_out_of_order" and self.reader.checked.get("status") == status:
                    rejected = dict(self.reader.checked)
                time.sleep(.01)
                continue
            meta, jpeg = item
            image = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
            if image is None or list(image.shape[:2][::-1]) != [meta["width"], meta["height"]]:
                return None, "bridge_decode_failed", dict(self.reader.checked)
            frame = Frame(image, meta["session"], meta["sequence"],
                          meta["capture_monotonic_ns"] / 1e9, meta)
            return frame, status, dict(self.reader.checked)
        return None, last_status, rejected or {}

    def fresh(self, frame):
        fresh, status = self.reader.check_age(frame.producer)
        return fresh, status, dict(self.reader.checked)
