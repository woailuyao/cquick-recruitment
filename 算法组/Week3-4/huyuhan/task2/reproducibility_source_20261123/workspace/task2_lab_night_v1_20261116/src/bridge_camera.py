"""OpenCV-like adapter for the working, explicitly identified v2 live source."""
import importlib.util
import time
from pathlib import Path
import cv2
import numpy as np


class BridgeCamera:
    def __init__(self,folder,timeout=1.5):
        self.folder=Path(folder)
        module_path=self.folder.parent/'bridge_transport.py'
        if not module_path.is_file():
            raise RuntimeError('找不到已安装的v2桥接模块：'+str(module_path))
        spec=importlib.util.spec_from_file_location('cquick_v2_capture_protocol',module_path)
        module=importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        self.reader=module.LatestReader(self.folder)
        self.timeout=timeout
        self.open=True
        self.last_metadata=None
        self.last_status='waiting_for_windows_camera'

    def isOpened(self):return self.open

    def read(self):
        start=time.perf_counter()
        while self.open and time.perf_counter()-start<self.timeout:
            item,status=self.reader.poll()
            self.last_status=status
            if item is None:
                time.sleep(.01)
                continue
            meta,jpeg=item
            image=cv2.imdecode(np.frombuffer(jpeg,np.uint8),cv2.IMREAD_COLOR)
            if image is None or [image.shape[1],image.shape[0]] != [meta['width'],meta['height']]:
                self.last_status='bridge_decode_failed'
                continue
            self.last_metadata=meta
            return True,image
        print('实时桥接读帧失败：',self.last_status,self.reader.diagnostics,flush=True)
        return False,None

    def freshness(self):
        if self.last_metadata is None:return False,'no_camera_frame'
        return self.reader.check_age(self.last_metadata)

    def provenance(self):
        return dict(source_transport='windows_v2_shared_frame_bridge',producer=self.last_metadata,
                    clock_bounds=self.reader.diagnostics,source_status=self.last_status)

    def get(self,prop):
        m=self.last_metadata or {}
        if prop==cv2.CAP_PROP_FPS:return float(m.get('capture_loop_fps',0))
        if prop==cv2.CAP_PROP_FRAME_WIDTH:return float(m.get('width',0))
        if prop==cv2.CAP_PROP_FRAME_HEIGHT:return float(m.get('height',0))
        return 0

    def release(self):self.open=False
