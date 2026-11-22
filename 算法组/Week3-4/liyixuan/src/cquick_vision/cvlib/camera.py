"""统一的相机/视频源打开逻辑，Linux(WSL/Ubuntu) 与 Windows 都能用。

为什么单独抽出来：Linux 要用 V4L2 并且必须指定 MJPG（否则 720p 常被限到 10 fps），
Windows 要用 DSHOW/MSMF；写死一个后端在另一个平台会直接打不开。
"""
from __future__ import annotations

import sys

import cv2


def _backend_candidates():
    """按平台给出要尝试的 backend 列表（顺序即优先级）。

    Windows 实测：DSHOW 在 1280x720 只能给到 ~10fps（拿不到 MJPG，退化成 YUY2），
    MSMF 能跑满 30fps，所以 MSMF 优先，DSHOW 作为兜底。
    """
    if sys.platform.startswith("win"):
        cands = []
        for name in ("CAP_MSMF", "CAP_DSHOW"):
            api = getattr(cv2, name, None)
            if api is not None:
                cands.append((name, api))
        cands.append(("ANY", None))
        return cands
    cands = []
    api = getattr(cv2, "CAP_V4L2", None)
    if api is not None:
        cands.append(("V4L2", api))
    cands.append(("ANY", None))
    return cands


def is_index(src) -> bool:
    return isinstance(src, int) or (isinstance(src, str) and src.strip().isdigit())


def open_capture(src, width: int = 1280, height: int = 720, fps: int = 30,
                 mjpg: bool = True, verbose: bool = True):
    """打开相机索引或视频文件，失败返回 None。"""
    if src is None:
        return None

    if not is_index(src):
        cap = cv2.VideoCapture(str(src))
        if not cap.isOpened():
            cap.release()
            return None
        if verbose:
            print(f"[cam] 视频文件 {src} {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}"
                  f"x{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}")
        return cap

    index = int(src)
    cap = None
    used = None
    for name, api in _backend_candidates():
        c = cv2.VideoCapture(index) if api is None else cv2.VideoCapture(index, api)
        if c is not None and c.isOpened():
            cap, used = c, name
            break
        if c is not None:
            c.release()

    if cap is None:
        return None

    # MJPG：720p 想跑满 30fps 必须用它。未压缩的 YUYV 会被 USB 带宽限到 10~15fps。
    # Windows(DSHOW) 也支持设置 FOURCC，所以两个平台都设。
    if mjpg:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    try:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    except Exception:
        pass

    if verbose:
        gw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        gh = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        cc = int(cap.get(cv2.CAP_PROP_FOURCC) or 0)
        cc_str = "".join(chr((cc >> (8 * i)) & 0xFF) for i in range(4)) if cc else "????"
        print(f"[cam] 相机 {index} 打开成功 backend={used} 实际={gw}x{gh} "
              f"fourcc={cc_str} 驱动报告fps={cap.get(cv2.CAP_PROP_FPS):.1f}")
        if (gw, gh) != (width, height):
            print(f"[warn] 实际分辨率 {gw}x{gh} != 请求 {width}x{height}")
        # MSMF 后端拿不到 MJPG 的 fourcc，但实测能跑满 30fps，所以这里不报警
        if mjpg and cc_str in ("YUY2", "YUYV", "I420", "NV12"):
            print("[warn] 当前是未压缩格式，720p 帧率可能只有 10fps 左右")
    return cap
