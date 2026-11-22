"""统一的相机打开封装：稳定协商 MJPG + 1280x720 + 30fps。

两个实测结论（Windows DSHOW，务实避坑）：
  1. 必须先设分辨率，再设 FOURCC；顺序反了会被 DSHOW 重协商回 YUY2。
  2. 720p 的 YUY2 在 USB2 上只够 10fps，必须走 MJPG 才能 30fps。

用法：
    from cam_open import open_camera
    cap, info = open_camera(1, verbose=True)
    cap, info = open_camera(1, fourcc="YUY2")     # 需要裸流时
"""
import sys
import time

import cv2


def _backend_code(name):
    if name in (None, "auto"):
        return cv2.CAP_DSHOW if sys.platform.startswith("win") else getattr(cv2, "CAP_V4L2", cv2.CAP_ANY)
    table = {"dshow": cv2.CAP_DSHOW, "msmf": cv2.CAP_MSMF, "any": cv2.CAP_ANY}
    if hasattr(cv2, "CAP_V4L2"):
        table["v4l2"] = cv2.CAP_V4L2
    return table[name]


def fourcc_int(code):
    return cv2.VideoWriter_fourcc(*code)


def read_fourcc(cap):
    fc = int(cap.get(cv2.CAP_PROP_FOURCC))
    return "".join(chr((fc >> 8 * i) & 0xFF) for i in range(4) if 32 <= (fc >> 8 * i) & 0xFF < 127)


def _configure(cap, width, height, fps, fourcc):
    """按正确顺序协商格式：分辨率 -> 帧率 -> 像素格式。"""
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    if fps:
        cap.set(cv2.CAP_PROP_FPS, fps)
    if fourcc:
        cap.set(cv2.CAP_PROP_FOURCC, fourcc_int(fourcc))


def measure_fps(cap, seconds=1.0, warmup=6):
    for _ in range(warmup):
        cap.read()
    n, t0 = 0, time.time()
    while time.time() - t0 < seconds:
        ok, _ = cap.read()
        if ok:
            n += 1
    return n / max(time.time() - t0, 1e-6)


def open_camera(index=0, width=1280, height=720, fps=30, fourcc="MJPG",
                backend="auto", verify_fps=20.0, verbose=False):
    """打开相机并尽量协商到目标格式，返回 (cap, info)。

    info = {opened, width, height, fps, fourcc, measured_fps, attempts}
    """
    api = _backend_code(backend)
    attempts = []

    for attempt in range(2):
        cap = cv2.VideoCapture(index, api)
        if not cap.isOpened():
            attempts.append("open失败")
            cap.release()
            if attempt == 0:
                cap = cv2.VideoCapture(index, api,
                                       [cv2.CAP_PROP_FRAME_WIDTH, width,
                                        cv2.CAP_PROP_FRAME_HEIGHT, height])
            if not cap.isOpened():
                cap.release()
                continue

        _configure(cap, width, height, fps, fourcc)
        got = read_fourcc(cap)
        need = fourcc.upper() if fourcc else ""
        # 格式没生效时，重设一次分辨率再设格式（DSHOW 常见）
        if need and got.upper() != need:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            cap.set(cv2.CAP_PROP_FOURCC, fourcc_int(fourcc))
            got = read_fourcc(cap)

        measured = measure_fps(cap, seconds=1.0)
        attempts.append(f"{cap.get(cv2.CAP_PROP_FRAME_WIDTH):.0f}x"
                        f"{cap.get(cv2.CAP_PROP_FRAME_HEIGHT):.0f} "
                        f"[{got}] {measured:.0f}fps")

        if verify_fps is None or measured >= verify_fps:
            info = dict(opened=True,
                        width=int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                        height=int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
                        fps=cap.get(cv2.CAP_PROP_FPS),
                        fourcc=got, measured_fps=measured, attempts=attempts)
            if verbose:
                print(f"[cam {index}] " + attempts[-1] + " -> OK")
            return cap, info

        cap.release()  # 帧率不达标，重试一轮

    # 实在不行，回退到最普通的打开方式
    cap = cv2.VideoCapture(index, api)
    if not cap.isOpened():
        info = dict(opened=False, width=0, height=0, fps=0,
                    fourcc="", measured_fps=0, attempts=attempts)
        if verbose:
            print(f"[cam {index}] 打不开: {attempts}")
        return cap, info

    _configure(cap, width, height, fps, None)
    got = read_fourcc(cap)
    measured = measure_fps(cap, seconds=1.0)
    info = dict(opened=True,
                width=int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                height=int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
                fps=cap.get(cv2.CAP_PROP_FPS),
                fourcc=got, measured_fps=measured, attempts=attempts + [f"回退 {got} {measured:.0f}fps"])
    if verbose:
        print(f"[cam {index}] 回退模式: {got} {measured:.0f}fps")
    return cap, info


if __name__ == "__main__":
    idx = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    cap, info = open_camera(idx, verbose=True)
    print(info)
    cap.release()
