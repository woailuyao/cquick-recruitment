"""共用相机模块。

重点：UVC 相机默认协商 YUYV，1280x720@30 需要约 442 Mbps 带宽，
在 VirtualBox 的 USB2.0(EHCI) 通道上必然 select() timeout。
所以这里优先用 MJPG，失败再逐级降级，并把能用的参数打印出来。
"""
import time

import cv2

TARGET_W, TARGET_H = 1280, 720
TARGET_FPS = 30

# (设备号, 宽, 高, fourcc, fps)  按优先级排列
CANDIDATES = [
    (0, 1280, 720, "MJPG", 30),
    (0, 1280, 720, "MJPG", 0),
    (0, 1280, 720, "YUYV", 30),
    (0, 640, 480, "MJPG", 30),
    (0, 640, 480, "YUYV", 30),
    (1, 1280, 720, "MJPG", 30),
    (1, 640, 480, "MJPG", 30),
    (2, 1280, 720, "MJPG", 30),
]


def configure(cap, fourcc, width, height, fps):
    """向驱动下发格式参数，切换格式需要一点时间让驱动重新协商。"""
    if fourcc:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    if fps:
        cap.set(cv2.CAP_PROP_FPS, fps)


def first_frame(cap, tries=10):
    """连续读若干帧，拿到第一帧非空画面。读不到返回 None。"""
    for _ in range(tries):
        ok, frame = cap.read()
        if ok and frame is not None and frame.size > 0:
            return frame
    return None


def describe(cap):
    fcc = int(cap.get(cv2.CAP_PROP_FOURCC))
    fourcc = "".join(chr((fcc >> 8 * i) & 0xFF) for i in range(4))
    return {
        "fourcc": "".join(c for c in fourcc if c.isprintable()) or "?",
        "width": int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
        "height": int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
        "fps": float(cap.get(cv2.CAP_PROP_FPS)),
    }


def measure_fps(cap, frames=20):
    """实测帧率。CAP_PROP_FPS 只是驱动回报值，不一定等于真实出帧速度。"""
    t0 = time.perf_counter()
    n = 0
    for _ in range(frames):
        ok, _ = cap.read()
        if not ok:
            break
        n += 1
    dt = time.perf_counter() - t0
    return round(n / dt, 1) if n and dt > 0 else 0.0


def open_camera(index=0, width=TARGET_W, height=TARGET_H, fourcc="MJPG", fps=TARGET_FPS):
    """按指定参数打开相机，失败直接抛异常。"""
    cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(f"/dev/video{index} 打不开（节点不存在或被占用）")
    configure(cap, fourcc, width, height, fps)
    frame = first_frame(cap)
    if frame is None:
        cap.release()
        raise RuntimeError(f"/dev/video{index} 打开成功但取不到帧（多半是格式/带宽问题）")
    return cap, frame, describe(cap)


def auto_open(verbose=True):
    """按 CANDIDATES 逐个尝试，返回第一个能出帧的组合。"""
    errors = []
    for index, width, height, fourcc, fps in CANDIDATES:
        tag = f"/dev/video{index} {fourcc} {width}x{height}" + (f"@{fps}" if fps else "")
        try:
            cap, frame, info = open_camera(index, width, height, fourcc, fps)
        except Exception as exc:  # noqa: BLE001
            errors.append(f"{tag}: {exc}")
            if verbose:
                print(f"  [x] {tag} -> {exc}")
            continue
        if verbose:
            print(
                f"  [v] {tag} -> 实际 {info['fourcc']} "
                f"{info['width']}x{info['height']} {info['fps']:.1f}fps"
            )
        return cap, frame, info
    raise RuntimeError("所有相机参数都失败：\n" + "\n".join(errors))


if __name__ == "__main__":
    cap, frame, info = auto_open()
    print("可用：", info, "frame:", frame.shape)
    print("实测帧率：", measure_fps(cap), "fps")
    cap.release()
