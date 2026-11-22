#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rosimg.py —— sensor_msgs/Image <-> numpy 的零依赖转换

为什么不直接用 cv_bridge：
    1. Ubuntu 22.04 自带的是 numpy 1.x 编译的 cv_bridge，装 numpy 2.x 的 venv
       里 import 会抛 `AttributeError: _ARRAY_API not found`（虽然有时被 except 吞掉，
       真到 imgmsg_to_cv2 时才炸），排查起来很浪费时间；
    2. cv_bridge 还会悄悄把 rgb8 当成 bgr8 处理或改色序，反过来让"红球变蓝球"；
    3. 这里只做一件事：按 encoding 把 msg.data 重新排成 HxWxC 的 uint8 数组，
       逻辑一眼能看懂，跨 numpy 1/2 都稳定。

支持的编码：bgr8 / rgb8 / bgra8 / rgba8 / mono8 / 8UC1 / 8UC3 / yuv422(yuyv) / bayer_*8
"""

import array

import numpy as np

try:  # cv2 只用来做色序/灰度/去马赛克转换，缺了也能跑 bgr8/rgb8
    import cv2
except Exception:  # pragma: no cover
    cv2 = None


_BAYER_CODES = {
    "bayer_rggb8": None,      # 用 cv2 常量，见下
    "bayer_bggr8": None,
    "bayer_gbrg8": None,
    "bayer_grbg8": None,
}


def _reshape(msg, channels):
    """把 msg.data 按 step/height/width 重排成 HxWxC，兼容行尾有 padding 的驱动。

    性能提示（实测 1280x720）：msg.data 类型是 array.array('B')，用 memoryview
    直接 zero-copy 取数组是 0.2 ms；如果先 bytes(msg.data) 也只多 0.1 ms。
    """
    h, w, step = int(msg.height), int(msg.width), int(msg.step)
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    row_bytes = w * channels
    if step <= 0 or step == row_bytes:
        return buf[: h * row_bytes].reshape(h, w, channels)
    # 行尾有 padding：逐行切片再拼
    rows = [buf[i * step: i * step + row_bytes] for i in range(h)]
    return np.stack(rows, axis=0).reshape(h, w, channels)


def imgmsg_to_bgr(msg):
    """sensor_msgs/Image -> BGR ndarray（HxWx3 uint8）。不支持的编码抛 ValueError。"""
    enc = (msg.encoding or "bgr8").strip().lower()

    if enc in ("bgr8", "8uc3"):
        img = _reshape(msg, 3)
        return np.ascontiguousarray(img)
    if enc == "rgb8":
        img = _reshape(msg, 3)
        if cv2 is None:
            return np.ascontiguousarray(img[:, :, ::-1])
        return cv2.cvtColor(np.ascontiguousarray(img), cv2.COLOR_RGB2BGR)
    if enc in ("bgra8", "rgba8"):
        img = _reshape(msg, 4)
        if cv2 is None:
            idx = [2, 1, 0] if enc == "rgba8" else [0, 1, 2]
            return np.ascontiguousarray(img[:, :, idx])
        code = cv2.COLOR_RGBA2BGR if enc == "rgba8" else cv2.COLOR_BGRA2BGR
        return cv2.cvtColor(np.ascontiguousarray(img), code)
    if enc in ("mono8", "8uc1"):
        img = _reshape(msg, 1)[:, :, 0]
        if cv2 is None:
            return np.repeat(img[:, :, None], 3, axis=2)
        return cv2.cvtColor(np.ascontiguousarray(img), cv2.COLOR_GRAY2BGR)
    if enc == "mono16" or enc == "16uc1":
        img = np.frombuffer(bytes(msg.data), dtype=np.uint16)[: int(msg.height) * int(msg.width)]
        img = img.reshape(int(msg.height), int(msg.width))
        img8 = (img.astype(np.float32) / 257.0).clip(0, 255).astype(np.uint8)
        if cv2 is None:
            return np.repeat(img8[:, :, None], 3, axis=2)
        return cv2.cvtColor(img8, cv2.COLOR_GRAY2BGR)
    if enc in ("yuv422", "yuyv", "yuv422_yuyv", "yuv422_yuyv8"):
        img = _reshape(msg, 2)
        if cv2 is None:
            raise ValueError("yuv 编码需要 opencv")
        return cv2.cvtColor(np.ascontiguousarray(img), cv2.COLOR_YUV2BGR_YUYV)
    if enc in ("uyvy", "yuv422_uyvy"):
        img = _reshape(msg, 2)
        if cv2 is None:
            raise ValueError("yuv 编码需要 opencv")
        return cv2.cvtColor(np.ascontiguousarray(img), cv2.COLOR_YUV2BGR_UYVY)
    if enc in _BAYER_CODES:
        if cv2 is None:
            raise ValueError("bayer 编码需要 opencv")
        img = _reshape(msg, 1)[:, :, 0]
        code = {
            "bayer_rggb8": cv2.COLOR_BayerRG2BGR,
            "bayer_bggr8": cv2.COLOR_BayerBG2BGR,
            "bayer_gbrg8": cv2.COLOR_BayerGB2BGR,
            "bayer_grbg8": cv2.COLOR_BayerGR2BGR,
        }[enc]
        return cv2.cvtColor(np.ascontiguousarray(img), code)

    raise ValueError(
        "不支持的图像编码 %r；请让相机驱动发 bgr8/rgb8/mono8，"
        "或在 rosimg.imgmsg_to_bgr 里补一条转换" % msg.encoding)


def bgr_to_imgmsg(frame, stamp=None, frame_id="camera", encoding="bgr8"):
    """BGR ndarray -> sensor_msgs/Image。stamp 传 rclpy 的 Time 消息（可选）。"""
    from sensor_msgs.msg import Image  # 延迟导入，方便不装 ROS 时复用本文件

    if frame is None:
        raise ValueError("frame 为空")
    arr = np.ascontiguousarray(frame)
    if arr.dtype != np.uint8:
        arr = arr.astype(np.uint8)
    if arr.ndim == 2:  # 灰度
        arr = np.repeat(arr[:, :, None], 3, axis=2)
    h, w = arr.shape[:2]
    enc = encoding.strip().lower()
    if enc == "rgb8":
        arr = np.ascontiguousarray(arr[:, :, ::-1])
    elif enc not in ("bgr8",):
        raise ValueError("bgr_to_imgmsg 只支持 bgr8/rgb8，收到 %r" % encoding)

    msg = Image()
    msg.height = int(h)
    msg.width = int(w)
    msg.encoding = enc
    msg.is_bigendian = 0
    msg.step = int(w * 3)
    # 关键：一定要给 array.array('B')，不能直接给 bytes。
    # rosidl 的 python 生成器对 bytes 会逐元素校验/转换，1280x720 实测 127 ms/帧；
    # 换成 array.array('B') 后是 0.15 ms/帧（差 800 倍，直接决定 FPS 能不能到 30）。
    msg.data = array.array("B", arr.tobytes())
    if stamp is not None:
        msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    return msg
