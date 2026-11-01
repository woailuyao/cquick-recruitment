# -*- coding: utf-8 -*-
"""detector.py 的单元测试（不需要 ROS2，普通 pytest 就能跑）。

运行：
    cd src/cquick_vision
    python3 -m pytest test -v
或者在整个工作空间里：
    colcon test --packages-select cquick_vision
"""
import numpy as np
import pytest

from cquick_vision.detector import BlueTargetDetector


def make_blue_image(cx: int, cy: int, radius: int, size=(480, 640)) -> np.ndarray:
    """画一个蓝色实心圆（BGR），当作"目标"。"""
    h, w = size
    img = np.full((h, w, 3), 200, dtype=np.uint8)      # 浅灰背景
    yy, xx = np.mgrid[0:h, 0:w]
    mask = (xx - cx) ** 2 + (yy - cy) ** 2 <= radius ** 2
    img[mask] = (200, 90, 40)                          # BGR 蓝色
    return img


def test_detect_center():
    """蓝色圆的中心应该被算出来，误差在几个像素内。"""
    det = BlueTargetDetector(min_area=200.0)
    res = det.detect(make_blue_image(320, 240, 60))
    assert res.detected is True
    assert abs(res.u - 320) < 3
    assert abs(res.v - 240) < 3
    assert res.area > 10000
    assert res.center_method == "moments"


def test_no_target_returns_zeros():
    """没有蓝色目标时：detected=False，而且 u/v/area 必须是 0，不能是上一帧的残留。"""
    det = BlueTargetDetector(min_area=200.0)
    blank = np.full((480, 640, 3), 200, dtype=np.uint8)
    res = det.detect(blank)
    assert res.detected is False
    assert res.u == 0.0 and res.v == 0.0 and res.area == 0.0
    assert res.center_method == "none"


def test_small_blob_filtered_by_min_area():
    """面积小于阈值的候选要被过滤掉。"""
    img = make_blue_image(100, 100, 8)                 # 半径 8 -> 面积约 200
    assert BlueTargetDetector(min_area=800.0).detect(img).detected is False
    assert BlueTargetDetector(min_area=100.0).detect(img).detected is True


def test_largest_is_selected():
    """两个候选时，largest 规则选面积大的那个。"""
    img = make_blue_image(150, 150, 50)
    h, w = img.shape[:2]
    yy, xx = np.mgrid[0:h, 0:w]
    img[(xx - 500) ** 2 + (yy - 300) ** 2 <= 20 ** 2] = (200, 90, 40)
    res = BlueTargetDetector(min_area=200.0, select="largest").detect(img)
    assert res.detected is True
    assert res.candidates == 2
    assert abs(res.u - 150) < 3 and abs(res.v - 150) < 3


def test_bad_hsv_raises():
    """上界小于下界这种明显错误要当场报错，而不是静默输出 0。"""
    with pytest.raises(ValueError):
        BlueTargetDetector(hsv_lower=(130, 90, 45), hsv_upper=(100, 255, 255))


def test_empty_image_raises():
    with pytest.raises(ValueError):
        BlueTargetDetector().detect(np.zeros((0, 0, 3), dtype=np.uint8))
