from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np


@dataclass
class DetectorConfig:
    h_min: int = 5
    h_max: int = 25
    s_min: int = 100
    s_max: int = 255
    v_min: int = 70
    v_max: int = 255
    min_area: int = 800
    kernel_size: int = 5
    morph_mode: str = "open_close"

    def validate(self) -> None:
        if not 0 <= self.h_min <= self.h_max <= 179:
            raise ValueError(
                "HSV 的 H 范围必须满足 0 <= h_min <= h_max <= 179"
            )
        if not 0 <= self.s_min <= self.s_max <= 255:
            raise ValueError(
                "HSV 的 S 范围必须满足 0 <= s_min <= s_max <= 255"
            )
        if not 0 <= self.v_min <= self.v_max <= 255:
            raise ValueError(
                "HSV 的 V 范围必须满足 0 <= v_min <= v_max <= 255"
            )
        if self.min_area < 0:
            raise ValueError("min_area 不能为负数")
        if self.kernel_size < 1 or self.kernel_size % 2 == 0:
            raise ValueError("kernel_size 必须是正奇数")
        if self.morph_mode not in {
            "none",
            "open",
            "close",
            "open_close",
            "close_open",
        }:
            raise ValueError(
                "morph_mode 只能是 "
                "none/open/close/open_close/close_open"
            )


@dataclass
class Detection:
    detected: bool
    u: float = 0.0
    v: float = 0.0
    area: float = 0.0
    contour: Optional[np.ndarray] = None


def read_image(path: str) -> Optional[np.ndarray]:
    """读取图片，并兼容中文路径。"""
    try:
        data = np.fromfile(path, dtype=np.uint8)
    except (OSError, ValueError):
        return None

    if data.size == 0:
        return None
    return cv2.imdecode(data, cv2.IMREAD_COLOR)


def write_image(path: str, image: np.ndarray) -> bool:
    """写入图片，并兼容中文路径。"""
    suffix = "." + path.rsplit(".", 1)[-1] if "." in path else ".png"
    ok, encoded = cv2.imencode(suffix, image)
    if not ok:
        return False
    encoded.tofile(path)
    return True


def create_mask(
    image_bgr: np.ndarray,
    config: DetectorConfig,
) -> np.ndarray:
    config.validate()

    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    lower = np.array(
        [config.h_min, config.s_min, config.v_min],
        dtype=np.uint8,
    )
    upper = np.array(
        [config.h_max, config.s_max, config.v_max],
        dtype=np.uint8,
    )
    mask = cv2.inRange(hsv, lower, upper)

    if config.morph_mode != "none":
        kernel = np.ones(
            (config.kernel_size, config.kernel_size),
            dtype=np.uint8,
        )

        if config.morph_mode in {"open", "open_close"}:
            mask = cv2.morphologyEx(
                mask,
                cv2.MORPH_OPEN,
                kernel,
            )

        if config.morph_mode in {"close", "close_open"}:
            mask = cv2.morphologyEx(
                mask,
                cv2.MORPH_CLOSE,
                kernel,
            )

        if config.morph_mode == "close_open":
            mask = cv2.morphologyEx(
                mask,
                cv2.MORPH_OPEN,
                kernel,
            )

    return mask


def detect_target(
    image_bgr: np.ndarray,
    config: DetectorConfig,
) -> Detection:
    if image_bgr is None or image_bgr.size == 0:
        raise ValueError("输入图像为空")

    mask = create_mask(image_bgr, config)
    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )

    candidates = []
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area >= config.min_area:
            candidates.append((area, contour))

    if not candidates:
        return Detection(detected=False)

    area, target_contour = max(
        candidates,
        key=lambda item: item[0],
    )
    moments = cv2.moments(target_contour)

    if moments["m00"] <= 0.0:
        return Detection(detected=False)

    u = moments["m10"] / moments["m00"]
    v = moments["m01"] / moments["m00"]
    return Detection(
        detected=True,
        u=float(u),
        v=float(v),
        area=float(area),
        contour=target_contour,
    )


def draw_detection(
    image_bgr: np.ndarray,
    detection: Detection,
    config: DetectorConfig,
) -> np.ndarray:
    output = image_bgr.copy()

    if detection.detected and detection.contour is not None:
        cv2.drawContours(
            output,
            [detection.contour],
            -1,
            (0, 255, 0),
            3,
        )
        center = (
            round(detection.u),
            round(detection.v),
        )
        cv2.circle(output, center, 6, (0, 0, 255), -1)

        text = (
            f"u={detection.u:.1f}, v={detection.v:.1f}, "
            f"area={detection.area:.1f}"
        )
        cv2.putText(
            output,
            text,
            (12, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
    else:
        cv2.putText(
            output,
            "TARGET NOT DETECTED",
            (12, 36),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )

    parameter_text = (
        f"HSV H[{config.h_min},{config.h_max}] "
        f"S[{config.s_min},{config.s_max}] "
        f"V[{config.v_min},{config.v_max}] "
        f"area>={config.min_area} "
        f"morph={config.morph_mode}"
    )
    cv2.putText(
        output,
        parameter_text,
        (12, output.shape[0] - 14),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )

    return output
