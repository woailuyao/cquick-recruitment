"""Standalone HSV color segmentation for uint8 OpenCV BGR images."""

from dataclasses import dataclass
import math
from numbers import Integral, Real
from typing import Optional, Tuple

import cv2
import numpy as np


@dataclass(frozen=True)
class DetectorConfig:
    """HSV bounds are inclusive; lower H > upper H wraps through red (179/0)."""

    hsv_lower: Tuple[int, int, int] = (5, 100, 80)
    hsv_upper: Tuple[int, int, int] = (25, 255, 255)
    min_area: float = 100
    morphology: str = "none"
    kernel_size: int = 3

    def __post_init__(self) -> None:
        for name in ("hsv_lower", "hsv_upper"):
            value = getattr(self, name)
            try:
                values = tuple(value)
            except TypeError as exc:
                raise ValueError(f"{name} must contain three integers") from exc
            if len(values) != 3 or any(
                isinstance(v, bool) or not isinstance(v, Integral) for v in values
            ):
                raise ValueError(f"{name} must contain three integers")
            if not (0 <= values[0] <= 179 and all(0 <= v <= 255 for v in values[1:])):
                raise ValueError(f"{name}: hue must be 0..179, saturation/value 0..255")
            object.__setattr__(self, name, tuple(int(v) for v in values))
        if any(self.hsv_lower[i] > self.hsv_upper[i] for i in (1, 2)):
            raise ValueError("lower saturation/value must not exceed upper saturation/value")
        if (
            isinstance(self.min_area, bool)
            or not isinstance(self.min_area, Real)
            or not math.isfinite(self.min_area)
            or self.min_area < 0
        ):
            raise ValueError("min_area must be a finite nonnegative number")
        object.__setattr__(self, "min_area", float(self.min_area))
        if self.morphology not in ("none", "open", "close", "open_close"):
            raise ValueError("morphology must be none, open, close, or open_close")
        if (
            isinstance(self.kernel_size, bool)
            or not isinstance(self.kernel_size, Integral)
            or self.kernel_size < 1
        ):
            raise ValueError("kernel_size must be a positive integer")
        object.__setattr__(self, "kernel_size", int(self.kernel_size))


@dataclass
class DetectionResult:
    """Pixel centroid (u, v), contour area, selected contour, and full binary mask.

    On a miss, detected is False, u/v/area are zero, contour is None, and
    mask still contains the thresholded, optionally cleaned segmentation.
    """

    detected: bool
    u: float
    v: float
    area: float
    contour: Optional[np.ndarray]
    mask: np.ndarray


def _validate_image(image: np.ndarray) -> None:
    if (
        not isinstance(image, np.ndarray)
        or image.dtype != np.uint8
        or image.ndim != 3
        or image.shape[2] != 3
        or image.shape[0] == 0
        or image.shape[1] == 0
    ):
        raise ValueError("image must be a nonempty H x W x 3 uint8 BGR array")


def detect(image: np.ndarray, config: DetectorConfig = DetectorConfig()) -> DetectionResult:
    """Return the largest external contour with area >= min_area and valid moments.

    The input is not modified. Centroids use OpenCV moments in pixel coordinates.
    The returned uint8 mask has values 0/255 and the same height/width as image.
    """
    _validate_image(image)
    if not isinstance(config, DetectorConfig):
        raise TypeError("config must be a DetectorConfig")
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower, upper = config.hsv_lower, config.hsv_upper
    if lower[0] <= upper[0]:
        mask = cv2.inRange(hsv, lower, upper)
    else:
        high = cv2.inRange(hsv, lower, (179, upper[1], upper[2]))
        low = cv2.inRange(hsv, (0, lower[1], lower[2]), upper)
        mask = cv2.bitwise_or(high, low)
    if config.morphology != "none":
        kernel = np.ones((config.kernel_size, config.kernel_size), dtype=np.uint8)
        if config.morphology in ("open", "open_close"):
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        if config.morphology in ("close", "open_close"):
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    # [-2] supports both OpenCV 3 and OpenCV 4 findContours return signatures.
    contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    candidates = sorted(
        ((float(cv2.contourArea(contour)), contour) for contour in contours),
        key=lambda item: item[0],
        reverse=True,
    )
    for area, contour in candidates:
        if area < config.min_area:
            break
        moments = cv2.moments(contour)
        if moments["m00"] == 0:
            continue
        return DetectionResult(
            True,
            float(moments["m10"] / moments["m00"]),
            float(moments["m01"] / moments["m00"]),
            area,
            contour,
            mask,
        )
    return DetectionResult(False, 0.0, 0.0, 0.0, None, mask)


def annotate(image: np.ndarray, result: DetectionResult) -> np.ndarray:
    """Return a copy with contour, center, and metrics, or a 'No target' label."""
    _validate_image(image)
    output = image.copy()
    if result.detected:
        if result.contour is not None:
            cv2.drawContours(output, [result.contour], -1, (0, 255, 0), 2)
        center = (int(round(result.u)), int(round(result.v)))
        cv2.circle(output, center, 4, (0, 0, 255), -1)
        label = f"Target u={result.u:.1f} v={result.v:.1f} area={result.area:.1f}"
        color = (0, 255, 0)
    else:
        label, color = "No target", (0, 0, 255)
    cv2.putText(output, label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA)
    return output
