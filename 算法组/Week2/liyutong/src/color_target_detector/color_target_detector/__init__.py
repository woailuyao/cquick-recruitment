"""Standalone OpenCV color-target detection; no ROS or GUI dependency.

Public API: DetectorConfig, DetectionResult, detect(image, config), and
annotate(image, result). Images are nonempty uint8 BGR NumPy arrays.
"""

from .detector import DetectionResult, DetectorConfig, annotate, detect

__all__ = ["DetectorConfig", "DetectionResult", "detect", "annotate"]
