"""颜色区域检测：离线流程与 ROS 2 节点共用一个检测核心。"""
from .core import Detection, DetectorConfig, detect

__all__ = ["Detection", "DetectorConfig", "detect"]
