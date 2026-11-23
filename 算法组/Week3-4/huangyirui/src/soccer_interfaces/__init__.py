#想知道一帧定位结果有哪些字段，可以读 pose_types.py。
#想知道这份结果如何保存或发给其他程序，可以接着读 ros2_stub.py。
#这里导出位姿类型和换算函数，定位结果可以写到文件或通过ROS2发出去

#从同一个文件夹下的 pose_types.py 里，把两个名字拿到当前文件来用。
#这样别处写 `from soccer_interfaces import FieldPose` 就能直接拿到这个类。
from .pose_types import FieldPose, camera_pose_from_carrier

#__all__ 是一个约定：写在清单里的名字，才允许被 `from 包名 import *` 一次性导出。
#不写也能跑，但写了能让读代码的人一眼看清这个包对外提供什么。
__all__ = ["FieldPose","camera_pose_from_carrier"]
