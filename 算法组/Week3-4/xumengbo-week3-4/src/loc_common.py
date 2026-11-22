"""
定位链路公共模块（loc_common）
================================
职责：加载内参 / 外参 / 场地地图配置，提供 SE(2) 位姿变换等基础工具。
被 landmark / ground_projection / map_matcher / field_locator 复用。

坐标系约定（与 field_geometry.yaml 一致）：
  地图系：原点=场地中心，+X 朝球门，+Y 面朝 +X 左侧，Z 向上，地面 Z=0，单位米。
  载体系：机器人底座 / 移动测试架参考原点，轴向与地图系平行。
  相机系：光心为原点，光轴 +Z，图像 +u 向右、+v 向下。
"""
import os
import math
import yaml
import numpy as np

# 项目根目录（本文件在 tools/ 下，上一级即项目根）
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG_DIR = os.path.join(BASE_DIR, "config")
CALIB_DIR = os.path.join(BASE_DIR, "calibration")


def load_yaml(path):
    """读取 YAML 为字典；文件不存在时抛出明确错误，避免静默用空配置。"""
    if not os.path.exists(path):
        raise FileNotFoundError(f"配置文件不存在: {path}")
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


# ---------------- 内参 ----------------
class Intrinsics:
    """相机内参与畸变，来自 config/camera_intrinsics.yaml。"""

    def __init__(self, K, dist, w, h):
        self.K = np.array(K, dtype=float)          # 3x3 内参矩阵
        self.dist = np.array(dist, dtype=float)   # [k1,k2,p1,p2,k3]
        self.w = int(w)
        self.h = int(h)

    @classmethod
    def from_config(cls, path=None):
        path = path or os.path.join(CONFIG_DIR, "camera_intrinsics.yaml")
        d = load_yaml(path)
        cm = d["camera_matrix"]
        K = cm["matrix"]
        dist = d["dist_coeffs"]["coeffs"]
        return cls(K, dist, d["image_width"], d["image_height"])


# ---------------- 外参 ----------------
class Extrinsics:
    """相机相对载体系的固定安装参数，来自 calibration/camera_extrinsics.yaml。

    注意：height / pitch / roll / yaw 必须现场实测；配置里全 0 表示尚未标定，
    此时定位链路应输出 valid=false，不能把 0 当成真实外参。
    """

    def __init__(self, translation, rotation_deg, measured, orientation_base="legacy"):
        self.t = np.array(translation, dtype=float)   # 载体系下相机光心平移 (x,y,z)
        self.rot_deg = rotation_deg                     # {pitch, roll, yaw} 度
        self.measured = measured                         # 是否已现场实测
        # 安装基准：forward=水平朝前（光轴朝 +X）；legacy=相机 Z 与载体系 Z 对齐
        self.orientation_base = orientation_base

    @classmethod
    def from_config(cls, path=None):
        path = path or os.path.join(CALIB_DIR, "camera_extrinsics.yaml")
        d = load_yaml(path)
        cam = d["camera_to_carrier"]
        tr = cam["translation"]
        rot = cam["rotation"]
        translation = [tr["x"], tr["y"], tr["z"]]
        rotation_deg = {"pitch": rot["pitch"], "roll": rot["roll"], "yaw": rot["yaw"]}
        # 三次实测记录非空，才认为外参已标定
        measured = bool(d.get("measurements", {}).get("height_trials"))
        orientation_base = cam.get("orientation_base", "legacy")
        return cls(translation, rotation_deg, measured, orientation_base)

    def rotation_matrix(self):
        """由欧拉角合成 3x3 旋转矩阵 R（相机系 -> 载体系）。

        小角度部分沿用 R = Rz(yaw)·Ry(pitch)·Rx(roll)。
        orientation_base="forward" 时，先套水平朝前基准 R_base：
          相机 Z(前) -> 载体系 +X；相机 X(右) -> 载体系 −Y；相机 Y(下) -> 载体系 −Z。
        轴序为骨架约定；现场外参标定后需按实际安装方向复核。
        """
        ry = math.radians(self.rot_deg["yaw"])
        rp = math.radians(self.rot_deg["pitch"])
        rr = math.radians(self.rot_deg["roll"])

        Rz = np.array([[math.cos(ry), -math.sin(ry), 0],
                       [math.sin(ry), math.cos(ry), 0],
                       [0, 0, 1]])
        Ry = np.array([[math.cos(rp), 0, math.sin(rp)],
                       [0, 1, 0],
                       [-math.sin(rp), 0, math.cos(rp)]])
        Rx = np.array([[1, 0, 0],
                       [0, math.cos(rr), -math.sin(rr)],
                       [0, math.sin(rr), math.cos(rr)]])
        R_small = Rz @ Ry @ Rx

        if self.orientation_base == "forward":
            R_base = np.array([[0, 0, 1],
                               [-1, 0, 0],
                               [0, -1, 0]], dtype=float)
            return R_base @ R_small
        return R_small

    def T_camera_to_carrier(self):
        """4x4 齐次变换矩阵 T_base_camera（相机系 -> 载体系）。"""
        T = np.eye(4)
        T[:3, :3] = self.rotation_matrix()
        T[:3, 3] = self.t
        return T


# ---------------- 场地地图 ----------------
# 类别前缀 -> 地图分组名
TYPE_PREFIX = ("LCross", "TCross", "XCross")


class FieldMap:
    """加载 field.yaml（或联调用 field_theory.yaml）的已知标志世界坐标。

    markers: {类型: [(名称, x, y), ...]}，类型取键名前缀。
    """

    def __init__(self, markers, map_version="C0-v1"):
        self.markers = markers
        self.map_version = map_version

    @classmethod
    def from_yaml(cls, path):
        d = load_yaml(path)
        markers = {t: [] for t in TYPE_PREFIX}
        for key, val in d.items():
            if key == "map_version":
                continue
            # 找到键名属于哪个前缀（LCross1 / TCross10 / XCross2 / PenalityPoint）
            for prefix in TYPE_PREFIX:
                if key.startswith(prefix):
                    x, y = float(val[0]), float(val[1])
                    markers[prefix].append((key, x, y))
                    break
        return cls(markers, d.get("map_version", "C0-v1"))

    def candidates(self, marker_type):
        """返回某类别在地图上的全部候选点 [(name, x, y)]。"""
        return self.markers.get(marker_type, [])


# ---------------- SE(2) 工具 ----------------
def se2_matrix(x, y, yaw_deg):
    """生成 2D 位姿的 3x3 齐次变换矩阵 [[R, t], [0,1]]，yaw 单位度。"""
    a = math.radians(yaw_deg)
    R = np.array([[math.cos(a), -math.sin(a)],
                  [math.sin(a), math.cos(a)]])
    T = np.eye(3)
    T[:2, :2] = R
    T[:2, 2] = [x, y]
    return T


def transform_points(pose, pts):
    """把载体系点集 pts (N,2) 用位姿 pose=(x,y,yaw_deg) 变换到地图系。"""
    x, y, yaw = pose
    T = se2_matrix(x, y, yaw)
    pts = np.asarray(pts, dtype=float)
    hom = np.hstack([pts, np.ones((len(pts), 1))])
    out = (T @ hom.T).T[:, :2]
    return out


def wrap_angle_deg(angle):
    """把角度归一到 (-180, 180]。"""
    return (angle + 180.0) % 360.0 - 180.0
