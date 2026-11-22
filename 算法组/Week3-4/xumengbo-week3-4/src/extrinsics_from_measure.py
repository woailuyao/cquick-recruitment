"""
外参测量 -> 外参矩阵换算骨架（extrinsics_from_measure）
=====================================================
现场用卷尺 / 电子水平仪各测三次，填入下面 TRIALS 列表，脚本取平均并
写回 calibration/camera_extrinsics.yaml，同时打印旋转/平移矩阵供复核。

⚠️ 必须实测：不得用估计值冒充标定值。三次测量离散度超差会提示重测。

测量约定（与 camera_extrinsics.yaml 注释一致）：
  height : 相机光心离地高度，单位 m（地面 Z=0）
  pitch  : 俯仰角，光轴向下俯视为负、向上为正，单位 deg
  roll   : 滚转角，绕光轴左右倾斜，单位 deg
  yaw    : 偏航角，光轴水平投影相对载体 +X 的夹角，单位 deg

运行：python extrinsics_from_measure.py
"""
import os
import yaml
import numpy as np

CALIB_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                         "calibration")
EXT_YAML = os.path.join(CALIB_DIR, "camera_extrinsics.yaml")

# ===== 现场实测填入（三次）；未测量前保持空列表，脚本不会写伪值 =====
HEIGHT_TRIALS = []   # 例: [0.62, 0.62, 0.63]  单位 m
PITCH_TRIALS = []    # 例: [-12.0, -12.5, -12.2] 单位 deg
ROLL_TRIALS = []     # 例: [0.5, 0.3, 0.4]  单位 deg
YAW_TRIALS = []      # 例: [3.0, 2.8, 3.1]  单位 deg

# 离散度阈值：超过则提示重测
HEIGHT_SPREAD = 0.01   # m，三次高度最大最小差 >1cm 报警
ANGLE_SPREAD = 1.0     # deg，三次角度差 >1° 报警


def mean_or_none(vals):
    if not vals:
        return None
    return float(np.mean(vals))


def check_spread(name, vals, threshold, unit):
    """检查三次测量离散度，超差打印警告。"""
    if len(vals) < 2:
        return
    spread = max(vals) - min(vals)
    if spread > threshold:
        print(f"[警告] {name} 三次离散 {spread:.2f}{unit} > {threshold}{unit}，建议重测")


def build_rotation_matrix(pitch_deg, roll_deg, yaw_deg):
    """欧拉角合成 R = Rz(yaw)·Ry(pitch)·Rx(roll)，与 loc_common 约定一致。"""
    ry, rp, rr = np.radians(yaw_deg), np.radians(pitch_deg), np.radians(roll_deg)
    Rz = np.array([[np.cos(ry), -np.sin(ry), 0], [np.sin(ry), np.cos(ry), 0], [0, 0, 1]])
    Ry = np.array([[np.cos(rp), 0, np.sin(rp)], [0, 1, 0], [-np.sin(rp), 0, np.cos(rp)]])
    Rx = np.array([[1, 0, 0], [0, np.cos(rr), -np.sin(rr)], [0, np.sin(rr), np.cos(rr)]])
    return Rz @ Ry @ Rx


def main():
    # 任一测量为空则不写文件，提示先现场测量
    if not (HEIGHT_TRIALS and PITCH_TRIALS and ROLL_TRIALS and YAW_TRIALS):
        raise SystemExit("外参尚未实测：请把三次测量填入脚本顶部 TRIALS 列表后再运行。")

    check_spread("height", HEIGHT_TRIALS, HEIGHT_SPREAD, "m")
    check_spread("pitch", PITCH_TRIALS, ANGLE_SPREAD, "deg")
    check_spread("roll", ROLL_TRIALS, ANGLE_SPREAD, "deg")
    check_spread("yaw", YAW_TRIALS, ANGLE_SPREAD, "deg")

    height = mean_or_none(HEIGHT_TRIALS)
    pitch = mean_or_none(PITCH_TRIALS)
    roll = mean_or_none(ROLL_TRIALS)
    yaw = mean_or_none(YAW_TRIALS)

    # 读取现有 yaml 并回填
    with open(EXT_YAML, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    data["calibration"]["date"] = "2026-11-19"   # 改为实测当天日期
    data["calibration"]["method"] = "卷尺测高度 + 电子水平仪测俯仰/滚转，各测 3 次取平均"
    data["camera_to_carrier"]["translation"] = {"x": 0.0, "y": 0.0, "z": height}
    data["camera_to_carrier"]["rotation"] = {"pitch": pitch, "roll": roll, "yaw": yaw}
    data["measurements"] = {
        "height_trials": HEIGHT_TRIALS,
        "pitch_trials": PITCH_TRIALS,
        "roll_trials": ROLL_TRIALS,
        "yaw_trials": YAW_TRIALS,
    }

    with open(EXT_YAML, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False, width=120)

    R = build_rotation_matrix(pitch, roll, yaw)
    print("=== 外参复核 ===")
    print(f"光心高度 z = {height:.3f} m")
    print(f"pitch={pitch:.2f}  roll={roll:.2f}  yaw={yaw:.2f} (deg)")
    print("旋转矩阵 R:\n", np.round(R, 4))
    print(f"\n已写回: {EXT_YAML}")


if __name__ == "__main__":
    main()
