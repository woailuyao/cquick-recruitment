"""
外参单点反算自检（check_extrinsics_projection）
================================================
用途：在地面放已知真实坐标的标记，用画面像素反算地面点，对比卷尺真实位置，
      检验外参平移/角度是否填对。纯投影计算，不依赖模型检测。

现场步骤：
  1. 地面放 3 个以上标记（近/中/远、左/中/右），卷尺量出真实 (X_true, Y_true)；
  2. 相机拍到标记，读出标记在画面里的像素 (u, v)（u 向右、v 向下，画面中心 640,360）；
  3. 把每个点填进下面 MARKERS 列表；
  4. 运行：python check_extrinsics_projection.py

判据：单点平面误差十几厘米内为合理；平均超 0.5m 需排查外参。
"""
import os
import sys
import math

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import loc_common
import ground_projection

# 现场标记清单：(名称, 真实X(m), 真实Y(m), 像素u, 像素v)
# 像素从实拍帧/截图里读；u 向右、v 向下，1280x720 画面中心为 (640, 360)。
# 下面是占位示例，现场必须替换成实测点（真实坐标和像素都要填）。
MARKERS = [
    ("P1", -0.5, 0.5, 497, 626),
    ("P2",  0.5, 0.0, 668, 531),
    ("P3", -1.0,-0.5, 945, 648),
]


def main():
    intr = loc_common.Intrinsics.from_config()
    ext = loc_common.Extrinsics.from_config()

    if not ext.measured:
        print("外参尚未实测（measured=False），请先回填 measurements 三次读数。")
        return

    proj = ground_projection.GroundProjector(intr, ext, field_half=(3.9, 3.0))

    errs = []
    print(f"{'名称':<6} {'真实(X,Y)':<18} {'反算(X,Y)':<18} {'误差(cm)':<10} 状态")
    print("-" * 72)

    for name, xt, yt, u, v in MARKERS:
        res = proj.project_one(u, v)
        if not res["valid"]:
            print(f"{name:<6} ({xt:>5.2f},{yt:>5.2f})  {'--':<18} {'--':<10} 无效: {res['reason']}")
            continue
        xb, yb = res["x_b"], res["y_b"]
        err = math.hypot(xb - xt, yb - yt)
        errs.append(err)
        print(f"{name:<6} ({xt:>5.2f},{yt:>5.2f})  ({xb:>5.2f},{yb:>5.2f})  {err*100:>8.1f}  ok")

    print("-" * 72)
    if errs:
        mean_err = sum(errs) / len(errs)
        print(f"平均平面误差: {mean_err*100:.1f} cm；最大 {max(errs)*100:.1f} cm")
        if mean_err <= 0.15:
            print("结论：平均误差在十几厘米内，外参自检通过。")
        elif mean_err <= 0.5:
            print("结论：误差在 0.5m 内但偏大，建议复核高度/俯仰/偏航符号。")
        else:
            print("结论：误差超 0.5m，需排查外参（见下方顺序）。")
    else:
        print("没有有效反算点，检查像素坐标或外参。")

    print("\n偏差排查顺序：1) 高度 z  2) pitch 俯仰符号/角度  3) yaw 偏航符号  4) roll  5) 内参畸变")


if __name__ == "__main__":
    main()
