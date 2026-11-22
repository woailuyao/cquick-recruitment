# -*- coding: utf-8 -*-
"""几何自检：验证像素<->地面投影、视场范围、SE(2) 求解是否正确。

运行：python selftest_geometry.py
"""
import math
import numpy as np

from geometry import GroundProjector, fit_se2, solve_pose_with_matching

# ---- 你的实机参数 ----
K = np.array([[1621.833, 0.0, 589.579],
              [0.0, 1624.377, 323.090],
              [0.0, 0.0, 1.0]])
HEIGHT = 0.98      # m
PITCH = 30.0       # deg，镜头俯视
W, H = 1280, 720

proj = GroundProjector(K, HEIGHT, PITCH, max_range_m=50.0)

print("=== 1. 视场能看到的地面距离 ===")
near, far = proj.ground_band(W, H)
print(f"  画面最下边缘 ≈ {far:.2f} m，最上边缘 ≈ {near:.2f} m")
zc = proj.R @ np.array([0, 0, 1.0])
print(f"  光轴落地距离 = {HEIGHT / math.tan(math.radians(PITCH)):.2f} m")
print(f"  光轴方向(载体) = {np.round(zc, 3)}  (X向前, Z向上 -> 应为前下方)")

print("\n=== 2. 像素<->地面 往返一致性 ===")
for (X, Y) in [(1.5, 0.0), (1.7, 0.0), (2.0, 0.4), (2.5, -0.5), (1.2, 0.0)]:
    uv = proj.pixel_from_ground(X, Y)
    if uv is None:
        print(f"  ({X}, {Y}) 不在视场内")
        continue
    back = proj.ground_from_pixel(*uv)
    ok = back is not None and abs(back[0] - X) < 1e-6 and abs(back[1] - Y) < 1e-6
    print(f"  地面({X:+.2f},{Y:+.2f}) -> 像素({uv[0]:7.1f},{uv[1]:6.1f}) -> 反算"
          f"({back[0]:+.3f},{back[1]:+.3f})  {'OK' if ok else 'FAIL'}")

print("\n=== 3. 地平线之上的像素应判为无效 ===")
for v in (0, 50, 100, 120, 130, 140):
    r = proj.ground_from_pixel(W / 2, v)
    print(f"  v={v:4d} -> {'None(无效)' if r is None else f'距离 {r[2]:.2f} m'}")

print("\n=== 4. 视场内实际地面坐标范围(角点) ===")
for u, v, tag in [(0, 719, "左下"), (1279, 719, "右下"), (0, 0, "左上"), (1279, 0, "右上")]:
    r = proj.ground_from_pixel(u, v)
    print(f"  {tag} 像素({u},{v}) -> " + ("None" if r is None else f"X={r[0]:.2f} Y={r[1]:.2f} d={r[2]:.2f}"))

print("\n=== 5. SE(2) 位姿求解（合成数据） ===")
# 假设相机(载体原点)在地图 (1.0, 2.0)、朝向 40°，看到地图上两个标志
true_th = math.radians(40.0)
true_t = np.array([1.0, 2.0])
map_pts = np.array([[3.0, 2.5], [2.2, 3.6]])
# 载体坐标系下的观测 = R^T (map - t)
R = np.array([[math.cos(true_th), -math.sin(true_th)],
              [math.sin(true_th), math.cos(true_th)]])
obs = (R.T @ (map_pts - true_t).T).T
th, t, mean_err, rms = fit_se2(obs, map_pts)
print(f"  真值: t=({true_t[0]:.2f},{true_t[1]:.2f}) yaw=40.00°")
print(f"  估计: t=({t[0]:.2f},{t[1]:.2f}) yaw={math.degrees(th):.2f}° rms={rms:.6f}")

print("\n=== 6. 单标志 -> 应当判定无效（这是考核要求的失败状态） ===")
res = solve_pose_with_matching([{"type": "X", "x": 1.8, "y": 0.1}],
                               [{"type": "X", "x": 4.0, "y": 2.0},
                                {"type": "L", "x": 0.0, "y": 0.0}])
print(f"  {res['status']}  valid={res['valid']}  marker_count={res['marker_count']}")

print("\n=== 7. L+T 两标志 -> 应给出位姿 ===")
obs2 = [{"type": "L", "x": 1.32, "y": -0.45}, {"type": "T", "x": 1.95, "y": 0.35}]
field2 = [{"type": "L", "x": 3.0, "y": 1.0}, {"type": "T", "x": 3.6, "y": 1.8}]
res2 = solve_pose_with_matching(obs2, field2, max_residual_m=0.5)
print(f"  status={res2['status']} valid={res2['valid']} "
      f"pose=({res2['x']:.2f},{res2['y']:.2f},{res2['yaw_deg']:.1f}°) rms={res2['residual']:.4f}")
